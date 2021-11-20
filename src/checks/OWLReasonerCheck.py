import rospy
import rospkg
import pymongo
import bson
import os
import re

import subprocess
import urllib
import httplib
from shutil import copyfile

from rdflib import Graph, Literal, URIRef, Namespace, BNode
from rdflib.namespace import RDF, RDFS, XSD, NamespaceManager

from NV_Check import NV_Check
from NV_msg import NV_msg, require_keys

from checks.MongoCheck import MongoCheck
from checks.RosprologCheck import RosprologCheck

class OWLReasonerCheck(NV_Check):
    """
    OWLReasonerCheck runs the OWL reasoner HermiT on the triple store.
    This includes parsing the triples, and checking for linked OWL files at the
    file server.

    Options in the workflow YAML file:

    None

    Examples:

    - OWLReasonerCheck:
        fail_response: warn
    """

    __namespaces = {}
    __is_setup = False
    __output_iri = "http://knowrob.org/tmp/neem_validator.owl"

    def __init__(self, workflow_info, quiet = False):
        if not self.__is_setup:
            self.setup()
        self.query_str = "Check OWL2 consistency of triple store"
        self.collection = "triples"
        if "fail_response" in workflow_info:
            self.msg_level = workflow_info['fail_response']
        else:
            self.msg_level = "err"

        if "fail_reason" in workflow_info:
            self.default_fail_reason = workflow_info['fail_reason']

    def child_check(self):
        msg_fail_level = self.msg_level
        self.query_str = "parse triples to generate OWL2 ontology"
        expected_result = "parse everything"
        actual_result = expected_result
        point_in_neem = None

        mongoconfig = {}
        mongoconfig['collection'] = self.collection
        mongoconfig['command'] = "find"
        mongoconfig['query'] = "{}"

        mongo_cursor = MongoCheck(mongoconfig, True).check()

        rdf_graph = Graph()

        KNOWROB = Namespace("http://knowrob.org/kb/knowrob.owl#")
        OWL = Namespace("http://www.w3.org/2002/07/owl#")
        SOMA = Namespace("http://www.ease-crc.org/ont/SOMA.owl#")
        SRDL2COMP = Namespace("http://knowrob.org/kb/srdl2-comp.owl#")
        DUL = Namespace("http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#")
        URDF = Namespace("http://knowrob.org/kb/urdf.owl#")

        ns_manager = NamespaceManager(rdf_graph)

        ns_manager.bind('knowrob', KNOWROB, override=False)
        ns_manager.bind('owl', OWL, override=False)
        ns_manager.bind('soma', SOMA, override=False)
        ns_manager.bind('srdl2comp', SRDL2COMP, override=False)
        ns_manager.bind('dul', DUL, override=False)
        ns_manager.bind('urdf', URDF, override=False)
        ns_manager.bind('xsd', XSD, override=False)

        bnodes = {}
        owl_imports = set()
        to_remove = set()

        for triple in mongo_cursor:
            try:
                # remove Ontology-Definitions
                if ("http://www.w3.org/1999/02/22-rdf-syntax-ns#type" in triple['p'] and
                    "http://www.w3.org/2002/07/owl#Ontology" in triple['o']):
                    to_remove.add(triple['s'])

                if "http://www.w3.org/2002/07/owl#imports" in triple['p']:
                    owl_imports.add(triple['o'])

                if "lang(" in str(triple['o']): # skip all language specifications
                    continue

                if "type(" in str(triple['o']): # handles something like "o" : "type('http://qudt.org/vocab/unit#Kilogram','0')"
                    #rospy.logwarn("Unknown triple type in o:")
                    #rospy.logwarn("s:"+ triple['s'] + ", p:" + triple['p'] + ", o:"+triple['o'])
                    continue

                if ("http://knowrob.org/kb/knowrob.owl#vec3" in triple['s'] or
                    "http://knowrob.org/kb/knowrob.owl#vec3" in str(triple['o'])):
                    continue # knowrob:vec3 is not properly defined and leads Hermit to crash

                s = URIRef(triple['s'])
                p = URIRef(triple['p'])
                if "#_:" in triple['s']:
                    if triple['s'] in bnodes:
                        s = bnodes[triple['s']]
                    else:
                        s = BNode()
                        bnodes[triple['s']] = s

                if "http" in str(triple['o']):
                    o = URIRef(triple['o'])
                elif type(triple['o']) is bson.decimal128.Decimal128:
                    o = Literal(triple['o'], datatype=XSD.float) # fixme could also be double!
                elif type(triple['o']) is int:
                    o = Literal(triple['o'], datatype=XSD.integer)
                elif "Infinity" in str(triple['o']):
                    o = Literal(float('inf'), datatype=XSD.float)
                else:
                    o = Literal(triple['o'])

                if "#_:" in str(triple['o']):
                    if triple['o'] in bnodes:
                        o = bnodes[triple['o']]
                    else:
                        o = BNode()
                        bnodes[triple['o']] = o

                rdf_graph.add((s, p, o))
            except:
                actual_result = "Triple could not get parsed",
                point_in_neem = "s: "+triple['s']+"; p: "+triple['p']+"; o: "+str(triple['o'])


        for rem in to_remove:
            rdf_graph.remove((URIRef(rem), None, None)) # removes all triples with triple['s'] == rem

        # Define the new ontology
        s = URIRef(self.__output_iri)

        p = RDF.type
        o = OWL.Ontology
        rdf_graph.add((s, p, o))

        p = RDFS.comment
        o = Literal("This Ontology was generated by the NEEM validator for validation purposes.")
        rdf_graph.add((s, p, o))

        local_owls = []

        for imp in owl_imports:
            p = OWL.imports
            uri = imp
            if (not "package://knowrob/" in imp) and ("package://" in imp):
                uri = "package://knowrob/owl/owl" + str(len(local_owls)) + ".owl" # give new nummeric file names to prevent problems with duplicate names
                local_owls.append(imp)
            o = URIRef(uri)
            rdf_graph.add((s, p, o))


        # todo activate this as soon as KnowRob has fixed its ontologies
        # copy owl file from KnowRob
        #path_to_knowrob = rospkg.RosPack().get_path('knowrob')
        #subprocess.call(["cp", "-rf", path_to_knowrob + "/owl", self.path_to_this_pkg + "/owl"])

        f = open(self.path_to_this_pkg + "/owl/neem-triples.owl", "w+")
        f.write(rdf_graph.serialize(format='xml'))
        f.close()

        self.msg_level = msg_fail_level
        if actual_result == expected_result:
            self.msg_level = "success"

        NV_msg(caller = self,
            expected_result = expected_result,
            actual_result = actual_result,
            point_in_neem = point_in_neem)

        ####### Download external OWL files #######
        OWLDIR = self.path_to_this_pkg + "/owl/"

        # todo add opportunity to customize this list via workflow
        owls_to_uglify = ["knowrob.owl", "URDF.owl", "srdl2.owl", "srdl2-comp.owl", "srdl2-cap.owl"]

        # remove old owl files from previous runs
        owl_file_names = os.listdir(OWLDIR)
        for file in owl_file_names:
            if re.match(r'^owl[0-9]*.owl', file):
                os.remove(OWLDIR + file)

        owls_all_present = True

        for i, owl in enumerate(local_owls):
            filename = "owl"+str(i)+".owl"
            owl_url = owl.replace("package://", "https://neem-1.informatik.uni-bremen.de/data/") # todo make url customizable

            # Check if OWL file exists
            self.query_str = "Find OWL file for " + owl + " at " + owl_url
            self.msg_level = "success"
            expected_result = "HTTP response: 200"
            fileserver = httplib.HTTPConnection("neem-1.informatik.uni-bremen.de")
            fileserver.request("HEAD", owl_url)
            fsresponse = fileserver.getresponse().status
            fileserver.close()
            actual_result = "HTTP response: " + str(fsresponse)
            if fsresponse != 200:
                self.msg_level = msg_fail_level
                owls_all_present = False
            else:
                urllib.urlretrieve(owl_url, OWLDIR+filename)
                owls_to_uglify.append(filename)
            NV_msg(self, expected_result, actual_result)

        #if not owls_all_present:
        #    raise httplib.HTTPException("OWL file for " + owl + " couldn't be found")

        owls_to_uglify.append("neem-triples.owl")

        ####### Uglify #######

        self.query_str = "uglify OWL files to run HermiT on them, using src/prolog/uglify.pl"
        expected_result = "No Errors"
        actual_result = expected_result

        path = self.path_to_this_pkg + "/src/prolog/uglify.pl"
        rosprologconfig = {}
        rosprologconfig['solutions'] = 1
        rosprologconfig['look_for_var'] = "Error"
        rosprologconfig['expect_var_to_be'] = "_"
        rosprologconfig['query'] = "catch(load_files('" + path + "'), Error, true)."
        result = RosprologCheck(rosprologconfig, True).check()
        if result != "_":
            actual_result = "loading src/prolog/uglify.pl produced this error:\n"+result

        rosprologconfig['query'] = "catch(uglify("+ str(owls_to_uglify) +"), Error, true)."
        result = RosprologCheck(rosprologconfig, True).check()
        if result != "_":
            actual_result = "running uglify("+str(owls_to_uglify)+"). produced this error:\n"+result

        self.msg_level = msg_fail_level
        if actual_result == expected_result:
            self.msg_level = "success"

        NV_msg(caller = self,
            expected_result = expected_result,
            actual_result = actual_result)

        path_hermit = self.path_to_this_pkg + "/hermit"
        path_logs = self.path_to_this_pkg + "/logs"

        subprocess.call(["sed", "-i", "s/>inf</>INF</", path_hermit+"/knowrob.owl"])
        subprocess.call(["sed", "-i", "s/\&xsd;double/\&xsd;float/", path_hermit+"/knowrob.owl"])

        copyfile(path_hermit + "/knowrob.owl", path_logs + "/uglified.owl")

        ####### Starting Hermit Docker #######

        self.msg_level = msg_fail_level
        self.query_str = "check consistency of triple store"
        expected_result = "is consistent"
        actual_result = expected_result
        possible_reason = None

        ctn_name = "last-hermit-neem-validator"
        subprocess.call(["docker", "build", "-t", "hermit-neem-validator", path_hermit])

        try:
            rospy.loginfo("starting HermiT reasoner...")
            subprocess.check_output(
                ["docker", "run", "--name", ctn_name,
                "hermit-neem-validator", "knowrob.owl"], stderr=subprocess.STDOUT)
            subprocess.call(["docker", "start", ctn_name])
            subprocess.call(["docker", "cp", ctn_name+":/hermit/workspace/hermit.output", path_logs+"/hermit.output"])
            rospy.loginfo("hermit.output is now locally available.")
        except subprocess.CalledProcessError as e:
            failed = True
            if "InconsistentOntologyException" in e.output:
                actual_result = "HermiT OWL Reasoner throws InconsistentOntologyException"
                possible_reason = "Try opening the generated uglified logs/uglified.owl file in Protege and run an OWL reasoner in order to get more details"
            else:
                actual_result = "HermiT threw the following exception: " + e.output
                possible_reason = None
            NV_msg(caller = self,
                expected_result = expected_result,
                possible_reason = possible_reason,
                actual_result = actual_result)
            return actual_result

        finally:
            # clean up docker containers
            subprocess.call(["docker", "kill", ctn_name])
            subprocess.call(["docker", "rm", ctn_name])

        # todo remove this unstable python3 call!
        try:
            hermit_output = subprocess.check_output(
                    ["python3", self.path_to_this_pkg+"/scripts/hermit_test.py",
                    "-o", path_logs+"/hermit.html", path_logs+"/hermit.output"])
            possible_reason = "HermiT returned: \n"+hermit_output
        except subprocess.CalledProcessError as e:
            actual_result = "HermiT found some issues in the ontology. You can find the uglified ontology at logs/uglified.owl:\n" + e.output

        self.msg_level = msg_fail_level
        if actual_result == expected_result:
            self.msg_level = "success"

        NV_msg(caller = self,
            expected_result = expected_result,
            possible_reason = possible_reason,
            actual_result = actual_result)

        return actual_result

    def setup(self):
        MongoCheck.__is_setup = True
