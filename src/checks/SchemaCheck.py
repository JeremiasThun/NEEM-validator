import rospy
import json
import os.path

from NV_Check import NV_Check
from NV_msg import NV_msg, require_keys

from checks.MongoCheck import MongoCheck

class SchemaCheck(NV_Check):
    """
    The SchemaCheck looks for entries in the MongoDB that don't satisfy
    a specified schema.

    Options in the workflow YAML file:

    - file: The name of the json file that defines the schema.
    --> Required
    - collection: The MongoDB collection that has to satisfy the linked schema.
    --> Required

    Examples:

    YAML-Code for example:

    - SchemaCheck:
        file: tf_schema.json
        collection: tf
        fail_response: err

    - SchemaCheck:
        file: triples_schema.json
        collection: triples
        fail_response: err

    """

    __is_setup = False
    __local_schema_path = ""

    def __init__(self, workflow_info):
        if not self.__is_setup:
            self.setup()
        require_keys(self.me(), "workflow", ['file', 'collection'], workflow_info)
        self.file = self.__local_schema_path + "/" + workflow_info['file']
        self.collection = workflow_info['collection']
        self.query_str = "Check schema at " + self.file + " for collection " + self.collection
        if "fail_response" in workflow_info:
            self.msg_level = workflow_info['fail_response']
        else:
            self.msg_level = "err"
        self.failed = False

        if "fail_reason" in workflow_info:
            self.default_fail_reason = workflow_info['fail_reason']

    def child_check(self):
        f = open(self.file)
        schema = json.load(f)

        # Every entry in the schema gets two checks:
        # 1. Does the entry exist?
        # 2. Does the entry have the correct data type?
        # The checks are seperate in order to give the user a detailed
        # error response in case the check fails
        #
        # Every check is internally handled as a new instance of MongoCheck
        self.generate_next_mongocheck(schema)

        if not self.failed:
            self.msg_level = "success"
            expected_result = "All entries of " + self.collection + " satisfy the schema."
            NV_msg(caller = self,
                expected_result = expected_result,
                actual_result = expected_result)

    def generate_next_mongocheck(self, schema, key_prefix = ""):
        """
        Iteratively creates two MongoCheck instances for every entry in the
        schema. One MongoCheck is for checking if the key exists and a second
        is created for checking whether all entries for this key satisfy the
        specifications in the schema.

        @param  schema: The schema that is to be checked
        @type   schema: dictionary 
        """
        while True: # loops destructively through list schema
            try:
                entry = schema.popitem()
                schema_key = key_prefix + entry[0]
                schema_type = entry[1]

                mongoconfig = {}
                mongoconfig['collection'] = self.collection
                mongoconfig['fail_response'] = self.msg_level
                # check if the field exists for all entries
                mongoconfig['command'] = "find_one"
                mongoconfig['query'] = "{ '"+schema_key+"' : {'$exists': False}}"
                mongoconfig['expected_result'] = "None"
                checking_result = MongoCheck(mongoconfig, True).check() # find incorrect entries in MongoDB

                if not checking_result is None:
                    NV_msg(caller = self,
                        expected_result = "no entries without key: " + schema_key,
                        actual_result = checking_result,
                        possible_reason = "key " + schema_key + " is missing")
                    self.failed = True

                # are there sub-keys?
                if isinstance(schema_type, dict):
                    self.generate_next_mongocheck(schema_type, schema_key + ".")
                else:
                    # check if the field always has the correct type
                    if isinstance(schema_type, list): # a list of types is allowed
                        mongoconfig['query'] = "{'$and': ["
                        first = True
                        for possible_type in schema_type:
                            if first:
                                first = False
                            else:
                                mongoconfig['query'] += ", "
                            mongoconfig['query'] += "{ '"+schema_key+"' : {'$not': {'$type': '"+possible_type+"'}}}"
                        mongoconfig['query'] += "]}"
                    elif schema_type == "*": # all types are allowed
                        return
                    else:
                        mongoconfig['query'] = "{ '"+schema_key+"' : {'$not': {'$type': '"+schema_type+"'}}}"

                    checking_result = MongoCheck(mongoconfig, True).check()

                    if not checking_result is None:
                        NV_msg(caller = self,
                            expected_result = "no entries for " + schema_key + " with type other than " + str(schema_type),
                            actual_result = checking_result)
                        self.failed = True

            except KeyError: # list is empty
                return


    def setup(self):
        require_keys(self.me(), "settings", ["local_schema_path"], self.settings)
        SchemaCheck.__local_schema_path = self.path_to_this_pkg + "/" + self.settings["local_schema_path"]
        SchemaCheck.__is_setup = True
