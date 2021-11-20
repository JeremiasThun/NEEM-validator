import rospy
import httplib

from NV_Check import NV_Check
from NV_msg import NV_msg, require_keys

from checks.RosprologCheck import RosprologCheck

class FileserverCheck(NV_Check):
    """
    FilesystemCheck goes through the uploaded NEEM files and checks if all the
    specified files are present.

    Options in the workflow YAML file:

    - find_FILES_in: The Prolog query that should be run to find a list of files. The value of the variable FILES will be interpreted as a list of file names. These file names are expected to be present on the file server specified in settings.yaml.
    --> Required

    Examples:

    - FileserverCheck:
        find_FILES_in: has_kinematics_file(_, FILES, 'URDF')
        fail_response: err
    """

    __host = ""
    __kinematics_path = ""
    __is_setup = False

    def __init__(self, workflow_info):
        if not self.__is_setup:
            self.setup()
        require_keys(self.me(), "workflow", ['find_FILES_in'], workflow_info)
        self.prolog_query = workflow_info['find_FILES_in']
        if "fail_response" in workflow_info:
            self.msg_level = workflow_info['fail_response']
        else:
            self.msg_level = "err"
        if "fail_reason" in workflow_info:
            self.default_fail_reason = workflow_info['fail_reason']

    def child_check(self):
        rosprologconfig = {}
        rosprologconfig['query'] = self.prolog_query
        rosprologconfig['look_for_var'] = "FILES"
        rosprologconfig['expect_var_to_be'] = ""
        rosprologconfig['solutions'] = "all"

        failed = False

        self.query_str = "Lookup the existance of urdf files that are linked in the NEEM: " + self.prolog_query
        expected_result = "all linked urdf files are present on the fileserver"
        actual_result = expected_result
        possible_reason = None
        point_in_neem = None

        urdf_files = RosprologCheck(rosprologconfig, True).check()

        if not urdf_files:
            actual_result = "Could not find any links to URDF files in NEEM"
            failed = True
            possible_reason = self.prolog_query + " => false"

        if not failed:
            for urdf_result in urdf_files:
                urdf_file = urdf_result['FILES']
                fileserver = httplib.HTTPConnection(self.__host)
                filepath = self.__kinematics_path + urdf_file + ".urdf"
                fileserver.request("HEAD", filepath)
                fsresponse = fileserver.getresponse().status
                fileserver.close()
                if fsresponse != 200:
                    failed = True
                    actual_result = "couldn't find " + urdf_file + ".urdf on the fileserver: " + filepath
                    possible_reason = "Did you forget to upload the urdf file to the fileserver? The fileserver returned HTTP code " + str(fsresponse)
                    point_in_neem = "You can find the object by running the following query in rosprolog: has_kinematics_file(Object, '"+urdf_file+"', 'URDF')"


        if not failed:
            self.msg_level = "success"
        NV_msg(self, expected_result, actual_result, possible_reason, point_in_neem)

        return actual_result

    def setup(self):
        require_keys(self.me(), "settings", ["host", "kinematics_path"], self.settings)
        FileserverCheck.__host = self.settings['host']
        FileserverCheck.__kinematics_path = self.settings['kinematics_path']
        FileserverCheck.__is_setup = True
