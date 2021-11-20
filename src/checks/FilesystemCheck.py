import rospy
import os
from pprint import pformat

from NV_Check import NV_Check
from NV_msg import NV_msg, require_keys

class FilesystemCheck(NV_Check):
    """
    FilesystemCheck goes through the uploaded NEEM files and checks if all the
    specified files are present.

    Options in the workflow YAML file:

    - file_to_look_for: A file name (without extension) that should be present somewhere inside the directory.
    --> Default: all files (with specified extension)
    - extension: The file extension that should be present somewhere inside the directory
    --> Default: Default: .bson AND .metadata.json

    Examples:

    - FilesystemCheck:
        file_to_look_for: triples
        fail_response: err

    - FilesystemCheck:
        file_to_look_for: README
        fail_response: warn
        extension: md
    """

    __is_setup = False
    __found_files = []

    def __init__(self, workflow_info):
        if not self.__is_setup:
            self.setup()
        if "file_to_look_for" in workflow_info:
            self.file_to_look_for = workflow_info["file_to_look_for"]
        else:
            self.file_to_look_for = None
        if "store_result_as" in workflow_info:
            self.store_result_as = workflow_info['store_result_as']
        if "fail_response" in workflow_info:
            self.msg_level = workflow_info["fail_response"]
        else:
            self.msg_level = "err"
        if "extension" in workflow_info:
            self.extension = workflow_info["extension"]
            self.look_for_metadata = False
        else:
            self.extension = ".bson"
            self.look_for_metadata = True
        if "fail_reason" in workflow_info:
            self.default_fail_reason = workflow_info['fail_reason']

    def child_check(self):
        if not FilesystemCheck.__found_files:
            # This is true only for the first instance's check()
            for root, dirs, files in os.walk(self.path_to_neem):
                for file in files:
                    FilesystemCheck.__found_files.append(os.path.join(root, file))

        found_file = False
        found_metadata = False
        found_files_match_criteria = []

        for file_path in FilesystemCheck.__found_files:
            file = file_path.split("/")[-1] # get only filename
            if self.file_to_look_for:
                if file == self.file_to_look_for + self.extension:
                    found_files_match_criteria.append(file_path)
                    found_file = True
                    if not self.look_for_metadata or found_metadata:
                        break
            else:
                if file.endswith(self.extension):
                    found_files_match_criteria.append(file_path)
            if self.look_for_metadata and not found_metadata:
                if file == self.file_to_look_for + ".metadata.json":
                    found_files_match_criteria.append(file_path)
                    found_metadata = True
                    if found_file:
                        break

        # evaluate return value
        if self.look_for_metadata:
            self.query_str = "find files " + self.file_to_look_for +\
                    ".bson and " + self.file_to_look_for +\
                    ".metadata.json in path: " + self.path_to_neem
            expected_result = "find both files"
            if found_file and found_metadata:
                actual_result = "found both files"
                self.msg_level = "success"
            elif found_file:
                actual_result = "found " + self.file_to_look_for + ".bson but could not find " +\
                self.file_to_look_for + ".metadata.json"
            elif found_metadata:
                actual_result = "found " + self.file_to_look_for + ".metadata.json but could not find " +\
                self.file_to_look_for + ".bson"
            else:
                actual_result = "no corresponding files found."
        elif self.file_to_look_for:
            self.query_str = "look for file " + self.file_to_look_for + self.extension +\
                    " in path: " + self.path_to_neem
            expected_result = "find file"
            if found_file:
                self.msg_level = "success"
                actual_result = "found file"
            else:
                actual_result = "could not find file"

        else:
            self.query_str = "look for files with extension: " + self.extension +\
                    " in path: " + self.path_to_neem
            expected_result = "find some files"
            if found_files_match_criteria:
                actual_result = self.store_result_as + " = " + pformat(found_files_match_criteria)
                self.msg_level = "success"
            else:
                actual_result = "no files found"

        NV_msg(self, expected_result, actual_result)

        if self.msg_level == "success":
            return found_files_match_criteria
        else:
            return False

    def setup(self):
        FilesystemCheck.__is_setup = True
