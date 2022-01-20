import rospy
import os
import json
import re

from NV_Check import NV_Check
from NV_msg import NV_msg, require_keys
from prolog.prolog_query_client import PrologQuery

class RosprologCheck(NV_Check):
    """
    RosprologCheck runs prolog queries via the Rosprolog service.

    Options in the workflow YAML file:

    - query: Any valid Prolog query
    --> Required
    - look_for_var: Variable inside the Prolog query that should be evaluated
    --> Default: Return all variables as dictionary
    - expect_var_to_be: Expected content of the variable look_for_var. For the check to succeed, the first n bindings for this variable all have to comply with this content. n being the number in solutions.
    - use_regex: If True, the content of expect_var_to_be is interpreted as regular expression
    - solutions: The number of bindings thatt should be checked and displayed. The string 'all' may be inserted to check all solutions.
    --> Default: 1

    Examples:

    - RosprologCheck:
        stage: semantic_annotation
        query: is_action(Action), \+ has_time_interval(Action, _)
        look_for_var: Action
        solutions: 5
        expect_var_to_be: False
        fail_response: err
        fail_reason: These actions don't have a time interval.

    - RosprologCheck:
        stage: mongorestore
        query: member(Bson, $$FOUND_BSONS), nv_mng_restore(Bson, Response).
        look_for_var: Response
        use_regex_in_result: True
        expect_var_to_be: .\d+\s\D+0\s\D+
        fail_response: fatal
        solutions: all

    """

    pq = PrologQuery()
    __is_setup = False
    paths_to_pl = None

    def __init__(self, workflow_info, quiet = False):
        if not self.__is_setup:
            self.setup()
        self.quiet = quiet
        self.required_keys = ["query"]
        require_keys(self.me(), "workflow", self.required_keys, workflow_info)
        self.query = workflow_info["query"]
        self.query_str = self.query
        self.use_regex_in_result = False
        if "use_regex_in_result" in workflow_info:
            if workflow_info['use_regex_in_result']:
                self.use_regex_in_result = True
        if "solutions" in workflow_info:
            self.solutions = workflow_info['solutions']
            if self.solutions == "all":
                self.solutions = -1
        else:
            self.solutions = 1
        if "look_for_var" in workflow_info:
            self.look_for_var = workflow_info['look_for_var']
        else:
            self.look_for_var = None
        if "expect_var_to_be" in workflow_info:
            self.expect_var_to_be = workflow_info['expect_var_to_be']
        else:
            self.expect_var_to_be = None
        if "fail_response" in workflow_info:
            self.msg_level = workflow_info["fail_response"]
        else:
            self.msg_level = "err"
        if "fail_reason" in workflow_info:
            self.default_fail_reason = workflow_info['fail_reason']

    def child_check(self):
        if self.look_for_var:
            expected_result = self.look_for_var + " = " + str(self.expect_var_to_be)
        if self.pq.post_query(self.query):
            solution_obj = self.pq.get_solutions(self.solutions)
        else:
            raise ConnectionError("failed to connect to rosprolog service. Wanted to run RosprologCheck with this query: " + self.query)

        if not self.look_for_var:
            self.msg_level = "success"
            NV_msg(self, solution_obj, solution_obj)
            return solution_obj

        if not solution_obj:
            if self.expect_var_to_be == False:
                self.msg_level = "success"
            NV_msg(self, expected_result, False)
            return False

        solution = solution_obj
        if self.solutions == 1:
            solution = solution_obj[0][self.look_for_var]
        failed = False
        for sol in solution_obj:
            if not self.compare_solutions(sol[self.look_for_var]):
                failed = True
        if not failed:
            self.msg_level = "success"
        NV_msg(self, expected_result, solution)
        return solution

    def setup(self):
        if not "local_prolog_modules" in RosprologCheck.settings:
            RosprologCheck.__is_setup = True
            return
        RosprologCheck.paths_to_pl = RosprologCheck.settings['local_prolog_modules']
        # todo allow multiple paths
        path = self.path_to_this_pkg + "/" + RosprologCheck.paths_to_pl.pop()
        # todo implement a catch in prolog
        self.query_str = "load_files('" + path + "')"
        self.msg_level = "err"
        try:
            if self.pq.post_query(self.query_str) and self.pq.get_solutions(1):
                rospy.logout(self.query_str + " successfully executed.")
            else:
                NV_msg(caller = self,
                    expected_result = "Success",
                    actual_result = "Failure",
                    possible_reason = "Path to prolog file ("+path+") could be wrong...")
        except ValueError as e:
            NV_msg(caller = self,
                expected_result = "Success",
                actual_result = "Failure",
                possible_reason = "Path to prolog file ("+path+") could be wrong...")

        if RosprologCheck.paths_to_pl: # iterate until all modules are loaded.
            self.load_custom_pl()

        RosprologCheck.__is_setup = True

    def compare_solutions(self, actual_result):
        """
        compare the binding in actial_result with the expected result.
        """
        if self.use_regex_in_result:
            if re.match(self.expect_var_to_be, actual_result):
                return True
            else:
                return False
        else:
            if actual_result == self.expect_var_to_be:
                return True
            else:
                return False
