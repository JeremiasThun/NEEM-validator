import rospy
import json
import inspect
import sys
import traceback
import timeit
from pprint import pformat

class NV_msg:
    """
    An instance of NV_msg is a message. Upon instanciation, the message gets
    printed to rosout. All created messages are saved and can be bulk printed.
    """
    all_generated_messages = []
    got_errors = False
    got_warnings = False

    def __init__(self, caller, expected_result, actual_result, possible_reason = None, point_in_neem = None):
        """
        @param  caller: the check instance that is creating this message
        @type   instance of a NV_Check class
        @param  expected_result: the expected result of this check object
        @type   object
        @param  actual_result: the actual result of this check object
        @type   object
        @param  possible_reason: the most common reason for why the check failed
        @type   string
        @param  point_in_neem: the point in the NEEM where the error lies that should be fixed
        @type   string
        """
        self.check_name = caller.me()
        self.query = caller.get_query_str()
        self.msg_level = caller.get_msg_level()
        self.expected_result = pformat(expected_result)
        if (type(actual_result) == list and len(actual_result) > 10):
            self.actual_result = pformat(actual_result[:10])
            self.actual_result += "\n...and " + str(len(actual_result)-10) + " more!"
        else:
            self.actual_result = pformat(actual_result)
        self.possible_reason = possible_reason
        if not self.possible_reason:
            self.possible_reason = caller.default_fail_reason
        self.point_in_neem = point_in_neem

        if not caller.quiet:
            self.rosprint_message()

    def rosprint_message(self):
        """
        Prints the generated message to rosout and appends it to
        self.all_generated_messages.
        """
        if (self.msg_level == "err" or self.msg_level == "fatal"):
            NV_msg.got_errors = True
            output = "\033[0;31m[" + self.check_name + "] ERROR\n"
            output += "When executing query: " + self.query + "\n"
            output += "Expected: " + self.expected_result + "\n"
            output += "Got: " + self.actual_result + "\n"
            if self.point_in_neem:
                output += "At: " + self.point_in_neem + "\n"
            if self.possible_reason:
                output += "Possible reason: " + self.possible_reason + "\n"
            output += "=====\033[0m"
            rospy.logerr(output)
            NV_msg.all_generated_messages.append(output)

            if self.msg_level == "fatal":
                raise CheckFailedError(self.check_name)

        if self.msg_level == "warn":
            NV_msg.got_warnings = True
            output = "\033[0;33m[" + self.check_name + "] WARNING\n"
            output += "When executing query: " + self.query + "\n"
            output += "Expected: " + self.expected_result + "\n"
            output += "Got: " + self.actual_result + "\n"
            if self.point_in_neem:
                output += "At: " + self.point_in_neem + "\n"
            if self.possible_reason:
                output += "Possible reason: " + self.possible_reason + "\n"
            output += "=====\033[0m"
            rospy.logwarn(output)
            NV_msg.all_generated_messages.append(output)

        if self.msg_level == "success":
            output = ""
            if len(self.query) <= 70:
                output += "[" + self.check_name +\
                    "] Query: " + self.query +\
                    "\033[0;32m => " + self.actual_result + "\033[0m"
            else:
                output += "[" + self.check_name +\
                    "] Query: " + self.query + "\n"
                output += "[" + self.check_name +\
                    "]\033[0;32m => " + self.actual_result + "\033[0m"
            rospy.loginfo(output)
            NV_msg.all_generated_messages.append(output)

    def get_message(self):
        """
        Creates json encoded version of the message.
        @return: JSON
        """
        return json.dumps({'Check': self.check_name,
                            'Query': self.query,
                            'Expected result': self.expected_result,
                            'Actual result': self.actual_result,
                            'Message level': self.msg_level,
                            'Possible reason(s)': self.possible_reason,
                            'Point in NEEM': self.point_in_neem})


class NV_exception:
    """
    Creates an exception output similar to NV_msg. This includes a
    traceback of the last calls.
    """
    def __init__(self, check, exception):
        NV_msg.got_errors = True
        output = "\033[0;31m[" + check.me() + "] PYTHON EXCEPTION\n"
        output += str(exception) + "\033[0m\n"
        rospy.logerr(output)
        NV_msg.all_generated_messages.append(output)
        # for debugging purposes
        exc_type, exc_value, exc_traceback = sys.exc_info()
        traceback.print_exception(exc_type, exc_value, exc_traceback,
                                  limit=5, file=sys.stdout)


class NV_log:
    """
    NV_log is a logger. Upon instanciation, a timer is started.
    Log messages are created by calling the log() method
    """
    def __init__(self):
        self.logging = True
        self.logs = []
        self.START = timeit.default_timer()
        self.last = self.START
        rospy.logout("start logging: " + str(self.START))

    # activate: bool
    def activate_logger(self, activate):
        """
        Activates or deactivates the logger. With a deactivated logger, all
        other methods do nothing.

        @param activate: Should the logger be activated?
        @type activate: bool
        """
        rospy.logout(activate)
        self.logging = activate

    def log(self, message, details=None):
        """
        Creates a log message. The time passed since starting the logger and
        the time passed since the last log message got created are automaically
        stored aswell.
        The log message as well as the time info are printed to rosout.
        """
        if self.logging:
            self.now = timeit.default_timer()
            text = str(round(self.now-self.last, 3)) + " sec since last entry | "
            text += str(round(self.now-self.START, 3)) + " sec since start | "
            text += message
            rospy.loginfo(text)
            if details:
                text += " | " + details
            self.logs.append(text)
            self.last = self.now

    def write_logs(self, file):
        """
        Writes all created log messages together with their time info to
        the file at specified file path

        @param file: File path where to store log messages
        @type file: string
        """
        if self.logging:
            f = open(file, "w+")
            for log in self.logs:
                f.write(log + "\n")
            f.close()

class CheckFailedError(AssertionError):
    """
    Custom Exception for cases where a NV_Check failed with fail_response=fatal
    """
    def __init__(self, name):
        self.message = name + " was required to pass but didn't"
        super(CheckFailedError, self).__init__(self.message)


class MissingKeyEror(KeyError):
    """
    Custom Exception for cases where the workflow yaml is missing a required key
    """
    def __init__(self, caller, dict, missing_key, context = None, stack_nr = 1):
        if dict == "settings":
            path = "setings/settings.yaml"
        if dict == "workflow":
            path = "settings/pipeline_workflow.yaml"
        if dict == "env-var":
            path = "your environment variables"
        self.message = caller + " requires you to specify the following key in "+path+": " + missing_key
        rospy.logerr(self.message)
        if context:
            rospy.logerr("Context: "+str(context))
        rospy.logerr("Used in: " + str(inspect.stack()[stack_nr][1])+", line: "+str(inspect.stack()[stack_nr][2])+" context: "+ str(inspect.stack()[stack_nr][3]))
        super(MissingKeyEror, self).__init__(self.message)

def require_keys(caller, file, keys, dict):
    """
    Checks if all keys are present in the dictionary.
    Throws an MissingKeyEror if keys are not present in dict.
    The error prints out caller and file for debugging reasons.

    @param  caller: the check instance that is creating this message
    @type   caller: instance of a NV_Check class
    @param  file: the name of the YAML file that is validated
    @type   file: string
    @param  keys: The keys that are required to be present in the dictionary
    @type   keys: list
    @param  dict: The dictionary that should contain all the keys
    @type   dict: dictionary
    """
    for key in keys:
        if not key in dict:
            raise MissingKeyEror(caller, file, key, dict, 2)
