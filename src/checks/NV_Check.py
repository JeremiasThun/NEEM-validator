import re
import os

from checks.NV_msg import NV_msg

import rospy

class NV_Check:
    """
    Abstract check class that every check has to inherit from.
    """
    myname = "" # default: return class name
    path_to_neem = None
    path_to_this_pkg = None
    is_setup = False
    store_result_as = None
    settings = {}
    query_str = ""
    quiet = False
    msg_level = "err"
    default_fail_reason = None

    # make it an abstract class
    def __init__(self):
        """
        Stores the specified attributes in this check. Runs setups for this
        instance. Also runs setup for the class by calling setup() method
        if it is not set up yet.
        """
        if type(self) is Base:
            raise NotImplementedError("NV_Check is an abstract class. You have to create an actual check.")

    # Replaces variables with their content,
    # calls check()
    # and stores the result as env variable if needed
    def check(self):
        """
        This method should be called to execute a check.
        It loooks for used environment variables, indicated by `$$`, and
        substitutes them.
        If the store_result_as attribute is set, this method also stores the
        check's result in the specified variable.
        Apart from handling environment variables, this method calls the
        child_check() method, implemented by children of this class.
        """
        # look for attributes with variables that should be replaces
        for var, content in vars(self).iteritems():
            if type(content) == str:
                if "$" in content:
                    newstring = re.sub(r'\$\$\w*', self.__look_in_env, content)
                    setattr(self, var, newstring)

        result = self.child_check()

        if self.store_result_as:
            os.environ[self.store_result_as] = str(result)

        return result

    def __look_in_env(self, var):
        """
        Looks for a variable name in unix's environment variables.
        If the variable is set, this method returns the content of
        tha variable. Otherwise, it returns the name of the variable lead
        by the double dollar sign.

        @param  var: the name of the variable
        @type   var: string
        """
        var_name = var.group(0)[2:] # .group(0) returns the variable with leading $$
        var_content = os.environ.get(var_name)
        if not var_content:
            old_query_str = self.query_str
            old_msg_level = self.msg_level
            self.query_str = "Use variable $" + var_name
            expected_result = "Find variable in environment variables"
            actual_result = "Fount no corresponding environment variable. Will not substitute variable..."
            self.msg_level = "warn"
            NV_msg(self, expected_result, actual_result)
            var_content = "$$" + var_name
            self.query_str = old_query_str
            self.msg_level = old_msg_level
        return var_content

    def child_check(self):
        """
        Runs the check with the already stored parameters
        """
        raise Exception("Method __child_check() for " + str(type(self)) + " needs to be implemented.")

    def setup(self):
        """
        Stores class-wide configurations. Runs queries that are necessary to
        setup the class. Especially interprets the settings.yaml specifications.
        """
        raise Exception("Method setup() for " + str(type(self)) + " needs to be implemented.")

    def set_name(self, name):
        """
        Set the name for this instance. This determins the name that is
        displayed in the NV_msg message

        @param  name: the name
        @type   name: string
        """
        self.myname = name

    def me(self):
        """
        Returns the name of this instance.

        @returns: string
        """
        if not self.myname:
            return str(self.__class__.__name__)
        else:
            return self.myname

    def get_query_str(self):
        """
        Returns a string representation of the query

        @returns: string
        """
        return str(self.query_str)

    def get_msg_level(self):
        """
        Returns the message type of this instance.
        Upon instanciation, this is equal to the fail_response, specified in
        the workflow file. If the check succeeds, this changes to "success".

        @returns: string
        """
        return self.msg_level

    def get_quiet(self):
        """
        Returns if the instance is run in quiet mode. The quiet mode determines
        whether messages should be created or not.

        @returns: bool
        """
        return self.quiet
