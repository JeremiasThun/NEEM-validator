#!/usr/bin/env python
import rospy
import rospkg
from std_msgs.msg import String

import yaml
import os
import sys
import timeit
import importlib
import signal

from ansi2html import Ansi2HTMLConverter

from checks.NV_Check import NV_Check
from checks.NV_msg import require_keys, NV_msg, NV_log, NV_exception, CheckFailedError

def create_check(check_name, config):
    obj = __get_check_object(check_name)
    if not obj:
        raise ValueError(check_name + " was referenced in the workflow but has no corresponding Check-class")
    return obj(config)

def create_settings(check_name, settings):
    obj = __get_check_object(check_name)
    if not obj:
        raise ValueError(check_name + " was referenced in settings but has no corresponding Check-class")
    obj.settings = settings

# maps all Check-names in yaml file to respective Python classes
def __get_check_object(check_name):
    check_module = importlib.import_module("checks." + check_name)
    return getattr(check_module, check_name)

# build the pipeline
def validator(logger, shutdown):
    neem_valid = True
    rospy.init_node('neem_validator', anonymous=True)
    logging = rospy.get_param('~logging')
    logger.activate_logger(logging)
    global path_to_this_pkg
    path_to_this_pkg = rospkg.RosPack().get_path('neem_validator')
    global path_to_neem
    path_to_neem = rospy.get_param('~path_to_neem')
    path_to_workflow = rospy.get_param('~workflow_file')

    NV_Check.path_to_neem = path_to_neem
    NV_Check.path_to_this_pkg = path_to_this_pkg

    # Apply Settinggs
    settings = yaml.safe_load(open(path_to_this_pkg + "/settings/settings.yaml"))

    for check_name in settings:
        create_settings(check_name, settings[check_name])

    # Create Workflow
    workflow = yaml.safe_load(open(path_to_this_pkg + "/" + path_to_workflow))

    # Get Stages
    stages = []
    for entry in workflow:
        if "Stages" in entry:
            if stages:
                raise ValueError("Your workflow file can have only one entry 'Stages'.")
            stages = entry["Stages"]

    if not stages:
        raise ValueError("Your workflow file needs to have an entry 'Stages'.")

    # Write default stage names
    for entry in workflow:
        if "Stages" in entry:
            continue
        for check, arguments in entry.iteritems(): # this has exactly one entry
            try:
                if not "stage" in arguments:
                    arguments['stage'] = check
            except TypeError:
                raise TypeError("Your workflow file seems to be malformed.")


    # Bring in order
    ordered_workflow = []
    for stage in stages:
        for entry in workflow:
            if "Stages" in entry:
                continue
            for arguments in entry.values(): # this has exactly one entry
                if stage == arguments['stage']:
                    ordered_workflow.append(entry)

    # Instanciate Check classes
    checks = []
    for entry in ordered_workflow:
        for check, arguments in entry.iteritems(): # this has exactly one entry
            check_obj = create_check(check, arguments)
            checks.append(check_obj)

    logger.log("Parsing and setting up the pipeline")

    # Call Check instances
    for check in checks:
        try:
            check.check()
            logger.log("finish " + check.me(), check.get_query_str())
        except CheckFailedError:
            raise
        except Exception as e:
            if shutdown.killing:
                rospy.logerr("User requested shutdown. Shutting down...")
                raise
            NV_exception(check, e)
            if check.msg_level == "fatal":
                raise

class ShutdownDetector(object):
    killing = False
    def __init__(self):
        signal.signal(signal.SIGINT, self.exit)
        signal.signal(signal.SIGTERM, self.exit)

    def exit(self, sig, frame):
        ShutdownDetector.killing = True


if __name__ == '__main__':
    shutdown = ShutdownDetector()
    logger = NV_log()
    try:
        validator(logger, shutdown)
    except rospy.ROSInterruptException:
        pass
    except:
        NV_msg.got_errors = True
        raise
    finally:
        logger.write_logs(path_to_this_pkg + "/logs/time")

        output = "### Validation Results for NEEM at: " + path_to_neem + " ###\n\n"
        for message in NV_msg.all_generated_messages:
            output += message + "\n\n"

        bash_file = open(path_to_this_pkg + "/logs/validation_results", "w+")
        bash_file.write(output)
        bash_file.close()

        conv = Ansi2HTMLConverter()
        html_output = conv.convert(output)
        html_file = open(path_to_this_pkg + "/logs/validation_results.html", "wp")
        html_file.write(html_output)
        html_file.close()

        f = open(path_to_this_pkg + "/logs/errors", "w+")
        if NV_msg.got_errors:
            f.write("1")
        else:
            f.write("0")
        f.close()
