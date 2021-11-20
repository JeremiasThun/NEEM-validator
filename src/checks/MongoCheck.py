import rospy
import pymongo
import os

from NV_Check import NV_Check
from NV_msg import NV_msg, require_keys

from checks.RosprologCheck import RosprologCheck

class MongoCheck(NV_Check):
    """
    MongoCheck runs queries in KnowRob's MongoDB.

    Options in the workflow YAML file:

    - collection: The Mongo collection to interact with.
    --> Required
    - command: The command to run on the specified collection. The most common command is probably: find.
    --> Required
    - query: The query to apply to the command. The empty set finds everything.
    --> Default: {}

    Examples:

    - MongoCheck:
        collection: triples
        command: drop
        fail_response: warn

    - MongoCheck:
        collection: tf
        command: find_one
        query: { 'header': {'$exists': False} }
        expected_result: None
        fail_response: err
    """
    
    __client = None
    __db = None
    __db_str = ""
    __is_setup = False

    def __init__(self, workflow_info, quiet = False):
        if not self.__is_setup:
            self.setup()
        require_keys(self.me(), "workflow", ['collection', 'command'], workflow_info)
        self.collection = workflow_info['collection']
        self.command = workflow_info['command']
        self.quiet = quiet
        if "query" in workflow_info:
            self.query = workflow_info['query']
        else:
            self.query = "{}"
        if "expected_result" in workflow_info:
            self.expected_result = workflow_info['expected_result']
        else:
            self.expected_result = None
        if "fail_response" in workflow_info:
            self.msg_level = workflow_info['fail_response']
        else:
            self.msg_level = "err"

        if "fail_reason" in workflow_info:
            self.default_fail_reason = workflow_info['fail_reason']

    def child_check(self):
        self.query_str = "db." + self.collection + "." + str(self.command)+"("+str(self.query)+") in db " + self.__db_str
        col = self.__db[self.collection]

        if self.command == "find":
            result = []
            post_iterator = col.find(eval(self.query))
            for post in post_iterator:
                result.append(post)

        else:
            command_method = getattr(col, self.command)
            result = command_method(eval(self.query))

        if result == self.expected_result or self.expected_result == "None" and result is None:
            self.msg_level = "success"

        NV_msg(self, self.expected_result, result)

        return result

    def setup(self):
        # default values
        port = 27017
        ip = "localhost"

        rosprologconfig = {}
        rosprologconfig['query'] = "setting(mng_client:db_name, DBName)."
        prolog_result = RosprologCheck(rosprologconfig, True).check()
        db = prolog_result[0]['DBName']

        # get MongoDB credentials from environment variables as KnowRob expects them
        ip = os.environ.get('KNOWROB_MONGO_HOST')
        user = os.environ.get('KNOWROB_MONGO_USER')
        password = os.environ.get('KNOWROB_MONGO_PASS')
        port = os.environ.get('KNOWROB_MONGO_PORT')
        if port:
            port = int(port)

        # override with custom settings if they were set in settings.yaml
        if "settings" in vars():
            if "MongoCheck" in settings:
                if "mongo" in settings['MongoCheck']:
                    cust_cred = settings['MongoCheck']['mongo']

        if "cust_cred" in vars():
            if cust_cred['ip']:
                ip = cust_cred['ip']
            if cust_cred['db']:
                db = cust_cred['db']
            if cust_cred['port']:
                port = cust_cred['port']
            if cust_cred['user']:
                user = cust_cred['user']
            if cust_cred['password']:
                passwort = cust_cred['password']

        # creating MongoDB cursor
        if user:
            require_keys(self.me(), "settings", ["password"], vars())
            MongoCheck.__client = pymongo.MongoClient(
                "mongodb://" + \
                user + ":" + \
                password + "@" + \
                ip + ":" + \
                str(port) + \
                "/?authSource=the_database&authMechanism=SCRAM-SHA-1",
                unicode_decode_error_handler='ignore')
        else:
            MongoCheck.__client = pymongo.MongoClient(
                ip, port, unicode_decode_error_handler='ignore')

        MongoCheck.__db_str = db
        MongoCheck.__db = self.__client[db]
        MongoCheck.__is_setup = True
