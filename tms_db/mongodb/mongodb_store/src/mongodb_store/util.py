import rospy
import genpy
from std_srvs.srv import Empty
import yaml
from bson import json_util, Binary
import json

import copy
import StringIO
from mongodb_store_msgs.msg import SerialisedMessage
from mongodb_store_msgs.srv import MongoQueryMsgRequest

import importlib

def check_connection_to_mongod(db_host, db_port):
    """
    Check connection to mongod server

    :Returns:
        | bool : True on success, False if connection is not established.
    """
    if check_for_pymongo():
        try:
            from pymongo import Connection
            Connection(db_host, db_port)
            return True
        except Exception as e:
            print("Error: %s" % str(e))
            print("Could not connect to mongo server %s:%d" % (db_host, db_port))
            print("Make sure mongod is launched on your specified host/port")
            return False
    else:
        return False
    

def wait_for_mongo():
    """
    Waits for the mongo server, as started through the mongodb_store/mongodb_server.py wrapper
    
    :Returns:
        | bool : True on success, False if server not even started.
    """
    # Check that mongo is live, create connection
    try:
        rospy.wait_for_service("/datacentre/wait_ready",10)
    except rospy.exceptions.ROSException, e:
        rospy.logerr("Can't connect to MongoDB server. Make sure mongodb_store/mongodb_server.py node is started.")
        return False
    wait = rospy.ServiceProxy('/datacentre/wait_ready', Empty)
    wait()
    return True

def check_for_pymongo():
    """
    Checks for required version of pymongo python library.
    
    :Returns:
        | bool : True if found, otherwise Fale
    """
    try:
        import pymongo
    except:
        print("ERROR!!!")
        print("Can't import pymongo, this is needed by mongodb_store.")
        print("Make sure it is installed (sudo pip install pymongo)")
        return False
    
    return True

"""
Pick an object to use as MongoClient based on the currently installed pymongo
version. Use this instead of importing Connection or MongoClient from pymongo
directly.

Example:
    MongoClient = util.importMongoClient()
"""
def import_MongoClient():
    import pymongo
    if pymongo.version >= '2.4':
        def mongo_client_wrapper(*args, **kwargs):
            return pymongo.MongoClient(*args, **kwargs)
        return mongo_client_wrapper
    else:
        import functools
        def mongo_client_wrapper(*args, **kwargs):
            return pymongo.Connection(*args, **kwargs)
        return functools.partial(mongo_client_wrapper, safe=True)


"""
Given a ROS msg and a dictionary of the right values, fill in the msg
"""
def _fill_msg(msg,dic):
    for i in dic:
        if isinstance(dic[i],dict):
            _fill_msg(getattr(msg,i),dic[i])
        else:
            setattr(msg,i,dic[i])
    

"""
Given a document in the database, return metadata and ROS message -- must have been
"""
def document_to_msg_and_meta(document, TYPE):
    meta = document["_meta"]
    msg = TYPE()
    _fill_msg(msg,document["msg"])
    return meta,msg

"""
Given a document return ROS message
"""
def document_to_msg(document, TYPE):
    msg = TYPE()
    _fill_msg(msg,document)
    return meta

    
def msg_to_document(msg):
    """
    Given a ROS message, turn it into a (nested) dictionary suitable for the datacentre.

    >>> from geometry_msgs.msg import Pose
    >>> msg_to_document(Pose())
    {'orientation': {'w': 0.0, 'x': 0.0, 'y': 0.0, 'z': 0.0},
    'position': {'x': 0.0, 'y': 0.0, 'z': 0.0}}
    
    :Args:
        | msg (ROS Message): An instance of a ROS message to convert
    :Returns:
        | dict : A dictionary representation of the supplied message.
    """

    


    d = {}

    slot_types = []
    if hasattr(msg,'_slot_types'):
        slot_types = msg._slot_types
    else:
        slot_types = [None] * len(msg.__slots__)


    for (attr, type) in zip(msg.__slots__, slot_types):
        d[attr] = sanitize_value(attr, getattr(msg, attr), type)

    return d

def sanitize_value(attr, v, type):
    """
    De-rosify a msg.

    Internal function used to convert ROS messages into dictionaries of pymongo insertable
    values. 

    :Args:
        | attr(str): the ROS message slot name the value came from
        | v: the value from the message's slot to make into a MongoDB able type
        | type (str): The ROS type of the value passed, as given by the ressage slot_types member.
    :Returns:
        | A sanitized version of v.
    """

        # print '---'
        # print attr
        # print v.__class__
        # print type
        # print v

    if isinstance(v, str):
        if type == 'uint8[]':
            v = Binary(v)
        else:
            # ensure unicode
            try:            
                v = unicode(v, "utf-8")
            except UnicodeDecodeError, e:
                # at this point we can deal with the encoding, so treat it as binary
                v = Binary(v)
        # no need to carry on with the other type checks below 
        return v   

    if isinstance(v, rospy.Message):
        return msg_to_document(v)
    elif isinstance(v, genpy.rostime.Time):
        return msg_to_document(v)
    elif isinstance(v, genpy.rostime.Duration):
         return msg_to_document(v)
    elif isinstance(v, list):
        result = []
        for t in v:            
            if hasattr(t, '_type'):
                result.append(sanitize_value(None, t, t._type))
            else: 
                result.append(sanitize_value(None, t, None))
        return result
    else:
        return v




def store_message(collection, msg, meta, oid=None):
    """
    Update ROS message into the DB

    :Args:
        | collection (pymongo.Collection): the collection to store the message in
        | msg (ROS message): an instance of a ROS message to store
        | meta (dict): Additional meta data to store with the ROS message
        | oid (str): An optional ObjectID for the MongoDB document created.
    :Returns:
        | str: ObjectId of the MongoDB document.
    """    
    doc=msg_to_document(msg)
    doc["_meta"]=meta
    #  also store type information
    doc["_meta"]["stored_class"] = msg.__module__ + "." + msg.__class__.__name__
    doc["_meta"]["stored_type"] = msg._type


    if hasattr(msg, '_connection_header'):
        print getattr(msg, '_connection_header')

    if oid != None:
        doc["_id"] = oid

    return collection.insert(doc)

# """
# Stores a ROS message into the DB with msg and meta as separate fields
# """    
# def store_message_separate(collection, msg, meta):
#     doc={}
#     doc["_meta"]=meta
#     doc["msg"]=msg_to_document(msg)
#     return collection.insert(doc)



def store_message_no_meta(collection, msg):
    """
    Store a ROS message sans meta data.

    :Args:
        | collection (pymongo.Collection): The collection to store the message in
        | msg (ROS message): An instance of a ROS message to store
    :Returns:
        | str: The ObjectId of the MongoDB document created.
    """
    doc=msg_to_document(msg)
    return collection.insert(doc)


def fill_message(message, document):
    """
    Fill a ROS message from a dictionary, assuming the slots of the message are keys in the dictionary.

    :Args:
        | message (ROS message): An instance of a ROS message that will be filled in
        | document (dict): A dicionary containing all of the message attributes

    Example:
    
    >>> from geometry_msgs.msg import Pose
    >>> d = dcu.msg_to_document(Pose())
    >>> d['position']['x']=27.0
    >>> new_pose = Pose(
    >>> fill_message(new_pose, d)
    >>>  new_pose
    position: 
      x: 27.0
      y: 0.0
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
    """
    for slot, slot_type in zip(message.__slots__,
                               getattr(message,"_slot_types",[""]*len(message.__slots__))):
        value = document[slot]
        # fill internal structures if value is a dictionary itself
        if isinstance(value, dict):
            fill_message(getattr(message, slot), value)
        elif isinstance(value, list) and slot_type.find("/")!=-1:
            # if its a list and the type is some message (contains a "/")
            lst=[]
            # Remove [] from message type ([:-2])
            msg_type = type_to_class_string(slot_type[:-2])
            msg_class = load_class(msg_type)
            for i in value:
                msg = msg_class()
                fill_message(msg, i)
                lst.append(msg)
            setattr(message, slot, lst)    
        else:
            if isinstance(value, unicode):
                setattr(message, slot, str(value))
            else:
                setattr(message, slot, value)    

def dictionary_to_message(dictionary, cls):
    """
    Create a ROS message from the given dictionary, using fill_message.

    :Args:
        | dictionary (dict): A dictionary containing all of the atributes of the message
        | cls (class): The python class of the ROS message type being reconstructed.
    :Returns:
        An instance of cls with the attributes filled.


    Example:
    
    >>> from geometry_msgs.msg import Pose
    >>> d = {'orientation': {'w': 0.0, 'x': 0.0, 'y': 0.0, 'z': 0.0},
       'position': {'x': 27.0, 'y': 0.0, 'z': 0.0}}
    >>> dictionary_to_message(d, Pose)
    position: 
      x: 27.0
      y: 0.0
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
    """
    message = cls()
    fill_message(message, dictionary)
    return message

def query_message(collection, query_doc, sort_query=[], find_one=False):
    """
    Peform a query for a stored messages, returning results in list.

    :Args:
        | collection (pymongo.Collection): The collection to query
        | query_doc (dict): The MongoDB query to execute
        | sort_query (list of tuple): The MongoDB query to sort
        | find_one (bool): Returns one matching document if True, otherwise all matching.
    :Returns:
        | dict or list of dict: the MongoDB document(s) found by the query
    """

    if find_one:
        ids = ()
        if sort_query:
            result = collection.find_one(query_doc, sort=sort_query)
        else:
            result = collection.find_one(query_doc)
        if result:
            return [ result ] 
        else:
            return []
    else:
        if sort_query:
            return [ result for result in collection.find(query_doc).sort(sort_query) ]
        else:
            return [ result for result in collection.find(query_doc) ]

def update_message(collection, query_doc, msg, meta, upsert):
    """
    Update ROS message in the DB, return updated id and true if db altered.

    :Args:
        | collection (pymongo.Collection): The collection to update in
        | query_doc (dict): The MongoDB query to execute to select document for update
        | msg (ROS message): An instance of a ROS message to update to
        | meta (dict): New meta data to update the stored message with
        | upsert (bool): If message does not already exits, create if upsert==True.
    :Returns:
        | str, bool: the OjectId of the updated document and whether it was altered by
                     the operation
    """    
    # see if it's in db first
    result = collection.find_one(query_doc)

    # if it's not in there but we're allowed to insert
    if not result:
        if upsert:
            return store_message(collection, msg, meta), True
        else:
            return "", False

    # convert msg to db document
    doc=msg_to_document(msg)
    
    #update _meta
    doc["_meta"] = result["_meta"]
    #merge the two dicts, overwiriting elements in doc["_meta"] with elements in meta
    doc["_meta"]=dict(list(doc["_meta"].items()) + list(meta.items())) 

    # ensure necessary parts are there too 
    doc["_meta"]["stored_class"] = msg.__module__ + "." + msg.__class__.__name__
    doc["_meta"]["stored_type"] = msg._type
    
    return collection.update(query_doc, doc), True


def query_message_ids(collection, query_doc, find_one):
    """
    Peform a query for a stored message, returning a tuple of id strings

    :Args:
        | collection (pymongo.Collection): The collection to search
        | query_doc (dict): The MongoDB query to execute
        | find_one (bool): Find one matching document if True, otherwise all matching.
    :Returns:
        | tuple of strings: all ObjectIds of matching documents
    """
    if find_one:
        result = collection.find_one(query_doc)
        if result:
            return str(result["_id"]), 
    else:
        return tuple(str(result["_id"]) for result in collection.find(query_doc, {'_id':1}))



def type_to_class_string(type):
    """ 
    Takes a ROS msg type and turns it into a Python module and class name. 

    E.g
    
    >>> type_to_class_string("geometry_msgs/Pose")
    geometry_msgs.msg._Pose.Pose

    :Args:
        | type (str): The ROS message type to return class string
    :Returns:
        | str: A python class string for the ROS message type supplied
    """    
    parts = type.split('/')
    cls_string = "%s.msg._%s.%s" % (parts[0], parts[1], parts[1])
    return cls_string

def load_class(full_class_string):
    """
    Dynamically load a class from a string
    shamelessly ripped from: http://thomassileo.com/blog/2012/12/21/dynamically-load-python-modules-or-classes/

    :Args:
        | full_class_string (str): The python class to dynamically load
    :Returns:
        | class: the loaded python class.
    """
    # todo: cache classes (if this is an overhead)
    class_data = full_class_string.split(".")
    module_path = ".".join(class_data[:-1])
    class_str = class_data[-1]
    module = importlib.import_module(module_path)
    # Finally, we retrieve the Class
    return getattr(module, class_str)


def serialise_message(message):
    """
    Create a mongodb_store_msgs/SerialisedMessage instance from a ROS message.

    :Args:
        | message (ROS message): The message to serialise
    :Returns:
        | mongodb_store_msgs.msg.SerialisedMessage: A serialies copy of message
    """
    buf=StringIO.StringIO() 
    message.serialize(buf)
    serialised_msg = SerialisedMessage()
    serialised_msg.msg = buf.getvalue() 
    serialised_msg.type = message._type
    return serialised_msg

def deserialise_message(serialised_message):
    """
    Create a ROS message from a mongodb_store_msgs/SerialisedMessage

    :Args:
        | serialised_message (mongodb_store_msgs.msg.SerialisedMessage): The message to deserialise
    :Returns:
        | ROS message: The message deserialised
    """
    cls_string = type_to_class_string(serialised_message.type)
    cls = load_class(cls_string)
    # instantiate an object from the class
    message = cls()
    # deserialize data into object
    message.deserialize(serialised_message.msg)
    return message


def string_pair_list_to_dictionary_no_json(spl):
    """
    Covert a mongodb_store_msgs/StringPairList into a dictionary, ignoring content

    :Args:
        | spl (StringPairList): The list of (key, value) to pairs convert
    :Returns:
        | dict: resulting dictionary
    """
    return dict((pair.first, pair.second) for pair in spl)

def string_pair_list_to_dictionary(spl):
    """
    Creates a dictionary from a mongodb_store_msgs/StringPairList which could contain JSON as a string.
    If the first entry in the supplied list is a JSON query then the returned dictionary is loaded from that.

    :Args:
        | spl (StringPairList): The list of (key, value) pairs to convert
    :Returns:
        | dict: resulting dictionary
    """
    if len(spl.pairs) > 0 and spl.pairs[0].first == MongoQueryMsgRequest.JSON_QUERY:
        # print "looks like %s", spl.pairs[0].second
        return json.loads(spl.pairs[0].second, object_hook=json_util.object_hook)
    # else use the string pairs
    else:
        return string_pair_list_to_dictionary_no_json(spl.pairs)

def topic_name_to_collection_name(topic_name):
    """
    Converts the fully qualified name of a topic into legal mongodb collection name.
    """
    return topic_name.replace("/", "_")[1:]
