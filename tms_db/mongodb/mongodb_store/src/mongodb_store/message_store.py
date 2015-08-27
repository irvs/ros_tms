import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store_msgs.msg import StringPair, StringPairList, SerialisedMessage
from bson import json_util
from bson.objectid import ObjectId
import json
import copy


class MessageStoreProxy:
	"""
	A class that provides functions for storage and retrieval of ROS Message
	objects in the mongodb_store. This is achieved by acting as a proxy to the 
	services provided by the MessageStore ROS node, and therefore requires the message 
	store node to be running in addition to the datacentre:

	`rosrun mongodb_store message_store_node.py`


	>>> from geometry_msgs.msg import Pose, Quaternion
	>>> msg_store = MessageStoreProxy()
	>>> p = Pose(Point(0, 1, 2), Quaternion(0, 0, 0 , 1))
	>>> msg_store.insert_named("my favourite pose", p)
        >>> retrieved = msg_store.query_named("my favourite pose", Pose._type)
	
	For usage examples, please see `example_message_store_client.py` within the scripts
	folder of mongodb_store.
	
	"""
	
	def __init__(self, service_prefix='/message_store', database='message_store', collection='message_store'):
		"""
		:Args:
		    | service_prefix (str): The prefix to the *insert*, *update*, *delete* and
		      *query_messages* ROS services/
		    | database (str): The MongoDB database that this object works with.
		    | collection (str): The MongoDB collection that this object works with.
		"""
		self.database = database
		self.collection = collection
		insert_service = service_prefix + '/insert'
		update_service = service_prefix + '/update'
		delete_service = service_prefix + '/delete'
		query_ids_service = service_prefix + '/query_messages'
                # try and get the mongo service, block until available
                found_services_first_try = True # if found straight away
                while True:
                        try:
                                rospy.wait_for_service(insert_service,5)
                                rospy.wait_for_service(update_service,5)
                                rospy.wait_for_service(query_ids_service,5)
                                rospy.wait_for_service(delete_service,5)
                                break
                        except rospy.ROSException, e:
                                found_services_first_try = False
                                rospy.logerr("Could not get message store services. Maybe the message "
                                             "store has not been started? Retrying..")
                if not found_services_first_try:
                        rospy.loginfo("Message store services found.")
		self.insert_srv = rospy.ServiceProxy(insert_service, dc_srv.MongoInsertMsg)
		self.update_srv = rospy.ServiceProxy(update_service, dc_srv.MongoUpdateMsg)
		self.query_id_srv = rospy.ServiceProxy(query_ids_service, dc_srv.MongoQueryMsg)
		self.delete_srv = rospy.ServiceProxy(delete_service, dc_srv.MongoDeleteMsg)


	def insert_named(self, name, message, meta = {}):
		"""
		Inserts a ROS message into the message storage, giving it a name for convenient
		later retrieval.
		.. note:: Multiple messages can be stored with the same name.
		
		:Args:
		    | name (str): The name to refere to this message as.
		    | message (ROS Message): An instance of a ROS message type to store
		    | meta (dict): A dictionary of additional meta data to store in association
                      		    with thie message.
		:Returns:
		    | (str) the ObjectId of the MongoDB document containing the stored message.
		"""
		# create a copy as we're modifying it
		meta_copy = copy.copy(meta)
		meta_copy["name"] = name
		return self.insert(message, meta_copy)
	

	def insert(self, message, meta = {}):
		"""
		Inserts a ROS message into the message storage.
		
		:Args:
		    | message (ROS Message): An instance of a ROS message type to store
		    | meta (dict): A dictionary of additional meta data to store in association
                      		    with thie message.
		:Returns:
		    | (str) the ObjectId of the MongoDB document containing the stored message.

		"""
		# assume meta is a dict, convert k/v to tuple pairs 
		meta_tuple = (StringPair(dc_srv.MongoQueryMsgRequest.JSON_QUERY, json.dumps(meta, default=json_util.default)),)
		serialised_msg = dc_util.serialise_message(message)
		return self.insert_srv(self.database, self.collection, serialised_msg, StringPairList(meta_tuple)).id

	def query_id(self, id, type):
		"""
		Finds and returns the message with the given ID.
		
		:Parameters:
		    | id (str): The ObjectID of the MongoDB document holding the message.
		    | type (str): The ROS message type of the stored messsage to retrieve.
		:Returns:
		    | message (ROS message), meta (dict): The retrieved message and associated metadata
		      or *None* if the named message could not be found.
		"""
		return self.query(type, {'_id': ObjectId(id)}, {}, True)
	
	def delete(self, message_id):
		"""
		Delete the message with the given ID.
		
		:Parameters:
		    | message_id (str) : The ObjectID of the MongoDB document holding the message.
		:Returns:
		    | bool : was the object successfully deleted.
		"""
		return self.delete_srv(self.database, self.collection, message_id)

	def query_named(self, name, type, single = True, meta = {}):
		"""
		Finds and returns the message(s) with the given name.
		
		:Args:
		    | name (str): The name of the stored messages to retrieve.
		    | type (str): The type of the stored message.
		    | single (bool): Should only one message be returned?
		    | meta (dict): Extra queries on the meta data of the message.
		:Return:
		    | message (ROS message), meta (dict): The retrieved message and associated metadata
		      or *None* if the named message could not be found.
		 
		"""
		# create a copy as we're modifying it
		meta_copy = copy.copy(meta)
		meta_copy["name"] = name
		return self.query(type, {}, meta_copy, single)

	def update_named(self, name, message, meta = {}, upsert = False):
		"""
		Updates a named message.
		
		:Args:
		    | name (str): The name of the stored messages to update.
		    | message (ROS Message): The updated ROS message
		    | meta (dict): Updated meta data to store with the message.
		    | upsert (bool): If True, insert the named message if it doesnt exist.
		:Return:
		    | str, bool: The MongoDB ObjectID of the document, and whether it was altered by
		                 the update.	 
		"""
		meta_query = {}
		meta_query["name"] = name

		# make sure the name goes into the meta info after update
		meta_copy = copy.copy(meta)
		meta_copy["name"] = name

		return self.update(message, meta_copy, {}, meta_query, upsert)		

	def update_id(self, id, message, meta = {}, upsert = False):
		"""
		Updates a message by MongoDB ObjectId.
		
		:Args:
		    | id (str): The MongoDB ObjectId of the doucment storing the message.
		    | message (ROS Message): The updated ROS message
		    | meta (dict): Updated meta data to store with the message.
		    | upsert (bool): If True, insert the named message if it doesnt exist.
		:Return:
		    | str, bool: The MongoDB ObjectID of the document, and whether it was altered by
		                 the update.
		 
		"""

		msg_query = {'_id': ObjectId(id)}
		meta_query = {}

		return self.update(message, meta, msg_query, meta_query, upsert)		

	def update(self, message, meta = {}, message_query = {}, meta_query = {},  upsert = False):
		"""
		Updates a message.
		
		:Args:
		    | message (ROS Message): The updated ROS message
		    | meta (dict): Updated meta data to store with the message.
		    | message_query (dict): A query to match the ROS message that is to be updated.
		    | meta_query (dict): A query to match against the meta data of the message to be updated
		    | upsert (bool): If True, insert the named message if it doesnt exist.
		:Return:
		    | str, bool: The MongoDB ObjectID of the document, and whether it was altered by
		                 the update.
		 
		"""
		# serialise the json queries to strings using json_util.dumps
		message_query_tuple = (StringPair(dc_srv.MongoQueryMsgRequest.JSON_QUERY, json.dumps(message_query, default=json_util.default)),)
		meta_query_tuple = (StringPair(dc_srv.MongoQueryMsgRequest.JSON_QUERY, json.dumps(meta_query, default=json_util.default)),)
		meta_tuple = (StringPair(dc_srv.MongoQueryMsgRequest.JSON_QUERY, json.dumps(meta, default=json_util.default)),)
		return self.update_srv(self.database, self.collection, upsert, StringPairList(message_query_tuple), StringPairList(meta_query_tuple), dc_util.serialise_message(message), StringPairList(meta_tuple))		


	"""
	Returns [message, meta] where message is the queried message and meta a dictionary of meta information. If single is false returns a list of these lists.
	"""
	def query(self, type, message_query = {}, meta_query = {}, single = False, sort_query = []):
		"""
		Finds and returns message(s) matching the message and meta data queries.
		
		:Parameters:
		    | type (str): The ROS message type of the stored messsage to retrieve.
		    | message_query (dict): A query to match the actual ROS message
		    | meta_query (dict): A query to match against the meta data of the message
		    | sort_query (list of tuple): A query to request sorted list to mongodb module
		    | single (bool): Should only one message be returned?
		:Returns:
		    | [message, meta] where message is the queried message and meta a dictionary of
		      meta information. If single is false returns a list of these lists.
		"""
		# assume meta is a dict, convert k/v to tuple pairs for ROS msg type

		# serialise the json queries to strings using json_util.dumps
		message_tuple = (StringPair(dc_srv.MongoQueryMsgRequest.JSON_QUERY, json.dumps(message_query, default=json_util.default)),)
		meta_tuple = (StringPair(dc_srv.MongoQueryMsgRequest.JSON_QUERY, json.dumps(meta_query, default=json_util.default)),)
		if len(sort_query) > 0:
				sort_tuple = [StringPair(str(k), str(v)) for k, v in sort_query]
		else:
				sort_tuple = []

		# a tuple of SerialisedMessages
		response = self.query_id_srv(self.database, self.collection, type, single, StringPairList(message_tuple), StringPairList(meta_tuple), StringPairList(sort_tuple))

		# print response

		if response.messages is None:
			messages = []
			metas = []
		else:
			messages = map(dc_util.deserialise_message, response.messages) 
			metas = map(dc_util.string_pair_list_to_dictionary, response.metas)

		if single:
			if len(messages) > 0:
				return [messages[0], metas[0]]
			else:				
				return [None, None]
		else:
			return zip(messages,metas)
