#ifndef __MESSAGE_STORE__H
#define __MESSAGE_STORE__H

#include "ros/ros.h"
#include "ros/console.h"
#include "mongodb_store_msgs/MongoInsertMsg.h"
#include "mongodb_store_msgs/MongoUpdateMsg.h"
#include "mongodb_store_msgs/MongoQueryMsg.h"
#include "mongodb_store_msgs/MongoDeleteMsg.h"
#include "mongodb_store_msgs/StringPair.h"
#include "mongodb_store_msgs/SerialisedMessage.h"

//include to get BSON. There's probably a much smaller of set of headers we could get away with
#include "mongo/client/dbclient.h"

#include  <boost/make_shared.hpp> 
#include <assert.h>



namespace mongodb_store {

typedef std::vector<mongodb_store_msgs::StringPair> StringPairs;


/**
Populates a SerialisedMessage using the given instance of MsgType
**/
template<typename MsgType> 
void fill_serialised_message(mongodb_store_msgs::SerialisedMessage & _sm, 
								const MsgType & _msg) {
	
	//record type
	_sm.type = ros::message_traits::DataType<MsgType>::value();

	//how long the data will be
	uint32_t serial_size = ros::serialization::serializationLength(_msg);
	
	// std::cout<<"serial_size: "<<serial_size<<std::endl;	

 	//set msg vector to this size + 4 to hold length description
	_sm.msg.resize(serial_size);
	//serialise the object into the vector via this stream
	ros::serialization::OStream stream(&(_sm.msg[0]), serial_size);
	
	// serialise message
	ros::serialization::serialize(stream, _msg);
}


template<typename MsgType> 
boost::shared_ptr<MsgType> deserialise_message(mongodb_store_msgs::SerialisedMessage & _sm) {

	// std::cout<<_sm<<std::endl;

	boost::shared_ptr<MsgType> message = boost::make_shared<MsgType>();

	uint32_t serial_size = _sm.msg.size();
	
	// std::cout<<"serial_size: "<<serial_size<<std::endl;	

	ros::serialization::IStream msgStream(&(_sm.msg[0]), serial_size);
	ros::serialization::deserialize(msgStream, *message);

	return message;
}


template<typename MsgType> 
const std::string get_ros_type() {
	return ros::message_traits::DataType<MsgType>::value();
}

template<typename MsgType> 
const std::string get_ros_type(const MsgType & _msg) {
	return get_ros_type<MsgType>();
}

mongodb_store_msgs::StringPair makePair(const std::string & _first, const std::string & _second);


class MessageStoreProxy
{
public:

	/**

	**/
	MessageStoreProxy(ros::NodeHandle handle, 
		const std::string & _collection = "message_store",
		const std::string & _database = "message_store", 		
		const std::string & _servicePrefix = "/message_store") :
		m_insertClient(handle.serviceClient<mongodb_store_msgs::MongoInsertMsg>(_servicePrefix + "/insert")),
		m_updateClient(handle.serviceClient<mongodb_store_msgs::MongoUpdateMsg>(_servicePrefix + "/update")),
		m_queryClient(handle.serviceClient<mongodb_store_msgs::MongoQueryMsg>(_servicePrefix + "/query_messages")),
		m_deleteClient(handle.serviceClient<mongodb_store_msgs::MongoDeleteMsg>(_servicePrefix + "/delete")),
		m_database(_database),
		m_collection(_collection)
	{

		m_insertClient.waitForExistence();
		m_updateClient.waitForExistence();
		m_queryClient.waitForExistence();
		m_deleteClient.waitForExistence();
	}

	MessageStoreProxy(const MessageStoreProxy& _rhs) :
		m_database(_rhs.m_database),
		m_collection(_rhs.m_collection),
		m_insertClient(_rhs.m_insertClient),
		m_updateClient(_rhs.m_insertClient),
		m_queryClient(_rhs.m_queryClient),
		m_deleteClient(_rhs.m_deleteClient)
	{}


	~MessageStoreProxy() {}


	template<typename MsgType> 
	std::string insert(const MsgType & _msg, const mongo::BSONObj & _meta = mongo::BSONObj()) {
		return insert(_msg, m_database, m_collection, _meta);
	}


	template<typename MsgType> 
	std::string insertNamed(const std::string & _name, const MsgType & _msg, 
		const mongo::BSONObj & _meta = mongo::BSONObj()) {

		//create a copy of the meta data with the name included
		mongo::BSONObjBuilder builder;
		builder.appendElements(_meta);
		builder.append("name", _name);

		//and insert as usual
		return insert(_msg, m_database, m_collection, builder.obj());
	}

	template<typename MsgType> 
	std::string insert(const MsgType & _msg, 
		const std::string & _database, 
		const std::string & _collection, 
		const mongo::BSONObj & _meta = mongo::BSONObj()) {

  		//Create message with basic fields
  		mongodb_store_msgs::MongoInsertMsg srv;
  		srv.request.database = _database;
  		srv.request.collection = _collection;
  		
 		
 		//if there's no meta then no copying is necessary
  		if(!_meta.isEmpty()) {
 			srv.request.meta.pairs.push_back(makePair(mongodb_store_msgs::MongoQueryMsgRequest::JSON_QUERY, _meta.jsonString()));
		}

	 	fill_serialised_message(srv.request.message, _msg);

  		//sent data over
  		m_insertClient.call(srv);
  		return srv.response.id;

	}

	template<typename MsgType> 
	bool queryNamed(const std::string & _name, 
					std::vector< boost::shared_ptr<MsgType> > & _messages, 
					bool _find_one = true) {

		
		mongo::BSONObj meta_query = BSON( "name" << _name );
		return query<MsgType>(_messages, EMPTY_BSON_OBJ, meta_query, _find_one);
	}

	template<typename MsgType> 
	std::pair<boost::shared_ptr<MsgType>, mongo::BSONObj> queryNamed(const std::string & _name, bool _find_one = true) {

		std::vector< std::pair<boost::shared_ptr<MsgType>, mongo::BSONObj> > msg_and_metas;
		mongo::BSONObj meta_query = BSON( "name" << _name );
		bool result = query(msg_and_metas, EMPTY_BSON_OBJ, meta_query, true, true);

		if(result) {
			return msg_and_metas[0];
		}
		else {
			return std::make_pair(boost::make_shared<MsgType>(), EMPTY_BSON_OBJ);
		}
	}




	template<typename MsgType> 
	bool queryID(const std::string & _id, 
					std::vector< boost::shared_ptr<MsgType> > & _messages) {
		
		mongo::BSONObj msg_query = BSON( "_id" << mongo::OID(_id) );
		return query<MsgType>(_messages, msg_query, EMPTY_BSON_OBJ, true);
	}


	template<typename MsgType> 
	std::pair<boost::shared_ptr<MsgType>, mongo::BSONObj> queryID(const std::string & _id) {
		mongo::BSONObj msg_query = BSON( "_id" << mongo::OID(_id) );

		std::vector< std::pair<boost::shared_ptr<MsgType>, mongo::BSONObj> > msg_and_metas;
		bool result = query(msg_and_metas, msg_query, EMPTY_BSON_OBJ, true, true);

		if(result) {
			return msg_and_metas[0];
		}
		else {
			return std::make_pair(boost::make_shared<MsgType>(), EMPTY_BSON_OBJ);
		}
	}





	template<typename MsgType> 
	bool query(std::vector< std::pair<boost::shared_ptr<MsgType>, mongo::BSONObj> > & _messages,
				const mongo::BSONObj & _message_query = mongo::BSONObj(),
				const mongo::BSONObj & _meta_query = mongo::BSONObj(),
				bool _find_one = false,
				bool _decode_metas = true) {

		//Create message with basic fields
  		mongodb_store_msgs::MongoQueryMsg msg;
  		msg.request.database = m_database;
  		msg.request.collection = m_collection;
  		msg.request.type = get_ros_type<MsgType>();
  		msg.request.single = _find_one;
  	
		//if there's no message then no copying is necessary
  		if(!_message_query.isEmpty()) {
 			msg.request.message_query.pairs.push_back(makePair(mongodb_store_msgs::MongoQueryMsgRequest::JSON_QUERY, _message_query.jsonString())); 			 			
		}

		//if there's no meta then no copying is necessary
		if(!_meta_query.isEmpty()) {
 			msg.request.meta_query.pairs.push_back(makePair(mongodb_store_msgs::MongoQueryMsgRequest::JSON_QUERY, _meta_query.jsonString())); 			 			
		}

  		if(m_queryClient.call(msg)) {
  			ROS_DEBUG("Got back %li messages", msg.response.messages.size());
  			if(msg.response.messages.size() > 0) {

  				assert(msg.response.messages.size() == msg.response.metas.size());

	  			for(size_t i = 0; i < msg.response.messages.size(); i ++) {

	  				if(_decode_metas && msg.response.metas[i].pairs[0].first == mongodb_store_msgs::MongoQueryMsgRequest::JSON_QUERY) {
	  					_messages.push_back(std::make_pair(deserialise_message<MsgType>(msg.response.messages[i]), mongo::fromjson(msg.response.metas[i].pairs[0].second)));		
					}	  			
					else {
						if(_decode_metas) {
							ROS_WARN("Can't handle non-json meta in cpp at the moment");
						}
						_messages.push_back(std::make_pair(deserialise_message<MsgType>(msg.response.messages[i]), EMPTY_BSON_OBJ));		
					}

				}

	  			return true;
	  		}
  		}

		return false;
	}

	template<typename MsgType> 
	bool query(std::vector< boost::shared_ptr<MsgType> > & _messages,
				const mongo::BSONObj & _message_query = mongo::BSONObj(),
				const mongo::BSONObj & _meta_query = mongo::BSONObj(),
				bool _find_one = false) {

		// call other query method, but ignore metas
		std::vector< std::pair<boost::shared_ptr<MsgType>, mongo::BSONObj> > msg_and_metas;
		bool result = query(msg_and_metas, _message_query, _meta_query, _find_one, false);

		for (typename std::vector< std::pair<boost::shared_ptr<MsgType>, mongo::BSONObj> >::iterator i = msg_and_metas.begin(); i != msg_and_metas.end(); ++i)
		{
			_messages.push_back(i->first);
		}


			


		return result;

	}


	template<typename MsgType> 
	bool updateID(const std::string & _id, 
					const MsgType & _msg, 
					const mongo::BSONObj & _meta = mongo::BSONObj()) {

		mongo::BSONObj msg_query = BSON( "_id" << mongo::OID(_id) );
		return update<MsgType>(_msg, _meta, msg_query, EMPTY_BSON_OBJ, false);
	}




	template<typename MsgType> 
	bool updateNamed(const std::string & _name, 
					const MsgType & _msg, 
					bool _upsert = false,
					const mongo::BSONObj & _meta = mongo::BSONObj()) {

		mongo::BSONObj meta_query = BSON( "name" << _name );

		// make sure the name goes into the meta info after update
		mongo::BSONObjBuilder builder;
		builder.appendElements(_meta);
		builder.append("name", _name);

		return update<MsgType>(_msg, builder.obj(), EMPTY_BSON_OBJ, meta_query, _upsert);
	}

	template<typename MsgType> 
	bool update(const MsgType & _msg, 
				const mongo::BSONObj & _meta = mongo::BSONObj(),
				const mongo::BSONObj & _message_query = mongo::BSONObj(),
				const mongo::BSONObj & _meta_query = mongo::BSONObj(),
				bool _upsert = false) {

		//Create message with basic fields
  		mongodb_store_msgs::MongoUpdateMsg msg;
  		msg.request.database = m_database;
  		msg.request.collection = m_collection;
  		msg.request.upsert = _upsert;
  	
		//if there's no message then no copying is necessary
  		if(!_message_query.isEmpty()) {
 			msg.request.message_query.pairs.push_back(makePair(mongodb_store_msgs::MongoQueryMsgRequest::JSON_QUERY, _message_query.jsonString())); 			 			
		}

		//if there's no meta then no copying is necessary
		if(!_meta_query.isEmpty()) {
 			msg.request.meta_query.pairs.push_back(makePair(mongodb_store_msgs::MongoQueryMsgRequest::JSON_QUERY, _meta_query.jsonString())); 			 			
		}

		fill_serialised_message(msg.request.message, _msg);

			//if there's no meta then no copying is necessary
  		if(!_meta.isEmpty()) {
 			msg.request.meta.pairs.push_back(makePair(mongodb_store_msgs::MongoQueryMsgRequest::JSON_QUERY, _meta.jsonString()));
		}


  		if(m_updateClient.call(msg)) {
  			return msg.response.success;
  		}
  		else {
 	 		return false;
 	 	}


	}

	bool deleteID(const std::string & _id) {

  		//Create message with basic fields
  		mongodb_store_msgs::MongoDeleteMsg srv;
  		srv.request.database = m_database;
  		srv.request.collection = m_collection;
  		srv.request.document_id = _id;
  		 		
  		//sent data over
  		m_deleteClient.call(srv);
  		return srv.response.success;
	}



protected:


	std::string m_database;
	std::string m_collection;
	ros::ServiceClient m_insertClient;
	ros::ServiceClient m_updateClient;
	ros::ServiceClient m_queryClient;
	ros::ServiceClient m_deleteClient;

	//an empty bson doc to save recreating one whenever one is not required
    static const mongo::BSONObj EMPTY_BSON_OBJ;
};

}


#endif
