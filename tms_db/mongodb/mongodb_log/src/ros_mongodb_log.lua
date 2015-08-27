#!/usr/bin/lua

----------------------------------------------------------------------------
--  ros_mongodb_log.lua - Lua based ROS to MongoDB logger
--
--  Created: Wed Nov 03 17:34:31 2010
--  Copyright  2010  Tim Niemueller [www.niemueller.de]
--                   Carnegie Mellon University
--                   Intel Labs Pittsburgh
----------------------------------------------------------------------------

--  This program is free software; you can redistribute it and/or modify
--  it under the terms of the GNU General Public License as published by
--  the Free Software Foundation; either version 2 of the License, or
--  (at your option) any later version.
--
--  This program is distributed in the hope that it will be useful,
--  but WITHOUT ANY WARRANTY; without even the implied warranty of
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--  GNU Library General Public License for more details.
--
--  Read the full text in the LICENSE.GPL file in the doc directory.


--  ************************************************************************
--  Note: this script is working and short, but it has been retired in favor
--        of the Python version, because that can write multiple messages
--        concurrently to the database which is beneficial on heavy traffic
--        topics.
--
--  It is however a very good start if you want to write a custom logger for
--  low-traffic topics and where you need to do special preprocessing etc.
--  ************************************************************************

require("roslua")
roslua.assert_version(0,4,1)

roslua.init_node{master_uri=os.getenv("ROS_MASTER_URI"),
		 node_name="/ros_mongodb_log"}

local dbname = "ros_mongodb_log"
local dbhost = "localhost"
local topics = { {name = "/chatter", type = "std_msgs/String"} }

require("mongo")

local db = mongo.Connection.New()
db:connect("localhost")

for _, t in ipairs(topics) do
   local s = roslua.subscriber(t.name, t.type)
   local subcolname = t.name:gsub("/", "_"):sub(2)
   local collection_name = dbname .. "." .. subcolname
   s:add_listener(function (message)
		     local doc = message:plain_value_table()
		     db:insert(collection_name, doc)
		  end)
   t.subscriber = s
end

roslua.run()
