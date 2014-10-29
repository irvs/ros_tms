//------------------------------------------------------------------------------
// @file   : request_generator.cpp
// @brief  : Based on the received text, create some request
// @author : Kazuto Nakashima
// @date   : 2014.06.27
//------------------------------------------------------------------------------
// Include for ROS
#include "ros/ros.h"
#include "tms_ur_glass_test/object_list.h"
#include "tms_ur_glass_test/dictation_data.h"
#include "tms_msg_rp/rp_arrow.h"
#include "msg_data/recognized_text.h"

// Include for std::
#include <map>
#include <vector>
#include <string>

std::map<std::string, std::string*> hash_table;
std::vector<std::string> value; // for hash_table

//------------------------------------------------------------------------------
// Set data table
// e.g. If detected sentences below on Julius,
// reference to the related tag.
// "ペットボトル。" "ボトル。" "飲み物。" >> "petbottle"
void SetDataTable() {
  std::vector<std::string>::iterator itr;
  value.push_back("chipstar");
  value.push_back("tea");
  value.push_back("soysauce");
  value.push_back("bottle");
  value.push_back("robot");
  // chipstar-------------------------------------------------------------------
  itr = value.begin();
  hash_table.insert(std::make_pair("お菓子",&(*itr)));
  hash_table.insert(std::make_pair("チップスター",&(*itr)));
  hash_table.insert(std::make_pair("ポテトチップス",&(*itr)));
  hash_table.insert(std::make_pair("チップス",&(*itr)));
  // tea------------------------------------------------------------------------
  itr++;
  hash_table.insert(std::make_pair("ペットボトル",&(*itr)));
  hash_table.insert(std::make_pair("お茶",&(*itr)));
  // soysauce-------------------------------------------------------------------
  itr++;
  hash_table.insert(std::make_pair("醤油",&(*itr)));
  hash_table.insert(std::make_pair("しょうゆ",&(*itr)));
  // bottle---------------------------------------------------------------------
  itr++;
  hash_table.insert(std::make_pair("ボトル",&(*itr)));
  // robot---------------------------------------------------------------------
  itr++;
  hash_table.insert(std::make_pair("ロボット",&(*itr)));

}

//------------------------------------------------------------------------------
// List candidates based on the given tag
void ListCandidates(std::string tag, tms_ur_glass_test::object_list* srv) {
  if (tag == "chipstar") {
    srv->request.id_in.push_back(7001); // chipstar_red
    srv->request.id_in.push_back(7002); // chipstar_orange
    srv->request.id_in.push_back(7003); // chipstar_green
  } else if (tag == "tea") {
    srv->request.id_in.push_back(7004); // greentea_bottle
    srv->request.id_in.push_back(7005); // soukentea_bottle
  } else if (tag == "soysauce") {
    srv->request.id_in.push_back(7009); // soysauce_bottle_black
    srv->request.id_in.push_back(7010); // soysauce_bottle_blue
    srv->request.id_in.push_back(7011); // soysauce_bottle_white
  } else if (tag == "bottle") {
    srv->request.id_in.push_back(7004); // greentea_bottle
    srv->request.id_in.push_back(7005); // soukentea_bottle
    srv->request.id_in.push_back(7009); // soysauce_bottle_black
    srv->request.id_in.push_back(7010); // soysauce_bottle_blue
    srv->request.id_in.push_back(7011); // soysauce_bottle_white
  } else if (tag == "robot") {
    srv->request.id_in.push_back(2001); // smartpal5
    srv->request.id_in.push_back(2002); // smartpal4
  }
}

//------------------------------------------------------------------------------
// Callback function
bool dictCallback(msg_data::recognized_text::Request  &req, 
                  msg_data::recognized_text::Response &res) {
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<tms_ur_glass_test::object_list>("object_decision");
  ros::ServiceClient disp_client = n.serviceClient<tms_msg_rp::rp_arrow>("rp_arrow");
  
  tms_ur_glass_test::object_list srv;
  tms_msg_rp::rp_arrow disp_srv;

  std::cout << "Detected sentence: " << req.request << std::endl;

  // Seek the key in the hash table
  if (hash_table.find(req.request) != hash_table.end()) {
    ROS_INFO("Matched: %s", (*(hash_table.at(req.request))).c_str());
    ListCandidates(*(hash_table.at(req.request)), &(srv));
  } else {
    ROS_ERROR("Nothing has matched\n");
    res.response = "Nothing has matched";
    return true;
  }

  // object_selector
  if (client.call(srv)) {
    // The most possible object
    ROS_INFO("%d %s\n", srv.response.id_out, srv.response.name.c_str());
    
    // Point to the object on the choreonoid
    disp_srv.request.x = srv.response.x * 1000;
    disp_srv.request.y = srv.response.y * 1000;
    disp_srv.request.z = srv.response.z * 1000;
    disp_srv.request.id = 20001;
    disp_srv.request.mode = 1;

    // rp_arrow
    if (disp_client.call(disp_srv)) {
      disp_srv.request.mode = 0;
      if (disp_client.call(disp_srv)) { 
        res.response = "Succeeded to call service rp_arrow";
        return true;
      }
    } else {
      ROS_ERROR("Failed to call service rp_arrow\n");
      res.response = "Failed to call service rp_arrow";
      return true;
    }

  } else {
    ROS_ERROR("Failed to call service object_selector\n");
    res.response = "Failed to call service object_selector";
  }
  return true;
}

//------------------------------------------------------------------------------
int main(int argc, char **argv) {
  ros::init(argc, argv, "request_generator");
  ros::NodeHandle nh;

  SetDataTable();
  ROS_INFO("Ready to create any request");

  ros::ServiceServer server_ = nh.advertiseService("recognition_test", dictCallback);
  ros::spin();

  return 0;
}
