#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "LinguisticTree.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "debug_linguistic_tree_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  NLP::LinguisticTree test("test.xml");

  while (ros::ok()) {
    std_msgs::String msg;

    std::stringstream ss;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
