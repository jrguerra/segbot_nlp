#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv) {
  ros::init(argc, argv, "debug_linguistic_tree_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  ros::Publisher pub = n.advertise<std_msgs::String>("/filename", 1000);
  
  std_msgs::String msg;
  msg.data = "/nishome/nlpros/ros/rosbuild_ws/segbot_nlp/src/debug/test.xml";
  while (ros::ok()) {
    int result = system("perl /nishome/nlpros/ros/rosbuild_ws/segbot/segbot_nlp/src/debug/lti6.pl");
    pub.publish(msg);
    //ros::spin();
  }


  return 0;
}
