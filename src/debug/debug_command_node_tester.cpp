#include "ros/ros.h"
#include "segbot_nlp/VoiceCommand.h"
#include "segbot_nlp/VoiceCommandTypes.h"
//#include <stdio>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
  
  ros::init(argc, argv, "debug_command_node_tester");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<segbot_nlp::VoiceCommand>("command_message", 1000);

  ros::Rate loop_rate(100);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  while (ros::ok()) {
    float meters = 0.0;
    float degrees = 0.0;
    std::cout << "Type in Meters:";
    std::cin >> meters;
    std::cout << "Type in Degrees:";
    std::cin >> degrees;

    segbot_nlp::VoiceCommand msg;
    msg.commandCode = MOVE;
    msg.distance = meters;
    msg.angle = degrees;

    chatter_pub.publish(msg);

  }


  return 0;
}

