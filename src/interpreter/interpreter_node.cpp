#include "ros/ros.h"
#include "std_msgs/String.h"
#include "Interpreter.h"
#include "LinguisticTree.h"
#include <list>
#include "segbot_nlp/VoiceCommand.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

ros::Publisher commandPublish;
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("Received filename: [%s]", msg->data.c_str());
  
  NLP::LinguisticTree tree(msg->data.c_str());
  std::list<CommandVector> commands = Interpreter::interpret(tree);
  std::cerr << commands.size() << '\n';
  
  for (std::list<CommandVector>::iterator i = commands.begin(); i != commands.end(); ++i) {
    segbot_nlp::VoiceCommand c;
    c.commandCode = i->trueType;
    c.distance = i->distance;
    if (i->dir == D_backward) {
        c.angle = 180.00;
    } else if (i->dir == D_right) {
        c.angle = - i->angle;
    } else if (i->dir == D_forward || i->dir == D_moveDefault) {
        c.angle = 0;
    } else {
        c.angle = i->angle;
    }

    c.loc = i->loc;
    c.numTimes = i->repetitions;
    commandPublish.publish(c); 
  } 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "interpreter");
  ros::NodeHandle n;

  NLP::Assistant::init();
  Interpreter::init();

  ros::Subscriber sub = n.subscribe("/filename", 1000, chatterCallback);
  commandPublish = n.advertise<segbot_nlp::VoiceCommand>("command_message", 1000);
  ROS_INFO("Interpeter Node Succesfully Started !\n");
  ros::spin();

  return 0;
}
