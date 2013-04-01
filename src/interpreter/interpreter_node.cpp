#include "ros/ros.h"
#include "std_msgs/String.h"
#include "Interpreter.h"
#include "LinguisticTree.h"
#include <list>
#include "segbot_nlp/VoiceCommand.h"
#include "segbot_nlp/VoiceCommandTypes.h"
#include <string>
#include <fstream>
#include <sstream>
#include <cstdio>
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

ros::Publisher commandPublish;
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("Received filename: [%s]", msg->data.c_str());
  
  NLP::LinguisticTree tree(msg->data.c_str());
  ROS_INFO("** Interpreter Created Valid Treee");
  remove(msg->data.c_str());
  
  std::list<CommandVector> commands = Interpreter::interpret(tree);
  ROS_INFO("** Interpreter Interpreted Valid Tree");
  std::cerr << commands.size() << '\n';
  
  for (std::list<CommandVector>::iterator i = commands.begin(); i != commands.end(); ++i) {
    segbot_nlp::VoiceCommand c;
    c.commandCode = i->trueType;
    
    if (i->distance < 0) {
        c.distance = 0.00;
    } else {
      c.distance = i->distance;
    }

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
  
/**
// This is code for using Jacob's testing script lti6.pl
// This script performs a similiar functionality as Craig's.  
  std::string str = "perl /nishome/nlpros/ros/rosbuild_ws/segbot/segbot_nlp/src/debug/lti6.pl \"";
  str.append(msg->data.c_str());
  str.append("\"");
  ROS_INFO("Interpreter: Executing Command Line: %s", str.c_str());
  system(str.c_str());

  std::string result = msg->data.c_str();
  result.append(".txt"); // this is the result of the perl script

  std::ifstream file(result.c_str());
  if (file.is_open()) {
    std::string line;
    std::string loc;
    int dis;
    int ang;
    int num;
    int his;
    while (file.good()) {
        segbot_nlp::VoiceCommand c;
        std::getline(file, line);
        std::istringstream stream(line);
        if (stream >> loc >> dis >> ang >> num >> his) {
            if (loc.length() > 1) {
                for (int i = 0; i < NUM_MAP_POINTS; i ++) {
                  if (loc.compare(LocationStrings[i]) == 0) {
                    c.commandCode = GOTO;
                    c.loc = i;
                  }
                }
                
                ROS_INFO("Correctly interpreted line %s", line.c_str());
                commandPublish.publish(c);
            } else {
	      c.commandCode = MOVE;
              c.distance = (float) dis;
              c.angle = (float) ang;
              c.numTimes = (float) num;
              ROS_INFO("Correctly interpreted line %s", line.c_str());
              commandPublish.publish(c);
            }
        }
    }
  }

  file.close();
*/

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
