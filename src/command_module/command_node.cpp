#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "segbot_nlp/VoiceCommand.h"
#include "segbot_nlp/VoiceCommandTypes.h"
#include "Interpreter.h"
#include <queue> 
#include <cmath>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef ros::NodeHandle NodeHandle;
typedef ros::Publisher Publisher;
typedef ros::Subscriber Subscriber; 
typedef move_base_msgs::MoveBaseGoal MoveBaseGoal;

#define PI 3.14159

enum FrameType {MAP_FRAME, ROBOT_FRAME}; 
// map will mape the movement to a location on the map
// robot will map the movement to the current location of the robot

void debug_fill_fifo(void); // fills the fifo as a test with three or four commands
MoveBaseGoal createGoal(FrameType frame, float x, float y, float z, float w);

float calcZ(float angle, float sign) {
    return sin((sign * angle * PI)/360.00);
}

float calcW(float angle, float sign) {
    return cos((sign * angle * PI)/360.00);
}

std::queue<move_base_msgs::MoveBaseGoal> GoalFifo;
void messageHandle(const segbot_nlp::VoiceCommand::ConstPtr& msg) {
	int opcode = msg->commandCode;
	float distance = msg->distance;
	float angle = msg->angle;
	int location = msg->loc;
	int numTimes = msg->numTimes;

	// angle calculations
        float z;
        float w;
        float sign;
        
        if (angle < 0) { 
          sign = -1.00;
          angle = -angle;
        } else {
          sign = 1.00;
        }

	MoveBaseGoal newGoal;
        
        for (int j = 0; j < numTimes; j ++) {
  	switch (opcode) {
	  	case RC_move:
			if (location != L_noSet) {
                          newGoal = createGoal(MAP_FRAME, LocationPoints[location].x, LocationPoints[location].y, 0.0, 1.0);
			  GoalFifo.push(newGoal);
                        } else {
                          int numAngleTurns = (int) (angle / 20);
                          if (numAngleTurns < 0) { numAngleTurns = -numAngleTurns; }
                          int numMovements = (int) (distance);

                          while(angle >= 170.00)  {
                            newGoal = createGoal(ROBOT_FRAME, 0.0, 0.0, calcZ(90.00, sign), calcW(90.00, sign));
   			    GoalFifo.push(newGoal);
                            angle -= 90.00;
                          }
                          newGoal = createGoal(ROBOT_FRAME, 0.0, 0.0, calcZ(angle, sign), calcW(angle, sign));
   			  GoalFifo.push(newGoal);
                          
                        for (int i = 0; i < numMovements; i ++) {
			    newGoal = createGoal(ROBOT_FRAME, 1.0, 0.0, 0.0, 1.0);
			    GoalFifo.push(newGoal);
                          }
                        }  			
			
                        break;
		case RC_goBack:
		        break;
 
                case RC_dance:
                        newGoal = createGoal(ROBOT_FRAME, 1.0, 0.0, 1.0, 0.0);
                        GoalFifo.push(newGoal);
                        
                        newGoal = createGoal(ROBOT_FRAME, -1.0, 0.0, 1.0, 0.0);
                        GoalFifo.push(newGoal);
                
                        newGoal = createGoal(ROBOT_FRAME, 1.0, 0.0, 1.0, 0.0);
                        GoalFifo.push(newGoal);
 
                        break;
                case RC_stop:
			break;

		default:
			break;
	}
        }

}


int main(int argc, char** argv){
  ros::init(argc, argv, "command_module");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  NodeHandle handler;
  Subscriber s = handler.subscribe("command_message", 1000, messageHandle);

   ROS_INFO("** Command Moudule: On-line");
  

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0)) && ros::ok()){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  ROS_INFO("** Command Module: Waiting for Valid Commands");

  move_base_msgs::MoveBaseGoal goal;
  //debug_fill_fifo();

  //we'll send a goal to the robot to move 1 meter forward

  ros::Rate loop_rate(10);
  while (ros::ok()) {

	if (!GoalFifo.empty()) {
		goal = GoalFifo.front();
		goal.target_pose.header.stamp = ros::Time::now();

		ROS_INFO("** Command Module: Sending Goal");
	
		ac.sendGoal(goal);

		ac.waitForResult();

		if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
			ROS_INFO("** Command Module: Goal Succeeded");
                        GoalFifo.pop();
		} else {
			ROS_INFO("** Command Module: Failure! Clearing the Fifo");
		        while (! GoalFifo.empty()) { GoalFifo.pop(); }
                }

	}

	ros::spinOnce();
	loop_rate.sleep();
  }
	

  return 0;
}

void debug_fill_fifo(void) {
	MoveBaseGoal goal;
	goal = createGoal(ROBOT_FRAME, 0.0, 0.0, 1.0, 0.0);
	GoalFifo.push(goal);

	goal = createGoal(ROBOT_FRAME, 0.0, 0.0, 1.41, 1.41);
	GoalFifo.push(goal);

	goal = createGoal(MAP_FRAME, 4.0, 4.0, 1.41, 1.41);
	GoalFifo.push(goal);

	goal = createGoal(ROBOT_FRAME, 0.0, 0.0, 0.0, 1.0);
	GoalFifo.push(goal);
}  
		
MoveBaseGoal createGoal(FrameType frame, float x, float y, float z, float w) {
	MoveBaseGoal goal;

	float val2 = (z*z) + (w*w);

	if ((val2 != 1)) {
		ROS_INFO("** Command Module: Inapproriate Goal asked for");
		//return NULL;
	}
	
	if (frame == MAP_FRAME) {
		goal.target_pose.header.frame_id = "/map";
	} else if (frame == ROBOT_FRAME) {
		goal.target_pose.header.frame_id = "/base_link";
	}

 	//goal.target_pose.header.stamp = ros::Time::now();

  	goal.target_pose.pose.position.x = x;
	goal.target_pose.pose.position.y = y;
	goal.target_pose.pose.position.z = 0.0;

	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = z;
	goal.target_pose.pose.orientation.w = w;

	return goal;
}

/*
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  return 0;
}*/

