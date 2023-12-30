#include "my_rosplan_interface/my_action.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_plan/PlanningAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <geometry_msgs/Twist.h>

namespace KCL_rosplan {

	MyActionInterface::MyActionInterface(ros::NodeHandle &nh) {
		// here the initialization
		goal_pub = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 10);
		cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	}

	bool MyActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		// here the implementation of the action 
		std::cout << "Going from " << msg->parameters[1].value << " to " << msg->parameters[2].value << std::endl;

		move_base_msgs::MoveBaseActionGoal goal_msg;
		geometry_msgs::Twist cmd_vel_msg;

		goal_msg.goal.target_pose.header.frame_id = "map";
		goal_msg.goal.target_pose.pose.orientation.w = 1.0;

		if (msg->name == "goto") {
			if(msg->parameters[2].value == "wp1") {
				goal_msg.goal.target_pose.pose.position.x = 6.0;
				goal_msg.goal.target_pose.pose.position.y = 2.0;
			}
			else if (msg->parameters[2].value == "wp2") {
				goal_msg.goal.target_pose.pose.position.x = 7.0;
				goal_msg.goal.target_pose.pose.position.y = -5.0;
			}
			else if (msg->parameters[2].value == "wp3") {
				goal_msg.goal.target_pose.pose.position.x = -3.0;
				goal_msg.goal.target_pose.pose.position.y = -8.0;
			}
			else if (msg->parameters[2].value == "wp4") {
				goal_msg.goal.target_pose.pose.position.x = -7.5;
				goal_msg.goal.target_pose.pose.position.y = -1.5;
			}
			else if (msg->parameters[2].value == "wp0") {
				goal_msg.goal.target_pose.pose.position.x = 0.0;
				goal_msg.goal.target_pose.pose.position.y = 1.0;
			}
			goal_pub.publish(goal_msg);
			ROS_INFO("Waypoint reached!");
		}
		else if (msg->name == "detect") {
			cmd_vel_msg.angular.z = 0.5;
			cmd_vel_pub.publish(cmd_vel_msg);
			ROS_INFO("Marker detected!");
		}

		return true;
	}
}

	int main(int argc, char **argv) {
		ros::init(argc, argv, "my_rosplan_action", ros::init_options::AnonymousName);
		ros::NodeHandle nh("~");
		KCL_rosplan::MyActionInterface my_aci(nh);
		my_aci.runActionInterface();
		return 0;
	}

