#include "my_rosplan_interface/my_action.h"
#include <unistd.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "geometry_msgs/Twist.h"

#include <cstdlib>

namespace KCL_rosplan {

	MyActionInterface::MyActionInterface(ros::NodeHandle &nh) {
	    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	    detected_ack_sub = nh.subscribe("/ack/detected", 10, &MyActionInterface::ack_callback, this);
	    ack = false;
	}
	
	void MyActionInterface::ack_callback(const std_msgs::Bool::ConstPtr& msg) {
        ack = msg->data;
    }

	bool MyActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		if (msg->name == "goto") {
			ROS_INFO("GOING FROM %s TO %s", msg->parameters[1].value.c_str(), msg->parameters[2].value.c_str());
			
			actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
			move_base_msgs::MoveBaseGoal goal;
			ac.waitForServer();

			goal.target_pose.header.frame_id = "map";
			goal.target_pose.pose.orientation.w = 1.0;

			if(msg->parameters[2].value == "wp1") {
				goal.target_pose.pose.position.x = 5.0;
				goal.target_pose.pose.position.y = 2.5;
			}
			else if (msg->parameters[2].value == "wp2") {
				goal.target_pose.pose.position.x = 7.0;
				goal.target_pose.pose.position.y = -5.0;
			}
			else if (msg->parameters[2].value == "wp3") {
				goal.target_pose.pose.position.x = -3.3;
				goal.target_pose.pose.position.y = -7.8;
			}
			else if (msg->parameters[2].value == "wp4") {
				goal.target_pose.pose.position.x = -7.5;
				goal.target_pose.pose.position.y = 1.5;
			}
			else if (msg->parameters[2].value == "wp0") {
				goal.target_pose.pose.position.x = 0.0;
				goal.target_pose.pose.position.y = 1.0;
			}

			ac.sendGoal(goal);
			ac.waitForResult();
		}
		else if (msg->name == "detect") {
			ROS_INFO("DETECTING %s", msg->parameters[2].value.c_str());

			geometry_msgs::Twist cmd_vel_msg;

		    while (!ack) {
		        cmd_vel_msg.angular.z = 0.3;
			    cmd_vel_pub.publish(cmd_vel_msg);
		    }
		}
		
		ROS_INFO("ACTION (%s) COMPLETED", msg->name.c_str());
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
