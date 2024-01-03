#include "my_rosplan_interface/my_action.h"
#include <unistd.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_plan/PlanningAction.h>

namespace KCL_rosplan {

	MyActionInterface::MyActionInterface(ros::NodeHandle &nh) {
	
	}

	bool MyActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		
		actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
		move_base_msgs::MoveBaseGoal goal;
		ac.waitForServer();

		goal.target_pose.header.frame_id = "map";
		goal.target_pose.pose.orientation.w = 1.0;

		if (msg->name == "goto") {
		    std::cout << "Going from " << msg->parameters[1].value << " to " << msg->parameters[2].value << std::endl;
			if(msg->parameters[2].value == "wp1") {
				goal.target_pose.pose.position.x = 5.0;
				goal.target_pose.pose.position.y = 2.5;
			}
			else if (msg->parameters[2].value == "wp2") {
				goal.target_pose.pose.position.x = 7.0;
				goal.target_pose.pose.position.y = -5.0;
			}
			else if (msg->parameters[2].value == "wp3") {
				goal.target_pose.pose.position.x = -3.0;
				goal.target_pose.pose.position.y = -7.5;
			}
			else if (msg->parameters[2].value == "wp4") {
				goal.target_pose.pose.position.x = -7.5;
				goal.target_pose.pose.position.y = 1.5;
			}
			else if (msg->parameters[2].value == "wp0") {
				goal.target_pose.pose.position.x = 0.0;
				goal.target_pose.pose.position.y = 1.0;
			}
		}

		ac.sendGoal(goal);
		ac.waitForResult();
		
		ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
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
