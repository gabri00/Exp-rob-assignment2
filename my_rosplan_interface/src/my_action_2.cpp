#include "my_rosplan_interface/my_action.h"
#include <unistd.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_plan/PlanningAction.h>
#include "geometry_msgs/Twist.h"

#include <cstdlib>

namespace KCL_rosplan {


	MyActionInterface::MyActionInterface(ros::NodeHandle &nh) {
	    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	    detected_ack_sub = nh.subscribe("/ack/detected", 10, &MyActionInterface::ack_callback, this);
	    // client = nh.serviceClient<assignment_pkg::detection_srv>("/detection");
	    ack = false;
	}
	
	    void MyActionInterface::ack_callback(const std_msgs::Bool::ConstPtr& msg) {
        ack = msg->data;
    }

	bool MyActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {	
		
		geometry_msgs::Twist cmd_vel_msg;
		
		
		if (msg->name == "detect") {
		    while (!ack) {
		        cmd_vel_msg.angular.z = 0.3;
			    cmd_vel_pub.publish(cmd_vel_msg);
		    }
		    /*std::cout << "Detect " << msg->parameters[1].value << " to " << msg->parameters[2].value << std::endl;
		    srv.request.id = 11;
		    
		    do
            {
                client.call(srv);
                ROS_INFO("ACK: %d", (int)srv.response.ack);        
		        cmd_vel_msg.angular.z = 0.3;
			    cmd_vel_pub.publish(cmd_vel_msg);
            } while (!srv.response.ack);*/
       }

		
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
