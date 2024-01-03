#include <ros/ros.h>
#include "rosplan_action_interface/RPActionInterface.h"
#include "assignment_pkg/detection_srv.h"
#include "std_msgs/Bool.h"

namespace KCL_rosplan {

	class MyActionInterface: public RPActionInterface
	{

	private:
	    ros::Publisher cmd_vel_pub;
	    // ros::ServiceClient client;
		// assignment_pkg::detection_srv srv;
		ros::Subscriber detected_ack_sub;
	public:
		bool ack;
		void ack_callback(const std_msgs::Bool::ConstPtr& msg);

		/* constructor */
		MyActionInterface(ros::NodeHandle &nh);

		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}

