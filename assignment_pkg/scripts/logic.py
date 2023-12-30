#!/usr/bin/env python

import rospy
import time

from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_srvs.srv import Empty

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller')
        
    def call_service(self, service_name):
        rospy.wait_for_service(service_name)
        try:
            service_proxy = rospy.ServiceProxy(service_name, Empty)
            response = service_proxy()
            return response
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

def main():
    time.sleep(10)
    r = RobotController()
    r.call_service("/rosplan_problem_interface/problem_generation_server")
    r.call_service("/rosplan_planner_interface/planning_server")
    r.call_service("/rosplan_parsing_interface/parse_plan")

    rospy.spin()

if __name__ == '__main__':
    main()

