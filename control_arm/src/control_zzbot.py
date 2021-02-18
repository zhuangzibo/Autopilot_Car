#!/usr/bin/env python

import rospy,roslib
from std_msgs.msg import Float64,String
import sys



def talker():
    rospy.init_node('talker', anonymous=True)
    left=rospy.Publisher("/zzbot/left_velocity_controller/command", Float64, queue_size = 10)
    right=rospy.Publisher("/zzbot/right_velocity_controller/command", Float64, queue_size = 10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        hello_str = "hello world " 
        left.publish(1)
        right.publish(-1)
        rate.sleep()

talker()