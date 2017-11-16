#!/usr/bin/python

# State detection script
# using information from Start Gesture Boolean topic and other topics,
# determines current state of the robot
# ex: retrieving bottle, stop running, etc.

import rospy
import numpy as np
from std_msgs.msg import Bool

def StartGestureCallback(msg):
    # this callback 
    start_gesture = msg.data
    print(start_gesture)
    if start_gesture == True:
        print("Start!")
    else:
        print("not boolean")

rospy.init_node('state_detection', anonymous=True)
rospy.Subscriber('/start_gesture_detected', Bool, StartGestureCallback)
rospy.spin()
