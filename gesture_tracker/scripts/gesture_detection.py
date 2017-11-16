#!/usr/bin/python

# 2.12 Group 5 gesture detection script
# Human Robot Comms team

# Current capabilities of this script:
# Subscribe and store human joint data for each detected person
# Next steps: 
# 1. Use that data to continually detect gestures
# 3. Pick a user to focus on (defined by the string key element in user_joints)
# 4. Publish states (ex: start sequence, stop sequence)


import rospy
import numpy as np
import cv2  # OpenCV module
import time

from geometry_msgs.msg import Point, Pose, Twist, Vector3, Quaternion
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Bool,String

#from sensor_msgs.msg import Image, CameraInfo
#from visualization_msgs.msg import Marker
#from std_msgs.msg import ColorRGBA
#from cv_bridge import CvBridge, CvBridgeError
#import message_filters


user_joints = {} #{id: {joint: transform}}
count = 0
user_movements = {} #{user: {gesture type:last detected movement}}
user_time = {} #{user: last time printed}

def detect_gestures():
    global user_joints
    global user_movements
    global user_time
    
    for user,joints in user_joints.items():        
        
        for side in ['right', 'left']:
            try:
                hand  = joints[side + '_hand']
                elbow = joints[side + '_elbow']
            except:
                print ('User {}, {} side not localized'.format(user,side))
                continue
                
            handpos, elbowpos = hand.translation, elbow.translation
            
            if .01 < time.clock() - user_time.setdefault(user,time.clock()) < .1:
                print(user)
                print(side)
                print("hand:{}".format(handpos))
                print("elbow:{}".format(elbowpos))
                print()
                user_time[user] = time.clock()
                #del user_joints[user]
            
            for direction in ['x','y']:
                gtype = side + direction #gesture type
                prev = user_movements.setdefault(user,{}).setdefault(gtype,'None')
                
                if handpos.z > elbowpos.z:
                    
                    h_c = getattr(handpos, direction)  #hand coordinate in the direction
                    e_c = getattr(elbowpos, direction) #elbow coordinate in the direction
                    
                    if h_c > e_c + .1:
                        user_movements[user][gtype] = 'wave1'
                        
                    elif h_c < e_c - .1:
                        user_movements[user][gtype] = 'wave2'
                        
                        if prev == 'wave1':
                            pub.publish("Wave!")
                
                else:
                  user_movements[user][gtype] = "None"
        
                    
    

def tfCallback(msg):
    # this callback needs to be populated with code that 
    # detects a specific gesture and publishes True when the gesture is detected
    global count
    global user_joints
    
    Transform = msg.transforms[0]
    name,ID = Transform.child_frame_id[:-2], Transform.child_frame_id[-1]
    
    user_joints.setdefault(ID,{})[name] = Transform.transform
    
    detect_gestures()
    
    if count == 200:
        startpub.publish(True)
        print("published True")
    count += 1 
    
startpub = rospy.Publisher('start_gesture_detected', Bool, queue_size=10)
pub = rospy.Publisher('gestures_detected', String, queue_size=10)

rospy.init_node('gesture_detection', anonymous=True)
startpub.publish(False)
print("published False")
rospy.Subscriber('/tf', TFMessage, tfCallback)
rospy.spin()
