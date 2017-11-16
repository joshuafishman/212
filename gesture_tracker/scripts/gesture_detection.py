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
user_segments = {} # {user: (last detected segment,timestamp,count)}
user_movements = {} #{user: {gesture type:last detected movement}}
user_time = {} #{user: last time printed}


def detect_segments(user,joints):
    global user_joints
    global user_movements
    global user_time
    wave_threshold = 0.15
        
    for side in ['right', 'left']:
        try:
            hand  = joints[side + '_hand']
            elbow = joints[side + '_elbow']
        except:
            print ('User {}, {} side not localized'.format(user,side))
            continue
            
        handpos, elbowpos = hand.translation, elbow.translation
        
        #if .01 < time.clock() - user_time.setdefault(user,time.clock()) < .1:
            #print(user)
            #print(side)
            #print("hand:{}".format(handpos))
            #print("elbow:{}".format(elbowpos))
            #print()
            #user_time[user] = time.clock()
            ##del user_joints[user]
        
        #detect waves
        for plane in ['x','y']:
            gtype = 'wave_'+side + plane #gesture type
            
            prev = user_movements.setdefault(user,{}).setdefault(gtype,[None,None]) #prev is the last movement associated with gesture gtype, initialized to [.5, .5]

            
            h_c = getattr(handpos, plane)  #hand coordinate in the x or y direction
            e_c = getattr(elbowpos, plane) #elbow coordinate in the x or y direction
            
            direction = handpos.z < elbowpos.z # True = down, False = up

            
            if h_c > e_c + wave_threshold:
                movement = [0,direction]
                    
            elif h_c < e_c - wave_threshold:
                movement = [1, direction]
            
            else:
                movement = prev
            
            user_movements[user][gtype] = movement
            
            if prev[0] != movement[0] and prev[1] == movement[1]:
                if direction:
                    return "Downward Wave"
                    
                else:
                    return "Upward Wave"
                    
            
def detect_gestures():
    global user_segments
    wave_time_limit = 0.02 # max time elapsed between wave segments in order for them to count as part of a gesture
    
    for user,joints in user_joints.items():
          
        seg = detect_segments(user, joints)  
        
        if seg is not None:   
            
            #update last detected gesture segment    
            t = time.clock()
            prev_seg, prev_time, prev_count = user_segments.setdefault(user,(None,0,0)) 
                   
            if seg == prev_seg and t - prev_time < wave_time_limit:
                count = prev_count + 1
            
            else:
                count = 1
            if count >= 3:
                pub.publish(seg)
                count = 0
            user_segments[user] = (seg,t, count)
            
                
                
                    
                    
                
                
        
                    
    

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
