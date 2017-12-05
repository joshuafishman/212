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
user_segments = {} # {user: (last detected segment,timestamp,count)}
user_movements = {} #{user: {gesture type:last detected movement}}
#user_time = {} #{user: last time printed}


def line(p1,p2):
    """
    Get an array representing the parametric equation for a line from 2 points
    If the parametric equation is {(x,y,z) = (px+n*tx,py+n*ty,pz+n*tz) for any n}, the array returned would be ([px,py,pz], [tx,ty,tz])
    Inputs:
        p1: set of x,y,z coordinates
        p2: set of x,y,z coordinates
    Outputs:
        2x3 line array
    """
    p = np.array(p1)
    t = np.array(p2) - np.array (p1)
    return (p,t) 

def detect_segments(user,joints):
    global user_joints
    global user_movements
    #global user_time
    wave_threshold = 0.1
    
    get_prev_movement = lambda gtype, default: user_movements.setdefault(user,{}).setdefault(gtype,default)
    getxyz = lambda t: np.array([t.x, t.y, t.z]) 
    
    torsoxyz = getxyz(joints['torso'].translation)
        
    for side in ['right', 'left']:
        try:
            hand  = joints[side + '_hand']
            elbow = joints[side + '_elbow']
            shoulder = joints[side + '_shoulder']
        except:
            print ('User {}, {} side not localized'.format(user,side))
            continue
            
        handpos, elbowpos, shoulderpos = hand.translation, elbow.translation, shoulder.translation
        shoulderxyz, elbowxyz,handxyz = getxyz(shoulderpos), getxyz(elbowpos), getxyz(handpos)
                   
        vertical_direction = "Downward" if handpos.z < elbowpos.z else "Upward"
        
        #if .01 < time.clock() - user_time.setdefault(user,time.clock()) < .1:
            #print(user)
            #print(side)
            #print("hand:{}".format(handpos))
            #print("elbow:{}".format(elbowpos))
            #print()
            #user_time[user] = time.clock()            
        
        #detect waves
        
        shoulder_line = line(torsoxyz[:2], shoulderxyz[:2]) #xy line from torso to shoulder
        shoulder_vec  = shoulder_line[1]/np.linalg.norm(shoulder_line[1]) #unit vector
        
        armline = line(elbowxyz[:2], handxyz[:2]) #xy line from elbow to wrist
        
        
        ### parallel ###
        
        armvec_parallel_mag = np.dot(armline[1], shoulder_vec)  #magnitude of arm vector parallel to the shoulder
        
        gtype = 'wave_'+side + '_parallel'
        parallel_prev = get_prev_movement(gtype, [None,None]) #prev is the last movement associated with gesture gtype
                                                              #initialized to [None, None]
                                        
        if armvec_parallel_mag > wave_threshold:
            parallel_movement = ['Outward', vertical_direction]
        
        elif armvec_parallel_mag < -wave_threshold:
            parallel_movement = ['Inward', vertical_direction]
        
        else:
            parallel_movement = parallel_prev


        user_movements[user][gtype] = parallel_movement
        
        
        if parallel_prev[0] != parallel_movement[0] and parallel_prev[1] == parallel_movement[1]:
            #normal wave
            return parallel_movement[1] + " Parallel Wave"
        
        if parallel_prev[0] == parallel_movement[0] and parallel_prev[1] != parallel_movement[1]:
            #sideways wave
            return parallel_movement[0] + " Parallel Wave"
            

        #XY wave finding
        #for plane in ['x','y']:
            #gtype = 'wave_'+side + plane #gesture type
            
            #prev = user_movements.setdefault(user,{}).setdefault(gtype,[None,None]) #prev is the last movement associated with gesture gtype, initialized to [None, None]
            
            #h_c = getattr(handpos, plane)  #hand coordinate in the x or y vertical_direction
            #e_c = getattr(elbowpos, plane) #elbow coordinate in the x or y vertical_direction
            
            #if h_c > e_c + wave_threshold:
                #movement = [0,vertical_direction]
                    
            #elif h_c < e_c - wave_threshold:
                #movement = [1, vertical_direction]
            
            #else:
                #movement = prev
            
            #user_movements[user][gtype] = movement
            
            #if prev[0] != movement[0] and prev[1] == movement[1]:
                #if vertical_direction:
                    #return "Downward Wave"
                    
                #else:
                    #return "Upward Wave"
                    
        ###Stationary gestures###
        min_dist = .1
        closed_pos  = [0.86,-0.26,-0.23]
        open_pos    = [0,.4,-.35]
        gripper_pos = [0,0,-.35]
        
        gtype = "stationary_"+side
        stationary_prev = get_prev_movement(gtype,handxyz)
        user_movements[user][gtype] = stationary_prev #don't change stored movement so we can see if the hand moves
        
        if np.linalg.norm(stationary_prev-handxyz) < min_dist: #hand is basically stationary
            
            if np.linalg.norm(handxyz-closed_pos) < min_dist:
                return "On Closed Drawer"
            if np.linalg.norm(handxyz-open_pos) < min_dist:
                return "On Open Drawer"
            if np.linalg.norm(handxyz-gripper_pos) < min_dist:
                return "On Gripper"
           
        user_movements[user][gtype] = handxyz #if the hand is not in a designated location, reset movement
            
def detect_gestures():
    global user_segments
    gesture_min_time =   0.005
    gesture_time_limit = 0.05 # max time elapsed between wave segments in order for them to count as part of a gesture
    gesture_count = 3 #segments in a gesture
    
    for user,joints in user_joints.items():
          
        seg = detect_segments(user, joints)  
        
        if seg is not None:   
            
            
            #update last detected gesture segment    
            t = time.clock()
            prev_seg, prev_time, prev_count = user_segments.setdefault(user,(None,0,0)) 
            
            dt =  t-prev_time
            
            testpub.publish(str(dt)+' '+seg)       
                   
            if dt < gesture_min_time:
                continue 
                
            if seg == prev_seg and  dt < gesture_time_limit:
                count = prev_count + 1
            
            else:
                count = 1
                
            if count >= gesture_count:
                pub.publish(str(t)+' '+seg)
                count = 0
                
            user_segments[user] = (seg,t, count)
            
                

def tfCallback(msg):
    # this callback needs to be populated with code that 
    # detects a specific gesture and publishes True when the gesture is detected
    global user_joints
    
    Transform = msg.transforms[0]
    name,ID = Transform.child_frame_id[:-2], Transform.child_frame_id[-1]
    
    user_joints.setdefault(ID,{})[name] = Transform.transform
    
    detect_gestures()
    

    
testpub = rospy.Publisher('segments', String, queue_size=10)
pub = rospy.Publisher('gestures_detected', String, queue_size=10)

rospy.init_node('gesture_detection', anonymous=True)
rospy.Subscriber('/tf', TFMessage, tfCallback)
rospy.spin()
