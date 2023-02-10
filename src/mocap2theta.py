#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped

"""
Transform mocap quaternion data into lever arm orientation
and publish current angle as theta
"""

# uses same logic as post-processing in matlab 

def quat2eul(pose_msg):
    """
    automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles
    converts quaternion to euler (roll. pitch, yaw) in radians (ccw)
    """
    p = pose_msg

    y = p.pose.orientation.y
    x = p.pose.orientation.x
    z = p.pose.orientation.z
    w = p.pose.orientation.w

    t0 = +2.0*(w*x+y*z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    # theta = roll_x - theta_0 # insert the angle we are concerned with and subtract initial angle
    # assume mocap publishes init pos as 0 deg! 
    theta = -roll_x * 180/math.pi # convert to deg
    euler = Float32MultiArray()
    euler.data = [roll_x*180/math.pi, pitch_y*180/math.pi, yaw_z*180/math.pi]
    pub.publish(euler)
    #pub.publish(pitch_y)
    #pub.publish(yaw_z)
    #pub.publish(theta)

# initializes everything
def start():
    # starts the node
    rospy.init_node('euler_node')

    # zero angle using initial theta
    # theta_0 = quat2eul() # not necessary 
    global pub	
    pub = rospy.Publisher('euler_angles_pose', Float32MultiArray, queue_size = 2)
    # subscribed to pose on topic 'vrpn_client_node/lever/pose' using callback function 'quat2eul'
    rospy.Subscriber("mavros/vision_pose/pose", PoseStamped, quat2eul)

    rospy.spin()

if __name__ == '__main__':
   start()   
