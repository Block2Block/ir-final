#!/bin/python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import random

def callback(msg):

    thr1 = 0.75 # Laser scan range threshold
    thr2 = 1.0

    # if no obstacles in front - move forward
    if msg.ranges[0] > thr1 and msg.ranges[45] > thr2 and msg.ranges[315] > thr2: # Checks if there are obstacles in front and 15 degrees left and right (Try changing the angle values as well as the thresholds)
        move.linear.x = 0.5 # go forward (linear velocity)
        move.angular.z = 0.0 # do not rotate (angular velocity)
    else:
        if msg.ranges[45] < thr2 and msg.ranges[315]< thr2:
            move.linear.x = 0.0 # donot go forward (linear velocity)
            if msg.ranges[45]==msg.ranges[315]:
                move.angular.z =random.choice([-0.75,0.75])# rotate (angular velocity) left  or right
            elif msg.ranges[45]>msg.ranges[315]:
                move.angular.z=0.5
            else:
                move.angular.z=-0.5
            if msg.ranges[0] > thr1 and msg.ranges[15] > thr2 and msg.ranges[345] > thr2:
                move.linear.x = 0.5
                move.angular.z = 0.0
                
        elif msg.ranges[45]< thr2 and msg.ranges[315]>thr2:
            move.linear.x = 0.0 # stop
            move.angular.z = -0.5 # rotate clockwise 
            if msg.ranges[0] > thr1 and msg.ranges[15] > thr2 and msg.ranges[345] > thr2:
                move.linear.x = 0.5
                move.angular.z = 0.0
            
        else:
            move.linear.x = 0.0 # stop
            move.angular.z = 0.5 # rotate counter-clockwise
            if msg.ranges[0] > thr1 and msg.ranges[15] > thr2 and msg.ranges[345] > thr2:
                move.linear.x = 0.5
                move.angular.z = 0.0
        

    pub.publish(move) # publish the move object

move = Twist() # Creates a Twist message type object
rospy.init_node('obstacle_avoidance_node') # Initialise the node
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)  # Publisher object which will publish "Twist" type messages on the "/cmd_vel" Topic, "queue_size" is the size of the outgoing message queue used for asynchronous publishing
sub = rospy.Subscriber("/scan", LaserScan, callback)  # Subscriber object which will listen "LaserScan" type messages from the "/scan" Topic and call the "callback" function each time it reads something from the Topic
rospy.spin() # Loops infinitely until someone stops the program execution
