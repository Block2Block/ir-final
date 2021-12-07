#!/bin/python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import json

def set_xy(input):
    global loaded_dict
    with open('/home/coziant/catkin_ws/src/ir-final/scripts/data.json') as json_file:
        loaded_dict = json.load(json_file)

    xy_cord = loaded_dict[input]
    print(xy_cord)

    global x_goal
    x_goal = xy_cord[0][0]
    print(x_goal)
    global y_goal
    y_goal = xy_cord[0][1]
    print(y_goal)

def movebase_client():

    # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

    # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # Set X, Y coodiandate
    goal.target_pose.pose.position.x = x_goal
    goal.target_pose.pose.position.y = y_goal

    # Set orientation
    # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1

    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
    # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()   

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    #os.system("rosnode kill qr_scan_node")
    #print(loaded_dict)
    
    try:
        # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        
        rospy.init_node('movebase_client_py')

        user_input = input('MILK, FRUITS, SNACK? ')
        if user_input == 'MILK':
            set_xy(user_input)
        if user_input == 'FRUITS':
            set_xy(user_input)
        if user_input == 'SNACK':
            set_xy(user_input)

        result = movebase_client()
        if result:
            rospy.loginfo("Goal Reached!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
