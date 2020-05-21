#! /usr/bin/env python

import rospy
import actionlib
from mocca_motion_renderrer.msg import MoccaMotionAction, MoccaMotionGoal

def mocca_motion_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('/mocca_motion', MoccaMotionAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    with open('/home/parallels/OK.json') as json_file:
        goal = MoccaMotionGoal(motion_data=json_file.read())
        rospy.loginfo('goal:%s', goal.motion_data)

        # Sends the goal to the action server.
        client.send_goal(goal)

        # Waits for the server to finish performing the action.
        client.wait_for_result()

        # Prints out the result of executing the action
        return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('mocca_motion_client')
        result = mocca_motion_client()
        rospy.loginfo("Result: %s", str(result))
    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion", sys.stderr)