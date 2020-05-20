#! /usr/bin/env python

import rospy
import actionlib
from mocca_motion_renderrer.msg import MoccaMotionAction, MoccaMotionGoal
import paho.mqtt.client as mqtt

def mocca_motion_client(motion_string):
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('/mocca_motion', MoccaMotionAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = MoccaMotionGoal(motion_data=motion_string)
    rospy.loginfo('goal:', goal.motion_data)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    rospy.loginfo("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("/mocca/motion/raw")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    rospy.loginfo(msg.topic+" "+str(msg.payload))
    result = mocca_motion_client(msg.payload)
    rospy.loginfo("Result:", ', '.join([str(n) for n in result.sequence]))

if __name__ == '__main__':
    try:
        rospy.init_node('mocca_motion_client')

        mqttClient = mqtt.Client()
        mqttClient.on_connect = on_connect
        mqttClient.on_message = on_message
        mqttClient.connect("121.137.95.17", 1883, 60)
        # while not mqttClient.connected_flag:
        #     time.sleep(1)
        # mqttClient.loop_stop()
        # mqttClient.disconnect()
        mqttClient.loop_start()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion", sys.stderr)