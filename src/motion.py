#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from sensor_msgs.msg import JointState
# from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult
from mocca_motion_renderrer.msg import MoccaMotionAction, MoccaMotionFeedback, MoccaMotionResult
import json
import time
import math

from dynamixel_sdk import *                    # Uses Dynamixel SDK library


# Control table address
ADDR_AX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_AX_GOAL_POSITION      = 30
ADDR_AX_PRESENT_POSITION   = 36

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 11                 # Dynamixel ID : 1
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque



class Pose:
    def __init__(self, angles=None):
        self.angles = angles if angles is not None else []
        # rospy.loginfo(self.angles)

class MotionFrame:
    def __init__(self, estimated_time_to_arrival, pose=Pose):
        self.eta = estimated_time_to_arrival
        self.pose = pose

class Motion:
    def __init__(self, frames=None):
        self.frames = frames if frames is not None else []

    def append(self, frame=MotionFrame):
        self.frames.append(frame)

    def loadJson(self, jsonString):
        json_obj = json.loads(jsonString)
        # rospy.loginfo('json_obj')
        # rospy.loginfo(json_obj['DoubleArrays'])
        # rospy.loginfo(len(json_obj['DoubleArrays']))
        for obj in json_obj['DoubleArrays']:
            # rospy.loginfo(obj['array'][1:])
            pose = Pose(obj['array'])
            # rospy.loginfo(pose)
            frame = MotionFrame(obj['time'], pose)
            # rospy.loginfo(frame.eta)
            self.append(frame)



class MoccaMotion():

    _feedback = MoccaMotionFeedback()
    _result = MoccaMotionResult()

    def dxlDegToPos(self, degree, direction):
        position = 512 + (degree*3.45) * direction
        return int(position)

    def dxlInit(self):
        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(DEVICENAME)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # Open port
        try :
            if self.portHandler.openPort():
                rospy.loginfo("Succeeded to open the port")
            else:
                rospy.logerr("Failed to open the port %s", DEVICENAME)
                return
        except:
            rospy.logerr("Failed to open the port %s", DEVICENAME)
            return

        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            rospy.loginfo("Succeeded to change the baudrate")
        else:
            rospy.logerr("Failed to change the baudrate %s", BAUDRATE)
            return


    def dxlEnabble(self, dxl_id, enable):
        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_AX_TORQUE_ENABLE, enable)
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            rospy.logerr("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            rospy.loginfo("Dynamixel has been successfully connected")


    def dxlPosition(self, dxl_id, position):
        # Write goal position
        # print('[dxlPosition]dxl_id:', dxl_id, 'position:', position)
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, ADDR_AX_GOAL_POSITION, position)
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            rospy.loginfo("%s" % self.packetHandler.getRxPacketError(dxl_error))



    def __init__(self):
        self.joint_name = ['joint_left_1', 'joint_left_2', 'joint_left_3', \
                          'joint_right_1', 'joint_right_2', 'joint_right_3', \
                          'joint_head_yaw', 'joint_head_pitch']
        self.dir = [1, -1, -1, 1, -1, -1, -1, 1]

        self.server = actionlib.SimpleActionServer(
            '/mocca_motion',
            MoccaMotionAction,
            self.execute,
            False)
        self.server.start()

        self.dxlInit()
        # self.dxlEnabble(DXL_ID, TORQUE_ENABLE)
        # self.dxlPosition(DXL_ID, 512)

        rospy.loginfo("Mocca motion ready");



    def execute(self, goal):
        finished = False
        # feedback = MoccaMotionFeedback
        # result = MoccaMotionResult
        joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        joint_state = JointState()

        rate = rospy.Rate(30) # 30hz

        # self.motion = json_string

        # print(goal)

        try:
            rospy.loginfo("execute goal: ", goal.motion_data)
            motion = Motion()
            motion.loadJson(goal.motion_data)

            start_time = time.time()
            total_time = 0
            going_time = 0

            frame_index = 0
            frame_total = len(motion.frames)
            frame_start_time = time.time()
            frame_going_time = 0
            frame_target_time = motion.frames[0].eta
            for f in motion.frames:
                total_time = total_time + f.eta
            rospy.loginfo('total_time: ', total_time)

            pose_start = Pose([0,0,0,0,0,0,0,0])
            pose_actual = Pose([0,0,0,0,0,0,0,0])
            pose_target = motion.frames[frame_index].pose

            while frame_index < frame_total:
                # rospy.loginfo('fram_id:', frame_index, 'total:', frame_total)
                going_time = time.time() - start_time
                frame_going_time = time.time() - frame_start_time
                frame_target_time = motion.frames[frame_index].eta

                if (frame_going_time > motion.frames[frame_index].eta):
                    pose_start = motion.frames[frame_index].pose
                    frame_start_time = time.time()
                    frame_index = frame_index + 1
                    if (frame_index >= frame_total):
                        break
                    pose_target = motion.frames[frame_index].pose
                    rospy.loginfo('frame_index:', frame_index, 'pose:', pose_target.angles)

                    # pos = self.dxlDegToPos(pose_target.angles[7], -1)
                    # self.dxlPosition(DXL_ID, pos)
                    # rospy.loginfo('[DXL]', pose_target.angles[6], ' => ', pos)
                    continue

                del joint_state.name[:]
                del joint_state.position[:]
                time_ration = 1
                if motion.frames[frame_index].eta != 0:
                    time_ratio = frame_going_time/motion.frames[frame_index].eta
                # rospy.loginfo('frame_dur:', frame_going_time, ', time_ratio:', time_ratio)
                for i in range(len(motion.frames[0].pose.angles)):
                    joint_state.name.append(self.joint_name[i])
                    # rospy.loginfo('frame_going_time:', frame_going_time)
                    pose_actual.angles[i] = (pose_target.angles[i]-pose_start.angles[i])*time_ratio + pose_start.angles[i]
                    joint_state.position.append(pose_actual.angles[i]*math.pi/180*self.dir[i])

                # rospy.loginfo('neck:', joint_state.position[0])
                joint_state.header.stamp = rospy.Time.now()
                joint_pub.publish(joint_state)
                self._feedback.processing = going_time / total_time
                self.server.publish_feedback(self._feedback)
                rate.sleep()
            self._result.success = True
            rospy.loginfo('success')
        except:
            self._result.success = False
            rospy.logerr('fail')


        rospy.loginfo('done')
        # rospy.loginfo(joint_state)
        rate.sleep()
        self.server.set_succeeded(self._result)


if __name__ == '__main__':
    try:
        rospy.init_node('mocca_motion_renderrer')
        server = MoccaMotion()
        rospy.spin()
        # with open('HI_after.json') as json_file:
        #     server.execute(json_file)
    except rospy.ROSInterruptException:
        pass
