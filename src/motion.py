#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from sensor_msgs.msg import JointState
# from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult
from mocca_motion_renderrer.msg import MoccaMotionAction, MoccaMotionFeedback, MoccaMotionResult
from std_msgs.msg import Float32MultiArray, Int8, Int16MultiArray, MultiArrayLayout
import json
import time
import math


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

        self.pubAngles = rospy.Publisher('mocca_robot_goal_angle', Float32MultiArray, queue_size=10)
        self.pubPose = rospy.Publisher('mocca_robot_goal_pose', Float32MultiArray, queue_size=10)

        rospy.loginfo("Mocca motion ready");



    def execute(self, goal):
        rospy.loginfo("execute %s", str(goal));
        finished = False
        # feedback = MoccaMotionFeedback
        # result = MoccaMotionResult
        joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        joint_state = JointState()

        rate = rospy.Rate(30) # 30hz

        # self.motion = json_string

        # print(goal)

        try:
            rospy.loginfo("execute goal: %s", goal.motion_data)
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
            rospy.loginfo('total_time: %f', total_time)

            pose_start = Pose([0,0,0,0,0,0,0,0])
            pose_actual = Pose([0,0,0,0,0,0,0,0])
            pose_target = motion.frames[frame_index].pose

            pub_msg = Float32MultiArray()
            pub_msg.layout = MultiArrayLayout()
            pub_msg.layout.data_offset = int(motion.frames[frame_index].eta * 1000)
            pub_msg.data = pose_target.angles
            self.pubPose.publish(pub_msg)


            while frame_index < frame_total:
                # rospy.loginfo('fram_id: %d, total: %d', frame_index, frame_total)
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
                    # rospy.loginfo('frame_index: %d, pose: %s', frame_index, pose_target.angles)

                    pub_msg = Float32MultiArray();
                    pub_msg.layout = MultiArrayLayout()
                    pub_msg.layout.data_offset = int(motion.frames[frame_index].eta * 1000)
                    pub_msg.data = pose_target.angles
                    self.pubPose.publish(pub_msg)
                    rospy.loginfo('Publish %s', str(pub_msg))

                    # for i in range(len(pose_target.angles)):
                    #     pos = self.dxlDegToPos(pose_target.angles[i], self.DXL_DR[i])
                    #     self.dxlPosition(self.DXL_ID[i], pos)
                    #     rospy.loginfo('[DXL] angle: %f, position: %d', pose_target.angles[i], pos)
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
        except Exception as e:
            self._result.success = False
            rospy.logerr(str(e))


        rospy.loginfo('done')
        # rospy.loginfo(joint_state)
        # rate.sleep()
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
