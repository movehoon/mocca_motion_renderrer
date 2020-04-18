#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import json
import time
import math

class Pose:
    def __init__(self, angles=None):
        self.angles = angles if angles is not None else []
        # print(self.angles)

class MotionFrame:
    def __init__(self, estimated_time_to_arrival, pose=Pose):
        self.eta = estimated_time_to_arrival
        self.pose = pose

class Motion:
    def __init__(self, frames=None):
        self.frames = frames if frames is not None else []

    def append(self, frame=MotionFrame):
        self.frames.append(frame)

# def set_pose():


def mocca_motion_renderrer():
    joint_name = ['joint_head_yaw', 'joint_head_pitch', 'joint_right_1', 'joint_right_2', 'joint_right_3', 'joint_left_1',
  'joint_left_2', 'joint_left_3']
    dir = [-1, -1, -1, -1, -1, -1, -1, -1]
    joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rate = rospy.Rate(30) # 30hz

    joint_state = JointState()

    while not rospy.is_shutdown():

        with open('HI_before.json') as json_file:
            motion = Motion()

            json_obj = json.load(json_file)
            # print('json_obj')
            # print(json_obj['DoubleArrays'])
            print(len(json_obj['DoubleArrays']))
            for obj in json_obj['DoubleArrays']:
                # print(obj['array'][1:])
                pose = Pose(obj['array'][1:])
                # print(pose)
                frame = MotionFrame(obj['array'][0], pose)
                # print(frame.eta)
                motion.append(frame)

            # print('motion')
            # print(motion.frames)

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
            print('total_time: ', total_time)

            pose_start = Pose([0,0,0,0,0,0,0,0])
            pose_actual = Pose([0,0,0,0,0,0,0,0])
            pose_target = motion.frames[frame_index].pose

            while frame_index < frame_total:
                # print('fram_id:', frame_index, 'total:', frame_total)
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
                    print('frame_index:', frame_index)
                    continue

                # pose = motion.frames[frame_index].pose

                joint_state.header.stamp = rospy.Time.now()
                del joint_state.name[:]
                del joint_state.position[:]
                time_ration = 1
                if motion.frames[frame_index].eta != 0:
                    time_ratio = frame_going_time/motion.frames[frame_index].eta
                print('frame_dur:', frame_going_time, ', time_ratio:', time_ratio)
                for i in range(len(pose.angles)):
                    joint_state.name.append(joint_name[i])
                    # print('frame_going_time:', frame_going_time)
                    pose_actual.angles[i] = (pose_target.angles[i]-pose_start.angles[i])*time_ratio + pose_start.angles[i]
                    joint_state.position.append(pose_actual.angles[i]*math.pi/180*dir[i])

                print('neck:', joint_state.position[0])
                joint_pub.publish(joint_state)
                rate.sleep()

            print('done')
            # rospy.loginfo(joint_state)
            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('mocca_motion_renderrer')
        mocca_motion_renderrer()
    except rospy.ROSInterruptException:
        pass