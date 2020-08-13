#!/usr/bin/env python

from tf import *
from tf.transformations import quaternion_from_matrix, rotation_matrix, translation_from_matrix, quaternion_matrix, rotation_from_matrix

from constrained_motion_planning import SuhanMotionPlanner, get_path_from_url, NameVector, NameMap, ParamVector, to_trajectory_msgs
from constrained_motion_planning.suhan_utils import joint_names, display
# from ..params.default_parameters import T_DXL_CTR
import moveit_commander

import numpy as np
import rospy
from math import pi
import rospkg
import time
import sys
import copy

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from matplotlib import pyplot as plt

class SuhanMotionPlannerManager:
    def __init__(self, argv):
        self.is_real = False #to skip 'is_initial_joint_updated' when not using real robot
        self.real_arm_num = 3
        self.arm_num = 3
        moveit_commander.roscpp_initialize(argv)
        self.is_initial_joint_updated = [False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False]
        self.planner = SuhanMotionPlanner()
        self.max_velocity_scaling_factor = 0.3    #0.9      
        self.max_acceleration_scaling_factor = 1.0     #0.35
        
        # self.joint_states = np.zeros(7*self.arm_num)
        self.joint_states = np.array([0, -0.785, 0, -1.571, 0, 1.571, 0.785, 0, -0.785, 0, -1.571, 0, 1.571, 0.785, 0, -0.785, 0, -1.571, 0, 1.571, 0.785])
        if self.real_arm_num == 2:
            self.joint_states[7*2:7*3] = np.array([0, 0, 0, -pi/3, 0, pi/3, pi/4])
            self.is_initial_joint_updated[7*2:7*3] = [True, True, True, True, True, True, True]
        self.joint_subscriber = rospy.Subscriber('/panda_dual/joint_states', JointState, self.__joint_state_callback)
        # TF
        self.listener = TransformListener()
        self.transformer = TransformerROS()
        self.broadcaster = TransformBroadcaster()

        arm = NameMap()
        obj = NameMap()
        arm['panda_left'] = 0
        arm['panda_right'] = 1
        arm['panda_top'] = 2

        self.hand_touch_links = {}
        self.hand_touch_links['panda_left'] = NameVector()
        self.hand_touch_links['panda_right'] = NameVector()
        self.hand_touch_links['panda_top'] = NameVector()
        self.hand_touch_links['panda_left'].append('panda_left_hand')
        self.hand_touch_links['panda_left'].append('panda_left_leftfinger')
        self.hand_touch_links['panda_left'].append('panda_left_rightfinger')
        self.hand_touch_links['panda_left'].append('panda_left_link7')
        self.hand_touch_links['panda_left'].append('panda_left_link8')
        self.hand_touch_links['panda_right'].append('panda_right_hand')
        self.hand_touch_links['panda_right'].append('panda_right_finger_left_link')
        self.hand_touch_links['panda_right'].append('panda_right_finger_right_link')
        self.hand_touch_links['panda_right'].append('panda_right_link7')
        self.hand_touch_links['panda_right'].append('panda_right_link8')
        self.hand_touch_links['panda_top'].append('panda_top_hand')
        self.hand_touch_links['panda_top'].append('panda_top_leftfinger')
        self.hand_touch_links['panda_top'].append('panda_top_rightfinger')
        self.hand_touch_links['panda_top'].append('panda_top_link7')
        self.hand_touch_links['panda_top'].append('panda_top_link8')

        eye = np.array([0,0,0,1])
        reversed_eye = np.array([1,0,0,0])
        # q_arm = np.array([0, 0, 0, -pi/2, 0, pi/2, pi/4, 0, 0, 0, -pi/2, 0, pi/2, pi/4, 0, 0, 0, -pi/2, 0, pi/2, pi/4])
        q_arm = np.array([0, -0.785, 0, -1.571, 0, 1.571, 0.785, 0, -0.785, 0, -1.571, 0, 1.571, 0.785, 0, -0.785, 0, -1.571, 0, 1.571, 0.785])
        ee_pos = np.array([0, 0, 0.103])
        ee_quat = np.array([0,0,0,1])

        # planner needs to be initialized
        self.planner.set_environment(arm, obj)
        self.planner.set_sigma(0.8)
        self.planner.set_max_ik_trials(2000)
        self.planner.set_max_planning_time(20)
        
        self.planner.set_robot_transform('panda_left', np.array([0, 0.3, 1.006]), eye)
        self.planner.set_robot_transform('panda_right', np.array([0, -0.3, 1.006]), eye)
        self.planner.set_robot_transform('panda_top', np.array([1.35, 0.3, 1.006]), np.array([0,0,1,0]))


        DXL_RAD = np.deg2rad(13.5)
        T_DXL_C = np.cos(DXL_RAD)
        T_DXL_S = np.sin(DXL_RAD)
        T_DXL_CTR = np.array([[T_DXL_C, 0, -T_DXL_S, 0.0129],
                        [0, 1.0000, 0, 0], 
                        [T_DXL_S, 0, T_DXL_C, 0.1132], 
                        [0, 0, 0, 1.0000]])
        pos, quat = self.__convert_to_vec_quat(T_DXL_CTR)
        print (pos, quat)

        self.planner.set_end_effector_frame('panda_left', ee_pos, ee_quat)
        self.planner.set_end_effector_frame('panda_right', pos, quat)
        self.planner.set_end_effector_frame('panda_top', ee_pos, ee_quat)
        self.planner.set_end_effector_margin('panda_left', np.array([0,0, 0.03]), eye)
        self.planner.set_end_effector_margin('panda_right', np.array([0,0, 0.03]), eye)
        self.planner.set_end_effector_margin('panda_top', np.array([0,0, 0.03]), eye)

        self.planner.add_box(np.array([0.35, 0.20, 0.16]), 'sub_table', np.array([0.72, -0.04, 1.0805]), np.array([0, 0, 0, 1]))
        self.planner.add_box(np.array([0.20, 0.15, 0.16]), 'sub_table2', np.array([0.40, -0.55, 1.0805]), np.array([0, 0, 0, 1]))
        self.planner.add_box(np.array([0.15, 0.20, 0.16]), 'sub_table3', np.array([0.6, 0.4, 1.0805]), np.array([0, 0, 0, 1]))
        self.planner.add_box(np.array([0.20, 0.20, 0.16]), 'sub_table4', np.array([0.44, 0.075, 1.0805]), np.array([0, 0, 0, 1]))
        

        self.planner.publish_planning_scene_msg()
        
        # for the first state
        self.planner.set_start_arm_states(q_arm)
        self.planner.print_start_state()
        # for the visualization
        self.planner.update_arm_states(q_arm)
        # self.planner.enable_tf_broadcasting()
        self.planner.publish_planning_scene_msg()


        if self.is_real:
            is_completed = False
            while is_completed is False:
                is_completed = True
                for value in self.is_initial_joint_updated:
                    if value is False:
                        is_completed = False
                    

    def __del__(self):
        moveit_commander.roscpp_shutdown()

    def __joint_state_callback(self, data):
        for i in range(0,len(joint_names)):
            for j in range(0, len(data.name)):
                if joint_names[i] == data.name[j]:
                    self.joint_states[i] = data.position[j]
                    self.is_initial_joint_updated[i] = True
        # print(self.joint_states)


    def __update_current_state(self):
        # TODO: implement this code (insepct TF, /panda_dual/joint_states)
        # Noise option (?)
        # self.planner.set_start_arm_states(self.joint_states)
        # self.planner.update_arm_states(self.joint_states)
        # for col_object in self.object_lists:
            # get obj pose
            # pos = ?
            # quat = ?
            # self.planner.update_object_states(col_object, pos, quat)
            # self.planner.set_start_object_states(col_object, pos, quat)
            # pass
        # pass
        self.print_current_collision_info()

    def __convert_to_name_vector(self, name_list):
        name_vector = NameVector()
        for name in name_list:
            name_vector.append(name)
        return name_vector

    def __convert_to_param_vector(self, param_list):
        param_vector = ParamVector()
        for param in param_list:
            param_vector.append(param)
        return param_vector

    def __convert_to_vec_quat(self, matrix):
        pos = np.empty(3)
        pos[0:3] = matrix[0:3, 3].T
        quat = quaternion_from_matrix(matrix)

        return pos, quat

    def attach_object(self, arm_name, object_id):
        self.__update_current_state() 
        self.planner.attach_object(object_id, arm_name + '_hand', self.hand_touch_links[arm_name])
        
    def detach_object(self, arm_name, object_id):
        self.__update_current_state() 
        self.planner.detach_object(object_id, arm_name + '_hand')

    def print_current_collision_info(self):
        self.planner.print_current_collision_infos()

    def plan_joint_pose(self, arm_name, joint_pose):
        self.__update_current_state() 
        r = self.planner.solve_for_joint_pose(arm_name, joint_pose)
        if r == False:
            return None
        
        self.planner.time_parameterize(self.max_velocity_scaling_factor, self.max_acceleration_scaling_factor)
        trajectory = to_trajectory_msgs(self.planner)
        return trajectory

    def plan_joint_pose_all(self, joint_pose):
        if len(joint_pose) is not 7*self.arm_num:
            print('error: len(joint_pose) is not', str(7*self.arm_num) ,'but', len(joint_pose))
            return None

        self.__update_current_state() 
        r = self.planner.solve_for_joint_pose_all(joint_pose)
        if r == False:
            return None

        self.planner.time_parameterize(self.max_velocity_scaling_factor, self.max_acceleration_scaling_factor)
        trajectory = to_trajectory_msgs(self.planner)
        return trajectory

    def plan_target_pose(self, arm_name, target_pos, target_quat):
        self.__update_current_state() 
        r = self.planner.solve_for_ik_pose(arm_name, target_pos, target_quat)
        if r == False:
            return None
        
        self.planner.time_parameterize(self.max_velocity_scaling_factor, self.max_acceleration_scaling_factor)
        trajectory = to_trajectory_msgs(self.planner)
        return trajectory

    def plan_object_ik(self, arm_name, object_id, grasp_param = 0.7):
        self.__update_current_state()
        self.planner.print_start_state()
        r = self.planner.solve_for_object_ik_pose(arm_name, object_id, grasp_param)
        # print(r)
        if r == False:
            return None
        self.planner.time_parameterize(self.max_velocity_scaling_factor, self.max_acceleration_scaling_factor)
        trajectory = to_trajectory_msgs(self.planner)
        return trajectory
    
    # The relative angle between two assembly frames is pi w.r.t x-axis
    def align_assembly_frames(self, assemble_quat):
        rot = quaternion_matrix(assemble_quat)
        rot_x = np.array([[1.0, 0.0, 0.0, 0.0],
                        [0.0, -1.0, 0.0, 0.0],
                        [0.0, 0.0, -1.0, 0.0],
                        [0.0, 0.0, 0.0, 1.0]])
        rot = np.dot(rot, rot_x)
        new_assemble_quat = quaternion_from_matrix(rot)
        return new_assemble_quat

    def add_assembly_margin(self, assemble_pos, assemble_quat, margin):        
        rot = quaternion_matrix(assemble_quat)[0:3,0:3]                
        p_margin = np.array([0, 0, margin])
        new_assemble_pos = assemble_pos + np.dot(rot, p_margin)
        dis = np.linalg.norm(assemble_pos)
        new_dis = np.linalg.norm(new_assemble_pos)
        if dis > new_dis:
            p_margin = np.array([0, 0, -margin])
            new_assemble_pos = assemble_pos + np.dot(rot, p_margin)
        else:
            new_assemble_pos = new_assemble_pos

        new_assemble_pos = list(new_assemble_pos)
        
        return new_assemble_pos

    def pose_msgs_to_numpy_tf(self, pose_msgs):
        pos = np.array([pose_msgs.position.x, pose_msgs.position.y, pose_msgs.position.z])
        quat = np.array([pose_msgs.orientation.x, pose_msgs.orientation.y, pose_msgs.orientation.z, pose_msgs.orientation.w])
        T = self.transformer.fromTranslationRotation(pos,quat)
        return T
    # 4x4 numpy matrix
    def get_frame_transform(self, frame_name, header_frame_name = '/base'):
        pos, quat = self.listener.lookupTransform(header_frame_name, frame_name, rospy.Time())
        T = self.transformer.fromTranslationRotation(pos, quat)
        return T

    def display_path(self, dt = 0.01):
        display(self.planner,dt)

if __name__ == "__main__":
    # TODO: Change the test codes using SuhanMotionPlannerManager
    rospy.init_node('suhan_motion_planner', anonymous=True)
    smpm = SuhanMotionPlannerManager(sys.argv)

    # rospy.spin()
    while smpm.joint_subscriber.get_num_connections() == 0:
        smpm.planner.publish_planning_scene_msg()
        rospy.sleep(0.5)
        
    smpm.planner.set_start_arm_states(smpm.joint_states)
    smpm.planner.update_arm_states(smpm.joint_states)
    smpm.planner.publish_planning_scene_msg()

    traj = smpm.plan_object_ik('panda_right','long_part', 0.32)
    if traj == None:
        print('plan failed')
        exit()

    client = actionlib.SimpleActionClient('/assembly_dual_controller/joint_trajectory_control', FollowJointTrajectoryAction)
    client.wait_for_server()
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = traj
    
    client.send_goal(goal)
    client.wait_for_result()

    print(client.get_result())
