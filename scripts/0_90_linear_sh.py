#!/usr/bin/env python

from __future__ import print_function
from suhan_motion_planner import SuhanMotionPlannerManager
import sys
import copy
import rospy
import moveit_commander
import actionlib

import std_msgs.msg
import geometry_msgs.msg
import moveit_msgs.msg
import tf_conversions
import math
from math import pi
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_matrix
import numpy as np
# from MoveGroupPlanner import *

import yaml
import tf2_ros
import rospkg
import tf


# class ContinuousGraspCandid():
#     def __init__(self, package='regrasp_constraint_planner', path='yaml/bottom/link1.yaml', dir='minusY'):
#         rospack = rospkg.RosPack()
#         self.package_path = rospack.get_path(package)
#         self.file_path = self.package_path + '/' + path
#         self.dir = dir
#         with open(self.file_path, 'r') as stream:
#             self.yaml = yaml.safe_load(stream)

#     def get_grasp(self, index, ratio):
#         lb = np.array(self.yaml[self.dir][index]['lower_bound'])
#         ub = np.array(self.yaml[self.dir][index]['upper_bound'])
#         ori = np.array(self.yaml[self.dir][index]['orientation'])
#         # print((ub-lb) * ratio + lb)
#         # print(ori)
#         return ((ub-lb) * ratio + lb , ori)

#     def get_grasp_pose_msg(self, index, ratio):
#         g = self.get_grasp(index, ratio)
#         pose_msg = geometry_msgs.msg.PoseStamped()
#         pose_msg.header.frame_id = "assembly_frame"
#         pose_msg.header.stamp = rospy.Time(0)
#         pose_msg.pose.position.x = g[0][0]
#         pose_msg.pose.position.y = g[0][1]
#         pose_msg.pose.position.z = g[0][2]
#         pose_msg.pose.orientation.x = g[1][0]
#         pose_msg.pose.orientation.y = g[1][1]
#         pose_msg.pose.orientation.z = g[1][2]
#         pose_msg.pose.orientation.w = g[1][3]
#         return pose_msg


class SceneObject():
    def __init__(self):
        self.stefan_dir = 'package://placement_grp_test//STEFAN/stl/assembly.stl'
        # self.stefan_dir = rospack.get_path(
        #     "grasping_point") + "/STEFAN/stl/assembly.stl"
        # self.stefan_dir = "package://STEFAN/stl/"
        self.assembly = "assembly"
        self.assembly_pose = geometry_msgs.msg.PoseStamped()
        self.assembly_pose.header.frame_id = "base"

        case = 7

        if case == 1:
            self.assembly_pose.pose.position.x = 1.0
            self.assembly_pose.pose.position.y = 0.2
            self.assembly_pose.pose.position.z = 1.58
            self.assembly_pose.pose.orientation.x = -0.6628315
            self.assembly_pose.pose.orientation.y = -0.288225
            self.assembly_pose.pose.orientation.z = 0.3090834
            self.assembly_pose.pose.orientation.w = 0.6181004


#         elif case == 2: # XYZ Euler_angle x:0 y:0 z:45
#             self.assembly_pose.pose.position.x = 1.0
#             self.assembly_pose.pose.position.y = 0.145
#             self.assembly_pose.pose.position.z = 1.2
#             q = (
# [ 0, 0, 0.3826834, 0.9238795 ]
#             )
#             self.assembly_pose.pose.orientation.x = q[0]
#             self.assembly_pose.pose.orientation.y = q[1]
#             self.assembly_pose.pose.orientation.z = q[2]
#             self.assembly_pose.pose.orientation.w = q[3]


        elif case == 3: # XYZ Euler_angle x:-90 y:0 z:0
            self.assembly_pose.pose.position.x = 0.95
            self.assembly_pose.pose.position.y = -0.05
            self.assembly_pose.pose.position.z = 1.58
            q = (
[ -0.7071068, 0, 0, 0.7071068 ]
            )
            self.assembly_pose.pose.orientation.x = q[0]
            self.assembly_pose.pose.orientation.y = q[1]
            self.assembly_pose.pose.orientation.z = q[2]
            self.assembly_pose.pose.orientation.w = q[3]


        elif case == 4: # XYZ Euler_angle x:-90 y:-20 z:0
            self.assembly_pose.pose.position.x = 1.0
            self.assembly_pose.pose.position.y = 0.05
            self.assembly_pose.pose.position.z = 1.58
            q = (
[ -0.6963642, -0.1227878, 0.1227878, 0.6963642 ]
            )
            self.assembly_pose.pose.orientation.x = q[0]
            self.assembly_pose.pose.orientation.y = q[1]
            self.assembly_pose.pose.orientation.z = q[2]
            self.assembly_pose.pose.orientation.w = q[3]


        elif case == 5: # XYZ Euler_angle x:-90 y:-40 z:0
            self.assembly_pose.pose.position.x = 0.975
            self.assembly_pose.pose.position.y = 0.125
            self.assembly_pose.pose.position.z = 1.58
            q = (
[ -0.664463, -0.2418448, 0.2418448, 0.664463 ]
            )
            self.assembly_pose.pose.orientation.x = q[0]
            self.assembly_pose.pose.orientation.y = q[1]
            self.assembly_pose.pose.orientation.z = q[2]
            self.assembly_pose.pose.orientation.w = q[3]


        elif case == 6: # XYZ Euler_angle x:-90 y:-60 z:0
            self.assembly_pose.pose.position.x = 1.0
            self.assembly_pose.pose.position.y = 0.2
            self.assembly_pose.pose.position.z = 1.58
            q = (
[ -0.6123724, -0.3535534, 0.3535534, 0.6123724 ]
            )
            self.assembly_pose.pose.orientation.x = q[0]
            self.assembly_pose.pose.orientation.y = q[1]
            self.assembly_pose.pose.orientation.z = q[2]
            self.assembly_pose.pose.orientation.w = q[3]


        elif case == 7: # XYZ Euler_angle x:-90 y:-80 z:0
            self.assembly_pose.pose.position.x = 0.95
            self.assembly_pose.pose.position.y = 0.35
            self.assembly_pose.pose.position.z = 1.58
            q = (
[ -0.5416752, -0.4545195, 0.4545195, 0.5416752 ]
            )
            self.assembly_pose.pose.orientation.x = q[0]
            self.assembly_pose.pose.orientation.y = q[1]
            self.assembly_pose.pose.orientation.z = q[2]
            self.assembly_pose.pose.orientation.w = q[3]


if __name__ == '__main__':

    sys.argv.append('joint_states:=/panda_dual/joint_states')
    rospy.init_node('assembly_task_manager', argv=sys.argv)
    topic_name = '/assembly/state_transition'
    mdp = SuhanMotionPlannerManager(sys.argv)
    
    mdp.planner.set_ompl_debug_level(2)
    
    stefan = SceneObject()
    mdp.planner.add_collision_mesh(stefan.stefan_dir, np.array([stefan.assembly_pose.pose.position.x, stefan.assembly_pose.pose.position.y, stefan.assembly_pose.pose.position.z]), np.array(
        [stefan.assembly_pose.pose.orientation.x, stefan.assembly_pose.pose.orientation.y, stefan.assembly_pose.pose.orientation.z, stefan.assembly_pose.pose.orientation.w]), "assembly")

    
    mdp.planner.publish_planning_scene_msg()
    assembly_frame = geometry_msgs.msg.TransformStamped()
    assembly_frame.header.frame_id = "base"
    assembly_frame.header.stamp = rospy.Time.now()
    assembly_frame.child_frame_id = "assembly_frame"
    assembly_frame.transform.translation = stefan.assembly_pose.pose.position
    assembly_frame.transform.rotation = stefan.assembly_pose.pose.orientation

    listener = tf.TransformListener()
    listener.setTransform(assembly_frame)

    mdp.planner.publish_planning_scene_msg()
    rospy.sleep(1)

    # grp_1st = ContinuousGraspCandid('regrasp_constraint_planner', 'yaml/top/link8.yaml', 'minusZ')
    # t1 = listener.transformPose("base", grp_1st.get_grasp_pose_msg(0, 0.4))

    # grp_2nd = ContinuousGraspCandid('regrasp_constraint_planner', 'yaml/bottom/link1.yaml', 'minusY')
    # t2 = listener.transformPose("base", grp_2nd.get_grasp_pose_msg(0, 0.7))

    # grp_3rd = ContinuousGraspCandid('regrasp_constraint_planner', 'yaml/bottom/link5.yaml', 'minusZ')
    # t3 = listener.transformPose("base", grp_3rd.get_grasp_pose_msg(0, 0.3))

    # mdp.planner.publish_planning_scene_msg()
    
    # plan1 = mdp.plan_target_pose("panda_left", np.array([t1.pose.position.x, t1.pose.position.y, t1.pose.position.z]), np.array([t1.pose.orientation.x, t1.pose.orientation.y, t1.pose.orientation.z, t1.pose.orientation.w]))
    # if plan1 == None:
    #     print("1 ik failed")
    # else:
    #     mdp.display_path()
    #     p = mdp.planner.get_solved_path()        
    #     mdp.planner.set_start_arm_states(p[-1])
    #     mdp.planner.update_arm_states(p[-1])
    #     rospy.sleep(2)
    
    
    # plan2 = mdp.plan_target_pose("panda_right", np.array([t2.pose.position.x, t2.pose.position.y, t2.pose.position.z]),np.array([t2.pose.orientation.x, t2.pose.orientation.y, t2.pose.orientation.z, t2.pose.orientation.w]))
    # if plan2 == None:
    #     print("2 ik failed")
    # else:
    #     mdp.display_path()    
    #     p = mdp.planner.get_solved_path()
    #     mdp.planner.set_start_arm_states(p[-1])
    #     mdp.planner.update_arm_states(p[-1])
    #     print(p[-1])
    #     rospy.sleep(2)
    

    # plan3 = mdp.plan_target_pose("panda_top", np.array([t3.pose.position.x, t3.pose.position.y, t3.pose.position.z]), np.array([t3.pose.orientation.x, t3.pose.orientation.y, t3.pose.orientation.z, t3.pose.orientation.w]))
    # if plan3 == None:
    #     print("3 ik failed")
    # else:
    #     mdp.display_path()
    #     rospy.sleep(3)
    #     p = mdp.planner.get_solved_path()
    #     mdp.planner.set_start_arm_states(p[-1])
    #     mdp.planner.update_arm_states(p[-1])
    #     print(p[-1])
        
    mdp.planner.publish_planning_scene_msg()
    mdp.planner.publish_planning_scene_msg()
    