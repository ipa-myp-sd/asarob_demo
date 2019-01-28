#!/usr/bin/python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal

from trajectory_msgs.msg import JointTrajectoryPoint
import control_msgs.msg
from geometry_msgs.msg import *
import roslib
import json
import argparse
import numpy
import math
from sensor_msgs.msg import JointState
import threading
import thread
import sys
import time
import os
import rospkg
import datetime

from geometry_msgs.msg import *
from trajectory_msgs.msg import *
from control_msgs.msg import *

class Trajectory:
    def __init__(self, arm_name):
        """
        Initializing the Parameters and the Client definition to send to the controller
        """
        self.goal = FollowJointTrajectoryGoal()
        self.client = actionlib.SimpleActionClient(arm_name+'/joint_trajectory_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)
        self.client.wait_for_server(rospy.Duration(5))

        # Trajectory points
        point0 = [1.080563267708147, 1.217155545308433, -1.3958713748888183, -1.6480694373057911, -0.8877923106149632, -0.35130350564666646, 1.068305519131009]
        point1 = [1.1042523228704306, 1.1305731443564335, -1.4643351484527116, -1.6461103026252353, -0.856494667666925, -0.283204827987106, 1.068305519131009]
        point2 = [1.4293331317214442, 0.9202928011615645, -1.7371342144837492, -1.6066390315926524, -0.7578441564939773, -0.2915972507295326, 1.068305519131009]
        point3 = [1.6955817735558911, 0.7480505182423105, -1.9605708619286162, -1.5743321318898014, -0.6770427714243787, -0.2984573917592144, 1.068305519131009]
        point4 = [2.0365471687616896, 0.5274752981259336, -2.246708212935909, -1.5329572790293566, -0.5735659588841662, -0.30724068380699165, 1.068305519131009]
        point5 = [2.8349540630098407, 0.010985892324975488, -2.9167374669334363, -1.4360783398255421, -0.33125283046196596, -0.3278046281006306, 1.068305519131009]
        point6 = [2.2671826983406342, -0.36800267278300436, -2.29318810419535, -1.2845797827603465, -0.09478883167581204, -0.3888069974667768, 1.0699740979351238] # used for pointing inthe torso motion
        self.points = [point0, point1, point2, point3, point4, point5, point6]
        #self.point = [point0]
        self.velocities = [1.0,1.0,1.0,1.0,1.0,1.0,1.0] #assume that all vel are same
        self.accelerations = [0.01,0.01,0.01,0.01,0.01,0.01,0.01] #assume that all acce are same
        self.ns_global_prefix = "/script_server"

    ## Parse and compose trajectory message
    def compose_trajectory(self, component_name, parameter_name):
        # get joint_names from parameter server
        param_string = self.ns_global_prefix + "/" + component_name + "/joint_names"
        if not rospy.has_param(param_string):
                rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",param_string)
                return (JointTrajectory(), 2)
        joint_names = rospy.get_param(param_string)

        # check joint_names parameter
        if not type(joint_names) is list: # check list
                rospy.logerr("no valid joint_names for %s: not a list, aborting...",component_name)
                print "joint_names are:",joint_names
                return (JointTrajectory(), 3)
        else:
            for i in joint_names:
                    #print i,"type1 = ", type(i)
                    if not type(i) is str: # check string
                            rospy.logerr("no valid joint_names for %s: not a list of strings, aborting...",component_name)
                            print "joint_names are:",param
                            return (JointTrajectory(), 3)
                    else:
                            rospy.logdebug("accepted joint_names for component %s",component_name)
        '''
        # get joint values from parameter server
        if type(parameter_name) is str:
                full_parameter_name = self.ns_global_prefix + "/" + component_name + "/" + parameter_name # parameter_name: joint_state
                if not rospy.has_param(full_parameter_name):
                        rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",full_parameter_name)
                        return (JointTrajectory(), 2)
                param = rospy.get_param(full_parameter_name)
        else:
                param = parameter_name

        # check trajectory parameters
        if not type(param) is list: # check outer list
                        rospy.logerr("no valid parameter for %s: not a list, aborting...",component_name)
                        print "parameter is:",param
                        return (JointTrajectory(), 3)
        '''
        param = self.points

        traj = []
        for point in param:
                #print point,"type1 = ", type(point)
                if type(point) is str:
                        full_parameter_name = self.ns_global_prefix + "/" + component_name + "/" + point
                        if not rospy.has_param(full_parameter_name):
                                rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",full_parameter_name)
                                return (JointTrajectory(), 2)
                        point = rospy.get_param(full_parameter_name)
                        point = point[0] # \todo TODO: hack because only first point is used, no support for trajectories inside trajectories
                        #print point
                elif type(point) is list:
                        rospy.logdebug("point is a list")
                else:
                        rospy.logerr("no valid parameter for %s: not a list of lists or strings, aborting...",component_name)
                        print "parameter is:",param
                        return (JointTrajectory(), 3)

                # here: point should be list of floats/ints
                #print point
                if not len(point) == len(joint_names): # check dimension
                        rospy.logerr("no valid parameter for %s: dimension should be %d and is %d, aborting...",component_name,len(joint_names),len(point))
                        print "parameter is:",param
                        return (JointTrajectory(), 3)

                for value in point:
                        #print value,"type2 = ", type(value)
                        if not ((type(value) is float) or (type(value) is int)): # check type
                                #print type(value)
                                rospy.logerr("no valid parameter for %s: not a list of float or int, aborting...",component_name)
                                print "parameter is:",param
                                return (JointTrajectory(), 3)

                        rospy.logdebug("accepted value %f for %s",value,component_name)
                traj.append(point)

        rospy.logdebug("accepted trajectory for %s",component_name)

        # get current pos
        timeout = 3.0
        try:
                start_pos = rospy.wait_for_message("/" + component_name + "/joint_states", JointState, timeout = timeout).position
        except rospy.ROSException as e:
                rospy.logwarn("no joint states received from %s within timeout of %ssec. using default point time of 8sec.", component_name, str(timeout))
                start_pos = []

        # convert to ROS trajectory message
        traj_msg = JointTrajectory()
        # if no timestamp is set in header, this means that the trajectory starts "now"
        traj_msg.joint_names = joint_names
        point_nr = 0
        traj_time = 0

        param_string = self.ns_global_prefix + "/" + component_name + "/default_vel"
        if not rospy.has_param(param_string):
                default_vel = 0.1 # rad/s
                rospy.logwarn("parameter %s does not exist on ROS Parameter Server, using default of %f [rad/sec].",param_string,default_vel)
        else:
                default_vel = rospy.get_param(param_string)

        param_string = self.ns_global_prefix + "/" + component_name + "/default_acc"
        if not rospy.has_param(param_string):
                default_acc = 1.0 # rad^2/s
                rospy.logwarn("parameter %s does not exist on ROS Parameter Server, using default of %f [rad^2/sec].",param_string,default_acc)
        else:
                default_acc = rospy.get_param(param_string)

        for point in traj:
                point_nr = point_nr + 1
                point_msg = JointTrajectoryPoint()
                point_msg.positions = point

                # set zero velocities for last trajectory point only
                #if point_nr == len(traj):
                #	point_msg.velocities = [0]*len(joint_names)

                # set zero velocity and accelerations for all trajectory points
                point_msg.velocities = [0]*len(joint_names)
                point_msg.accelerations = [0]*len(joint_names)

                # use hardcoded point_time if no start_pos available
                if start_pos != []:
                        print "start pose is not defined"
                        point_time = self.calculate_point_time(component_name, start_pos, point, default_vel, default_acc)
                else:
                        print "********Hello*******"
                        point_time = 8*point_nr

                start_pos = point
                point_msg.time_from_start=rospy.Duration(point_time + traj_time)
                traj_time += point_time
                traj_msg.points.append(point_msg)
        return (traj_msg, 0)

    def calculate_point_time(self, component_name, start_pos, end_pos, default_vel, default_acc):
        try:
                d_max = max(list(abs(numpy.array(start_pos) - numpy.array(end_pos))))
                t1 = default_vel / default_acc
                s1 = default_acc / 2 * t1**2
                if (2 * s1 < d_max):
                        # with constant velocity phase (acc, const vel, dec)
                        # 1st phase: accelerate from v=0 to v=default_vel with a=default_acc in t=t1
                        # 2nd phase: constante velocity with v=default_vel and t=t2
                        # 3rd phase: decceleration (analog to 1st phase)
                        s2 = d_max - 2 * s1
                        t2 = s2 / default_vel
                        t = 2 * t1 + t2
                else:
                        # without constant velocity phase (only acc and dec)
                        # 1st phase: accelerate from v=0 to v=default_vel with a=default_acc in t=t1
                        # 2nd phase: missing because distance is to short (we already reached the distance with the acc and dec phase)
                        # 3rd phase: decceleration (analog to 1st phase)
                        t = math.sqrt(d_max / default_acc)
                point_time = max(t, 0.4)	# use minimal point_time
        except ValueError as e:
                print "Value Error", e
                print "Likely due to mimic joints. Using default point_time: 3.0 [sec]"
                point_time = 3.0	# use default point_time
        return point_time


    def read_data(self):
        joint_names = []; position_values = []; time = []; velocity = []; acceleration = []
        for item in data[0]['points_'][0]['state_']['joints_']:
            joint_names.append(str(item['name_']))
        #print(joint_names)

        for i in range(len(data[0]['points_'])):
            time_i = data[0]['points_'][i]['time_']
            time.append(time_i)
            temp_value = []
            for item1 in data[0]['points_'][i]['state_']['joints_']:
                temp_value.append(item1['value_'])
            position_values.append(temp_value)

            temp_vel = []
            temp_acc = []
            for ii in data[0]['points_'][i]['state_']['joints_']:
                temp_vel.append(ii['velocity_'])
                temp_acc.append(ii['acceleration_'])
            velocity.append(temp_vel)
            acceleration.append(temp_acc)

        return [joint_names, position_values, time, velocity, acceleration]

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('input_arm_name', default='arm_right', help='enter the arm name')
    parser.add_argument('input_gripper_name', default='gripper_right', help='enter the gripper name')
    args, unknown = parser.parse_known_args()
    args = parser.parse_args()

    rospy.loginfo("Initializing the Node")
    rospy.init_node("follow_joint_trajectory_goal")
    trajectory = Trajectory(args.input_arm_name)

    (traj_msg, error_code) = trajectory.compose_trajectory(component_name="arm_right", parameter_name="joint_states")
    trajectory.goal.trajectory = traj_msg
    trajectory.client.send_goal(trajectory.goal)
    trajectory.client.wait_for_result()

