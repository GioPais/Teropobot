#!/usr/bin/env python

__author__ = "Giovanni Pais"

import roslib
roslib.load_manifest('my_dynamixel_tutorial')

import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg
import control_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal



class Joint:
        def __init__(self, motor_name):
            #arm_name should be b_arm or f_arm
            self.name = motor_name
            self.jta = actionlib.SimpleActionClient('/'+self.name+'_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            rospy.loginfo('Waiting for joint trajectory action')
            self.jta.wait_for_server()
            rospy.loginfo('Found joint trajectory action!')


        def move_joint(self, angles):
            goal = FollowJointTrajectoryGoal()
            char = self.name[0] #either 'f' or 'b'
            goal.trajectory.joint_names = [char+'_thigh_pitch_joint', char+'_tibia_joint',char+'_ankle_joint']#['joint_1'+char, 'joint_2'+char,'joint_3'+char]
            point = JointTrajectoryPoint()
            point.positions = angles
            point.time_from_start = rospy.Duration(3)
            goal.trajectory.points.append(point)
            self.jta.send_goal_and_wait(goal)


def main():
            l_leg = Joint('l_leg')
            r_leg = Joint('r_leg')
            rospy.loginfo('START')
            l_leg.move_joint([0.0,0.0,1.0])
            r_leg.move_joint([0.0,0.0,-1.0])
            l_leg.move_joint([0.0,0.6,1.0])
            #Moving left leg to: [ 0.0409544041617 ] [ 1.27770916364 ] [ -1.71760984643 ]
            #Moving right leg to: [ -0.463325480497 ] [ 0.951588450178 ] [ -1.21178042316 ]
            l_leg.move_joint([-0.0409544041617, 1.27770916364, 1.71760984643])
            r_leg.move_joint([-0.463325480497, -0.951588450178, -1.21178042316])
            rospy.loginfo('FINISHED')

if __name__ == '__main__':
      rospy.init_node('joint_position_tester')
      main()