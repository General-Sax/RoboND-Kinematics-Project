#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from sympy import *


def handle_calculate_IK(req, debug_return=False):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        # Rather than construct an error correction transform for the coordinate
        # system mismatch by evaluating substitutions numerically, which leads
        # to small but avoidable error, I opted to solve it symbolically and
        # bake it into the total end effector transform matrix.

        r, p, y = symbols('r p y') # r, p, y later substituted for EE roll, pitch, and yaw respectively
        # R_EE is a 'prefabricated' end effector orientation matrix w/correction for coordinate system mismatch/
        # Corrected using precisely-derived matrix R_corr, below:
        R_EE = Matrix([
            [sin(p)*cos(r)*cos(y) + sin(r)*sin(y), -sin(p)*sin(r)*cos(y) + sin(y)*cos(r), cos(p)*cos(y)],
            [sin(p)*sin(y)*cos(r) - sin(r)*cos(y), -sin(p)*sin(r)*sin(y) - cos(r)*cos(y), sin(y)*cos(p)],
            [                       cos(p)*cos(r),                        -sin(r)*cos(p),       -sin(p)]])
        # R_corr = Matrix([
        #     [ 0,  0,  1],
        #     [ 0, -1,  0],
        #     [ 1,  0,  0]])

        j3_radius = 1.501 # = N(sqrt(dh[a2]**2 + dh[d4]**2)), (rounded)
        j4_correction = 0.036 # = N(atan2(0.054, 1.50)), (rounded)

        ### Your FK code here
        ## Create symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')  # theta angles
        # d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        # a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        # alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        #
        # Enter table of values:
    	#
    	## Create Modified DH parameter table:
        # dh = {alpha0: 0,        a0: 0,      d1: 0.75,   q1: q1,
        #       alpha1: -pi / 2,  a1: 0.35,   d2: 0,      q2: q2 - pi / 2.,
        #       alpha2: 0,        a2: 1.25,   d3: 0,      q3: q3,
        #       alpha3: -pi / 2,  a3: -0.054, d4: 1.50,   q4: q4,
        #       alpha4: pi / 2,   a4: 0,      d5: 0,      q5: q5,
        #       alpha5: -pi / 2,  a5: 0,      d6: 0,      q6: q6,
        #       alpha6: 0,        a6: 0,      d7: 0.303,  q7: 0
        #       }
    	## Define Modified DH Transformation matrix
    	#
    	#
    	## Create individual transformation matrices
    	#
    	#
    	## Extract rotation matrices from the transformation matrices
    	#
    	#
        ###

        invR0_3 = Matrix([
            [sin(q2 + q3)*cos(q1), sin(q1)*sin(q2 + q3),  cos(q2 + q3)],
            [cos(q1)*cos(q2 + q3), sin(q1)*cos(q2 + q3), -sin(q2 + q3)],
            [            -sin(q1),              cos(q1),             0]])

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
	        # px,py,pz = end-effector position
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            # roll, pitch, yaw = end-effector orientation
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                 req.poses[x].orientation.z, req.poses[x].orientation.w])
            # these values of roll, pitch, and yaw are calculated using the 'sxyz' convention in tf,
            # s denotes static frame, xyz gives an axis sequence for the rotations

            # R_EE is the general symbolic rotation matrix for the end effector orientation, defined outside loop.
            # Rrpy represents this matrix for a given pose, substituting in the roll, pitch, and yaw variables
            # extracted from the pose orientation quaternion.
            Rrpy = R_EE.subs({r: roll, p: pitch, y: yaw})

            # Calculate wrist center location
            EE_vector = 0.303 * Rrpy[0:3, 2]

            wrist_x = px - EE_vector[0]
            wrist_y = py - EE_vector[1]
            wrist_z = pz - EE_vector[2]

            theta1 = N(atan2(wrist_y, wrist_x))

            wrist_s = N(sqrt(wrist_x**2 + wrist_y**2))

            # delta_s_behind = -1.0 * (wrist_s + 0.35)
            delta_s = wrist_s - 0.35
            delta_z = wrist_z - 0.75

            # j2_pitch_behind = N(atan2(delta_z, delta_s_behind))
            j2_pitch = N(atan2(delta_z, delta_s))

            # Triangle Solver
            A, B, a, b, c = symbols('A B a b c') # A, B are needed interior angles; a, b, c are side lengths
            j2_wrist_dist = N(sqrt(delta_s**2 + delta_z**2)) # distance from joint_2 to wrist center - side b length
            sides = {
                a: j3_radius,       # j3_radius defined outside loop = 1.501
                b: j2_wrist_dist,
                c: 1.25
                }
            A = acos((b**2 + c**2 - a**2)/(2 * b * c)).evalf(subs=sides)
            B = acos((a**2 + c**2 - b**2)/(2 * a * c)).evalf(subs=sides)

            theta2 = N(pi/2 - j2_pitch - A)
            theta3 = N(pi/2 - (B + j4_correction)) # j4_correction defined outside loop = 0.036

            # R0_3_inverse is calculated by substituting the first three joint angles into a template, invR0_3;
            # invR0_3 is invariant up to the following substitution, and thus is defined outside the loop
            R0_3_inverse = invR0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

            R3_6 = R0_3_inverse * Rrpy

            theta5 = N(atan2(sqrt(R3_6[0, 2]*R3_6[0, 2] + R3_6[2, 2]*R3_6[2, 2]), R3_6[1, 2]))

            theta4 = N(atan2(R3_6[2, 2], -R3_6[0, 2]))

            theta6 = N(atan2(-R3_6[1, 1], R3_6[1, 0]))

            # if theta4 > 0 and theta6 > 0:
            #     theta6 -= 2.0*pi
            # elif theta4 < 0 and theta6 < 0:
            #     theta6 += 2.0*pi

            # Comparison to the previous pose is not available for the first pose in a sequence - at least without
            # reference to additional pose information from ros. Comparison smoothing begins at x=1.

            # if py < 0.15:
            #     while theta4 > N(2*pi):
            #         theta4 -= N(pi)
            #         theta5 *= -1.0
            #     while theta4 < 0:
            #         theta4 += N(pi)
            #         theta5 *= -1.0
            # elif py > 0.15:
            #     while theta4 < -N(2*pi):
            #         theta4 += N(pi)
            #         theta5 *= -1.0
            #     while theta4 > 0:
            #         theta4 -= N(pi)
            #         theta5 *= -1.0



            if x != 0 and x != len(req.poses)-1:
                while abs(theta4 - joint_trajectory_list[x-1].positions[3]) > N(pi):
                    if joint_trajectory_list[x-1].positions[3] > theta4:
                        theta4 += N(pi)
                        theta5 *= -1.0
                    elif joint_trajectory_list[x-1].positions[3] < theta4:
                        theta4 -= N(pi)
                        theta5 *= -1.0

                while abs(theta6 - joint_trajectory_list[x-1].positions[5]) > N(pi):
                    if joint_trajectory_list[x-1].positions[5] > theta6:
                        theta6 += N(pi)
                    elif joint_trajectory_list[x-1].positions[5] < theta6:
                        theta6 -= N(pi)


            ###########

            # Populate response for the IK request
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))

        if debug_return:
            return joint_trajectory_list

        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
