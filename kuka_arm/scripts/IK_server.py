#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics Software Engineering Nanodegree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# Student contributor: Joel Tiura

# import modules
import rospy
from tf.transformations import euler_from_quaternion

from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose

import time
# import os

# TODO: matrix constructor functions and pickled matrices (perhaps in another module?):
# re-implement sympy matrix derivations from jupyter notebooks as automatic constructor functions;
# include functionality to pickle these so that they can be loaded faster and into other code.
# enhances reproducibility and transparency of algorithm.

# from sympy import symbols, sin, cos, sqrt, simplify, atan2, N, acos, pi
# from sympy.matrices import Matrix, eye

import numpy as np

# There's a lot of dense trig expressions ahead so I'll also directly import these for readability
# Since sympy versions of sqrt, sin and cos are not used in the live code there isn't any namespace ambiguity
from numpy import sin, cos, sqrt


def handle_calculate_IK(req):
    '''
    :param req: target end-effector position/orientation specified by the rospy geometry_msgs.msg/Pose message type
    :return: CalculateIKResponse(joint_trajectory_list) [list of list of floats representing joint angles]
    '''

    # record start time
    t_start = time.time()
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Rather than construct an error correction transform for the coordinate
        # system mismatch by evaluating substitutions numerically, which leads
        # to small but avoidable error, I opted to solve it symbolically and
        # bake it into the total end effector transform matrix.

        def construct_R_EE(orientation):
            '''
            :param orientation: tuple containing roll, pitch, and yaw values extracted with the tf.
            :return:
            '''
            # R_EE is a precise symbolically-derived ('pre-fab') end effector orientation matrix.
            # Includes a correction for coordinate system mismatch included by multiplying in the matrix:
            #
            # R_corr = Matrix([[ 0,  0,  1],
            #                  [ 0, -1,  0],
            #                  [ 1,  0,  0]])

            # from the pose orientation quaternion with tf.e

            # roll, pitch, and yaw are intended to be the values extracted

            r, p, y = orientation

            cos_p = cos(p)
            sin_p = sin(p)
            cos_y = cos(y)
            sin_y = sin(y)
            cos_r = cos(r)
            sin_r = sin(r)

            # substitute:
            R_EE = np.array([
                [sin_p * cos_r * cos_y + sin_r * sin_y, -sin_p * sin_r * cos_y + sin_y * cos_r, cos_p * cos_y],
                [sin_p * sin_y * cos_r - sin_r * cos_y, -sin_p * sin_r * sin_y - cos_r * cos_y, sin_y * cos_p],
                [cos_p * cos_r, -sin_r * cos_p, -sin_p]],
                dtype=np.float64)
            return R_EE

        # Define Error Analysis Function
        def evaluate_EE_error(theta_list, pose):
            '''
            :param theta_list: calculated values for joints 1:6 in ascending order
            :param pose: pose object contained in req.poses for which the
            :return:
            '''
            q1n, q2n, q3n, q4n, q5n, q6n = theta_list

            # fk_position is the translation vector from the total homogeneous transform
            # between the base and the end effector; this representation has frame error
            # correction baked in!
            fk_position = np.array([
                [-0.303 * (sin(q1n) * sin(q4n) + sin(q2n + q3n) * cos(q1n) * cos(q4n)) * sin(q5n)
                 + (1.25 * sin(q2n) - 0.054 * sin(q2n + q3n) + 1.5 * cos(q2n + q3n) + 0.35) * cos(q1n)
                 + 0.303 * cos(q1n) * cos(q5n) * cos(q2n + q3n)],
                [-0.303 * (sin(q1n) * sin(q2n + q3n) * cos(q4n) - sin(q4n) * cos(q1n)) * sin(q5n)
                 + (1.25 * sin(q2n) - 0.054 * sin(q2n + q3n) + 1.5 * cos(q2n + q3n) + 0.35) * sin(q1n)
                 + 0.303 * sin(q1n) * cos(q5n) * cos(q2n + q3n)],
                [-0.303 * sin(q5n) * cos(q4n) * cos(q2n + q3n) - 0.303 * sin(q2n + q3n) * cos(q5n)
                 - 1.5 * sin(q2n + q3n) + 1.25 * cos(q2n) - 0.054 * cos(q2n + q3n) + 0.75]], dtype=np.float64)

            pose_target = np.array([[pose.position.x], [pose.position.y], [pose.position.z]])
            error_vect = pose_target - fk_position
            xErr, yErr, zErr = error_vect[0][0], error_vect[1][0], error_vect[2][0]
            absError = sqrt(error_vect[0][0] ** 2 + error_vect[1][0] ** 2 + error_vect[2][0] ** 2)
            return xErr, yErr, zErr, absError

        def construct_R0_3_inverse(q1, q2, q3):
            R0_3_inverse = np.array([
                [sin(q2 + q3) * cos(q1), sin(q1) * sin(q2 + q3), cos(q2 + q3)],
                [cos(q1) * cos(q2 + q3), sin(q1) * cos(q2 + q3), -sin(q2 + q3)],
                [-sin(q1), cos(q1), 0]],
                dtype=np.float64)
            return R0_3_inverse

        # Initialize service response
        joint_trajectory_list = []
        # Initialize end effector position error list if logging is logging/plotting active

        err_dict = {'x': [], 'y': [], 'z': [], 'abs': []}

        # Loop through poses in requested sequence, and do IK calculation to determine the joint values for each:
        for x in xrange(0, len(req.poses)):
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # (px ,py, pz) is the end-effector position in the base reference frame
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            # This accomplishes the same essential goal as req.poses[x].orientation.?attributes
            quaternion_orientation = [
                req.poses[x].orientation.x,
                req.poses[x].orientation.y,
                req.poses[x].orientation.z,
                req.poses[x].orientation.w,
            ]

            # Extract roll, pitch, and yaw from the orientation quaternion using method from tf.transformations
            # (roll, pitch, yaw) = euler_from_quaternion(quaternion_orientation)
            r_EE = construct_R_EE(euler_from_quaternion(quaternion_orientation))

            # r_EE is the rotational component of the transformation matrix between the base coord
            # roll, pitch, and yaw specify end-effector orientation relative to the base coordinate system,
            # but only when the corresponding sequence of rotations is applied with respect to static coordinates, and
            # in an XYZ axis order.
            #
            # This sequence and convention is not a coincidence; it's the is the default behavior of the
            # tf.transformations.euler_from_quaternion() function.

            # (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            #     [req.poses[x].orientation.x,
            #      req.poses[x].orientation.y,
            #      req.poses[x].orientation.z,
            #      req.poses[x].orientation.w])

            # while the wrist center's location is not specified explicitly, it is already determined
            # *implicitly* by the specified end effector orientation.

            # (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            #     [req.poses[x].orientation.x, req.poses[x].orientation.y,
            #      req.poses[x].orientation.z, req.poses[x].orientation.w])
            # these values of roll, pitch, and yaw are calculated using the 'sxyz' convention in tf,
            # s denotes static frame, xyz gives an axis sequence for the rotations

            # r_EE is the general symbolic rotation matrix for the end effector orientation, defined outside loop.
            # r_EE represents this matrix for a given pose, substituting in the roll, pitch, and yaw variables
            # extracted from the pose orientation quaternion.

            # Calculate wrist center location
            EE_vector = 0.303 * r_EE[:, 2]

            wrist_x = px - EE_vector[0]
            wrist_y = py - EE_vector[1]
            wrist_z = pz - EE_vector[2]

            theta1 = np.arctan2(wrist_y, wrist_x)

            wrist_s = sqrt(wrist_x ** 2 + wrist_y ** 2)

            # delta_s_behind = -1.0 * (wrist_s + 0.35)
            delta_s = wrist_s - 0.35
            delta_z = wrist_z - 0.75

            # j2_pitch_behind = N(atan2(delta_z, delta_s_behind))
            j2_pitch = np.arctan2(delta_z, delta_s)

            # Triangle Solver
            # A, B, a, b, c = symbols('A B a b c') # A, B are needed interior angles; a, b, c are side lengths

            # j2_wrist_dist is the only side of said triangle not actually measured along links of the manipulator,
            # formed by the line between the midpoint/origin of joint_2 to wrist center
            j2_wrist_dist = sqrt(
                delta_s ** 2 + delta_z ** 2)  # distance from joint_2 to wrist center - side b length.

            # Triangle with side lengths:
            #     a: 1.500971685275908, the distance betweenj2_wrist_dist defined outside loop = 1.500971685275908
            #     b: j2_wrist_dist,
            #     c: 1.25
            #     }
            A = np.arccos((j2_wrist_dist ** 2 + 1.25 ** 2 - 1.501 ** 2) / (2 * j2_wrist_dist * 1.25))
            B = np.arccos((1.501 ** 2 + 1.25 ** 2 - j2_wrist_dist ** 2) / (2 * 1.501 * 1.25))

            theta2 = (np.pi / 2 - j2_pitch - A)
            theta3 = np.pi / 2 - (B + 0.036)  # j4_correction defined outside loop = 0.036

            # R0_3_inverse is constructed by passing the first three joint angles to the  substituting the first three
            # joint angles into a template, R0_3_inverse_template
            # invR0_3 is always equivalent to the same expression, making the template matrix reusable for each pose
            # up to the following substitution, and thus is defined outside the loop

            # R0_3_inverse is the (reciprocal) inverse of the rotational transform R0_3, matrix
            R0_3_inverse = construct_R0_3_inverse(theta1, theta2, theta3)

            # The very first step is to arbitrarily constrain the wrist center's motion
            # By matrix multiplying R0_3_inverse onto r_EE, the original orientation vector, we are left with
            R3_6 = R0_3_inverse.dot(r_EE)

            # theta4, theta5, and theta6 are all determined at this point, since we now have the inverse transform may
            # be isolated from solved for symbolically
            theta5 = np.math.atan2(sqrt(R3_6[0, 2] * R3_6[0, 2] + R3_6[2, 2] * R3_6[2, 2]), R3_6[1, 2])
            theta4 = np.math.atan2(R3_6[2, 2], -R3_6[0, 2])
            theta6 = np.math.atan2(-R3_6[1, 1], R3_6[1, 0])

            x_err, y_err, z_err, abs_err = evaluate_EE_error([theta1, theta2, theta3, theta4, theta5, theta6],
                                                             req.poses[x])

            err_dict['x'].append(x_err)
            err_dict['y'].append(y_err)
            err_dict['z'].append(z_err)
            err_dict['abs'].append(abs_err)

            # Populate response for the IK request
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)


        x_err_max = max(err_dict['x'])
        x_err_mean = np.mean(err_dict['x'])

        y_err_max = max(err_dict['y'])
        y_err_mean = np.mean(err_dict['y'])

        z_err_max = max(err_dict['z'])
        z_err_mean = np.mean(err_dict['z'])

        abs_err_max = max(err_dict['abs'])
        abs_err_mean = np.mean(err_dict['abs'])

        rospy.loginfo('Error Maxima\n x: {}\n y: {}\n z: {}\n abs: {}\n'.format(x_err_max, y_err_max, z_err_max, abs_err_max))
        rospy.loginfo('Error Means\n x: {}\n y: {}\n z: {}\n abs: {}\n'.format(x_err_mean, y_err_mean, z_err_mean, abs_err_mean))


        rospy.loginfo("Final length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        rospy.loginfo("Full compute time (with overhead): %s" % (time.time() - t_start))
        rospy.loginfo("Mean compute time per pose: {}\n".format((time.time() - t_start) / len(joint_trajectory_list)))

        if __name__ != "__main__":
            return joint_trajectory_list

        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request\n"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
