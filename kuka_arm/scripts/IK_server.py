#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics Software Engineering Nanodegree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# Student contributor: Joel Tiura
import time

import rospy
import numpy as np

from tf.transformations import euler_from_quaternion
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from error_handler import ErrorHandler

# There's a lot of dense trig expressions ahead so I'll also directly import these for readability
# SymPy versions of sqrt, sin and cos are *NOT* used in the live code anywhere, so there isn't any namespace ambiguity
sin = np.sin
cos = np.cos
sqrt = np.sqrt
acos = np.arccos
atan2 = np.arctan2


def construct_R_EE(rpy_orientation_tuple):
    '''
    construct_R_EE(orientation: Tuple[float, float, float]) -> np.ndarray:

    :param orientation: tuple containing roll, pitch, and yaw values extracted from the pose orientation quaternion
    :return: R_EE, 3x3 numpy ndarray representing the rotation matrix between the base link and the end effector

    Injects pose-specific roll/pitch/yaw euler angles into a preconfigured matrix representing the rotation transform
    between the base link coordinates and the end effector orientation in those coordinates.
    NOTE: The array construction below encodes the necessary coordinate system correction, included in its derivation.
    '''
    # unpack roll/pitch/yaw euler angles
    r, p, y = rpy_orientation_tuple
    
    # evaluate all trigonometric expressions before creating array
    cos_p = cos(p)
    sin_p = sin(p)
    cos_y = cos(y)
    sin_y = sin(y)
    cos_r = cos(r)
    sin_r = sin(r)
    
    # R_EE is a precise symbolically-derived ('pre-fab') end effector orientation matrix.
    R_EE = np.array([
        [ sin_p*cos_r*cos_y + sin_r*sin_y,  -sin_p*sin_r*cos_y + sin_y*cos_r,  cos_p*cos_y],
        [ sin_p*sin_y*cos_r - sin_r*cos_y,  -sin_p*sin_r*sin_y - cos_r*cos_y,  sin_y*cos_p],
        [                     cos_p*cos_r,                    -sin_r * cos_p,       -sin_p]], dtype=np.float64)

    return R_EE


def compute_EE_position_error(theta_list, pose):
    '''
    evaluate_EE_error(theta_list: List[float], pose: Pose) -> Tuple[float, float, float, float]:
    
    x_error, y_error, z_error, absolute_error = evaluate_EE_error([theta1, theta2, theta3, theta4, theta5, theta6])
    
    :param theta_list: calculated values for joints 1:6 in ascending order
    :param pose: pose object contained in req.poses for which the given theta list was calculated
    :return: x_err, y_err, z_err, abs_err
    
    Calculates spatial positioning error of the end effector given the theta values calculated via inverse kinematics.
    The error is calculated by comparing the position requested in 'pose' with a the vector 'fk_position', which is 
    computed by substituting the given theta values into the translation column of a total homogeneous DH transform
    matrix. 
    '''
    # unpack theta values
    q1, q2, q3, q4, q5, q6 = theta_list

    # substitute joint angle values into  between the base link and the end effector
    # fk_position as represented also has frame error correction baked in
    fk_position = np.array([
        [-0.303 * (sin(q1) * sin(q4) + sin(q2 + q3) * cos(q1) * cos(q4)) * sin(q5)
         + (1.25 * sin(q2) - 0.054 * sin(q2 + q3) + 1.5 * cos(q2 + q3) + 0.35) * cos(q1)
         + 0.303 * cos(q1) * cos(q5) * cos(q2 + q3)],
        [-0.303 * (sin(q1) * sin(q2 + q3) * cos(q4) - sin(q4) * cos(q1)) * sin(q5)
         + (1.25 * sin(q2) - 0.054 * sin(q2 + q3) + 1.5 * cos(q2 + q3) + 0.35) * sin(q1)
         + 0.303 * sin(q1) * cos(q5) * cos(q2 + q3)],
        [-0.303 * sin(q5) * cos(q4) * cos(q2 + q3) - 0.303 * sin(q2 + q3) * cos(q5)
         - 1.5 * sin(q2 + q3) + 1.25 * cos(q2) - 0.054 * cos(q2 + q3) + 0.75]], dtype=np.float64)
    
    pose_target = np.array([[pose.position.x], [pose.position.y], [pose.position.z]])
    error_vect = pose_target - fk_position
    x_err, y_err, z_err = error_vect[0][0], error_vect[1][0], error_vect[2][0]
    abs_err = sqrt(error_vect[0][0] ** 2 + error_vect[1][0] ** 2 + error_vect[2][0] ** 2)
    
    return x_err, y_err, z_err, abs_err


def construct_R0_3_inverse(q1, q2, q3):
    '''
    construct_R0_3_inverse(q1: float, q2: float, q3: float) -> np.ndarray:

    :param q1: theta1 numeric value
    :param q2: theta2 numeric value
    :param q3: theta3 numeric value
    :return: 3x3 numpy array (dtype = np.float64); inverse of R0_3

    Substitutes the first three joint angles into a pre-determined matrix expression which encodes
    the inverse of the rotation matrix between link_0 and link_3 and returns this as a numpy float64 array.
    '''

    R0_3_inverse = np.array([
        [ sin(q2 + q3) * cos(q1), sin(q1) * sin(q2 + q3),  cos(q2 + q3)],
        [ cos(q1) * cos(q2 + q3), sin(q1) * cos(q2 + q3), -sin(q2 + q3)],
        [               -sin(q1),                cos(q1),             0]], dtype=np.float64)
    
    return R0_3_inverse


def handle_calculate_IK(req):
    '''
    handle_calculate_IK(req: Pose) -> CalculateIKResponse(joint_trajectory_list) | joint_trajectory_list:

    :param req: target end-effector position/orientation specified by the rospy geometry_msgs.msg/Pose message type
    :return: CalculateIKResponse(joint_trajectory_list)  -  a list of list of floats representing joint angles
    '''
    t_start = time.time() # record start time
    n_poses = len(req.poses)
    rospy.loginfo("Received %s eef-poses from the plan" % n_poses)
    
    if 'error_handler' in globals():
        global error_handler
    else:
        error_handler = None

    if n_poses < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        # Initialize end effector position error list if logging is logging/plotting active
        err_dict = {'x': [], 'y': [], 'z': [], 'abs': []}
        # Loop through poses in requested sequence, and do IK calculation to determine the joint values for each:
        for x, pose in enumerate(req.poses):
            # Initialize service response entry for pose
            joint_trajectory_point = JointTrajectoryPoint()
            #######################################################################################################
            # Determine theta1 from pose constraints on wrist center
            #######################################################################################################
            # Extract end-effector position and orientation from request
            # px, py, pz represent the requested end-effector position in the base reference frame
            px = pose.position.x
            py = pose.position.y
            pz = pose.position.z
            # Extract roll, pitch, and yaw from the orientation quaternion using method from tf.transformations
            # (roll, pitch, yaw) = euler_from_quaternion(quaternion_orientation)
            r_EE = construct_R_EE(euler_from_quaternion([
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]))
            # Calculate wrist center location
            EE_vector = 0.303 * r_EE[:, 2] # 0.303 is the displacement between the wrist center and the end effector
            wrist_x = px - EE_vector[0]
            wrist_y = py - EE_vector[1]
            wrist_z = pz - EE_vector[2]
            theta1 = atan2(wrist_y, wrist_x)
            #######################################################################################################
            # Determine theta2 and theta3 with SSS triangle formulae
            #######################################################################################################
            # radial displacement of the wrist center from the base_link origin, projected onto the xy-plane.
            wrist_s = sqrt(wrist_x ** 2 + wrist_y ** 2)
            # When the arm's working plane is aligned such that it contains the wrist center - a precondition for any
            # valid solution - the joint_2 axis/origin is set forward 0.35m along the
            delta_s = wrist_s - 0.35
            delta_z = wrist_z - 0.75
            # j2_pitch is the angle between the xy-plane and a line through both the world origin and the wrist center
            j2_pitch = atan2(delta_z, delta_s)
            # Triangle Solver
            # A, B, a, b, c = sp.symbols('A B a b c') # A, B are needed interior angles; a, b, c are side lengths
            # side lengths:
            #     a: 1.500971685275908, # distance between joint 3 and the wrist center
            #     b: j2_wrist_dist,
            #     c: 1.25
            #     }
            # j2_wrist_dist (the line segment between the midpoint/origin of joint_2 and wrist center) is the only
            # side of said triangle not actually measured along links of the manipulator.
            j2_wrist_dist = sqrt(delta_s ** 2 + delta_z ** 2)  # distance from joint_2 to wrist center; == len(side b)

            # Triangle with
            A = acos((j2_wrist_dist ** 2 + 1.25 ** 2 - 1.501 ** 2) / (2 * j2_wrist_dist * 1.25))
            B = acos((1.501 ** 2 + 1.25 ** 2 - j2_wrist_dist ** 2) / (2 * 1.501 * 1.25))
            theta2 = (np.pi / 2 - j2_pitch - A)
            theta3 = np.pi / 2 - (B + 0.036)  # 0.036 is the small angular correction to joint 4
            #######################################################################################################
            # Determine theta4-theta6 with matrix algebra and clever trig substitutions
            #######################################################################################################
            # R0_3_inverse is constructed by passing the first three joint angles to the  substituting the first three
            # joint angles into a template, R0_3_inverse_template
            # invR0_3 is always equivalent to the same expression, making the template matrix reusable for each pose
            # up to the following substitution, and thus is defined outside the loop

            # R0_3_inverse is the (reciprocal) inverse of the rotational transform R0_3, matrix
            R0_3_inverse = construct_R0_3_inverse(theta1, theta2, theta3)

            # By matrix multiplying R0_3_inverse onto r_EE, the original orientation vector, we are left with a matrix
            # which must represent the rotation between link_3 and link_6 (and therefore also link_7)
            R3_6 = R0_3_inverse.dot(r_EE)

            # theta4, theta5, and theta6 are all determined at this point, since we now have the inverse transform may
            # be isolated from solved for symbolically
            theta5 = atan2(sqrt(R3_6[0, 2] * R3_6[0, 2] + R3_6[2, 2] * R3_6[2, 2]), R3_6[1, 2])
            theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
            theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])

            # Populate response for the IK request
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

            if error_handler is not None:
                error_handler.record_EE_position_error(joint_trajectory_point.positions, pose)
            else:
                # Calculate end-effector positioning error for this pose via FK comparison
                x_err, y_err, z_err, abs_err = compute_EE_position_error(joint_trajectory_point.positions, pose)
                # Record error for end-of-request summary
                err_dict['x'].append(x_err)
                err_dict['y'].append(y_err)
                err_dict['z'].append(z_err)
                err_dict['abs'].append(abs_err)


        # This section contains some elementary error logging/monitoring
        if error_handler is not None:
            if SHOW_FIGS:
                rospy.logwarn('Figure display is blocking execution! Close figure window to resume IK_server functionality.')

            means, maxes = error_handler.digest_request_results()
            x_err_mean, y_err_mean, z_err_mean, abs_err_mean = means
            x_err_max, y_err_max, z_err_max, abs_err_max = maxes
            
        else:
            x_err_max = max(err_dict['x'])
            x_err_mean = np.mean(err_dict['x'])
            y_err_max = max(err_dict['y'])
            y_err_mean = np.mean(err_dict['y'])
            z_err_max = max(err_dict['z'])
            z_err_mean = np.mean(err_dict['z'])
            abs_err_max = max(err_dict['abs'])
            abs_err_mean = np.mean(err_dict['abs'])

        rospy.loginfo('maximum request error:\n x: {}\n y: {}\n z: {}\n abs: {}\n'.format(x_err_max, y_err_max, z_err_max, abs_err_max))
        rospy.loginfo('mean request error:\n x: {}\n y: {}\n z: {}\n abs: {}\n'.format(x_err_mean, y_err_mean, z_err_mean, abs_err_mean))
        rospy.loginfo("Final length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        rospy.loginfo("Full compute time (with overhead): %s" % (time.time() - t_start))
        rospy.loginfo("Mean compute time per pose: {}\n".format((time.time() - t_start) / len(joint_trajectory_list)))

        # If this function is running in the debugger, the special __name__ variable will NOT be '__main__';
        # This allows us to return a different entity more suitable for use as a direct import.
        # This conditional logic adds a tiny amount of overhead compared to the huge performance boosts elsewhere.
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
    SAVE_FIGS = False
    SHOW_FIGS = False
    error_handler = ErrorHandler(show_figures=SHOW_FIGS, save_figures=SAVE_FIGS)
    IK_server()

