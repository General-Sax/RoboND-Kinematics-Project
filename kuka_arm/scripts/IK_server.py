#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics Software Engineering Nanodegree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# Student contributor: Joel Tiura

import os
import time

import rospy
import numpy as np
import matplotlib.pyplot as plt

from tf.transformations import euler_from_quaternion
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# There's a lot of dense trig expressions ahead so I'll also directly import these for readability
# SymPy versions of sqrt, sin and cos are *NOT* used in the live code anywhere, so there isn't any namespace ambiguity
sin = np.sin
cos = np.cos
sqrt = np.sqrt
acos = np.arccos
atan2 = np.arctan2


class PerformanceMonitor:
    figure_dims = (12, 10)
    default_save_location = "outputs"
    
    def __init__(self, show_figures=False, save_figures=True):
        self.session_start_time = time.time()
        
        self.show_figures = show_figures
        self.save_figures = save_figures
        
        self.n_requests_processed = 0
        self.total_poses_processed = 0
        
        self.session_max_x_err = None
        self.session_max_y_err = None
        self.session_max_z_err = None
        self.session_max_abs_err = None
        
        self.error_results_archive = {}
        self.theta_results_archive = [[]]
        
        self.x_error_list = []
        self.y_error_list = []
        self.z_error_list = []
        self.abs_error_list = []
        
        self.plot_file_names = []
        
    
    def record_EE_position_error(self, theta_list, pose):
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
        
        self.theta_results_archive[-1].append(tuple(theta_list))
        
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

        # Record error for end-of-request summary
        self.x_error_list.append(x_err)
        self.y_error_list.append(y_err)
        self.z_error_list.append(z_err)
        self.abs_error_list.append(abs_err)
        
        
    def digest_request_results(self):
        assert len(self.x_error_list) == len(self.y_error_list) == len(self.z_error_list) == len(self.abs_error_list)
        
        self.n_requests_processed += 1
        self.total_poses_processed += len(self.x_error_list)
        
        # This section contains some elementary error
        x_err_max = max(self.x_error_list)
        x_err_mean = np.mean(self.x_error_list)
    
        y_err_max = max(self.y_error_list)
        y_err_mean = np.mean(self.y_error_list)
    
        z_err_max = max(self.z_error_list)
        z_err_mean = np.mean(self.z_error_list)
    
        abs_err_max = max(self.abs_error_list)
        abs_err_mean = np.mean(self.abs_error_list)

        means = (x_err_mean, y_err_mean, z_err_mean, abs_err_mean)
        maxes = (x_err_max, y_err_max, z_err_max, abs_err_max)

        self.error_results_archive[self.n_requests_processed] = {
            'x': tuple(self.x_error_list),
            'y': tuple(self.y_error_list[:]),
            'z': tuple(self.z_error_list[:]),
            'abs': tuple(self.abs_error_list[:]),
            'mean': means,
            'max': maxes,
        }

        self.x_error_list = []
        self.y_error_list = []
        self.z_error_list = []
        self.abs_error_list = []
        
        self.theta_results_archive.append([])
        
        if self.show_figures or self.save_figures:
            if self.save_figures:
                save_loc = PerformanceMonitor.default_save_location
            else:
                save_loc = None
                
            write_name = self.plot_req_error(self.n_requests_processed,
                                             show_fig=self.show_figures,
                                             save_location=save_loc)
            
            self.plot_file_names.append(write_name)
            
            
    def plot_req_error(self, req_number, show_fig=True, save_location=None):
        ''''''
        assert isinstance(show_fig, bool)
        assert isinstance(req_number, int)
        assert req_number > 0
        assert req_number <= max(self.error_results_archive.keys())
        assert show_fig or save_location, "function has no useful effects under these parameters!"
        
        error_dict = self.error_results_archive[req_number]
        
        x_err = [ex for ex in error_dict['x']]
        y_err = [ey for ey in error_dict['y']]
        z_err = [ez for ez in error_dict['z']]
        abs_err = [ea for ea in error_dict['abs']]
        

        inds = [i for i in range(len(x_err))]
        zero = [0 for _ in inds]
        
        y_max = 1.05 * max(max(x_err), max(y_err), max(z_err), max(abs_err))
        y_min = 1.05 * min(min(x_err), min(y_err), min(z_err), min(abs_err))

        output_fig = plt.figure(figsize=PerformanceMonitor.figure_dims)
        
        plt.subplot(211)
        plt.plot(inds, zero, color='k', aa=True, lw=2, alpha=0.6)
        plt.plot(inds, x_err, label='x error', color='r', aa=True, lw=3, alpha=0.9)
        plt.plot(inds, y_err, label='y error', color='g', aa=True, lw=3, alpha=0.9)
        plt.plot(inds, z_err, label='z error', color='b', aa=True, lw=3, alpha=0.9)
        plt.plot(inds, abs_err, label='absolute error', linestyle='-', color='k', aa=True, lw=2)
        plt.xlabel('pose number ( in order given; pose[0] <- 1 )')
        plt.ylabel('end eff. positioning error (meters)')
        plt.xlim(1, len(zero))
        plt.ylim(y_min, y_max)
        plt.legend()
        plt.title('End Effector Error History for Request: {}'.format(req_number))
        
        plt.subplot(212)
        plt.plot(inds, abs_err, label='absolute error', color='k', aa=True, lw=2)
        plt.xlabel('pose number ( in order given; pose[0] <- 1 )')
        plt.ylabel('absolute error (meters)')
        plt.xlim(1, len(zero))
        plt.ylim(min(abs_err), max(abs_err))
        plt.legend()
        plt.title('Absolute Error Variations\n range: {}'.format(max(abs_err) - min(abs_err)))
        
        output_fig.tight_layout()

        write_name = None
        
        if save_location is not None:
            assert isinstance(save_location, str)
            assert os.path.isdir(save_location)

            write_name = "pos_error_plot_req_{}_{}.png".format(req_number, time.strftime('[%T]').replace(':', '_'))
            file_path = os.path.join(save_location, write_name)

            plt.savefig(file_path)
            plt.close(output_fig)
        
        if show_fig:
            plt.show()
            
        return write_name


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
            EE_vector = 0.303 * r_EE[:, 2] # 0.303 is the length of the

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
            #     a: 1.500971685275908, the distance betweenj2_wrist_dist defined outside loop = 1.500971685275908
            #     b: j2_wrist_dist,
            #     c: 1.25
            #     }
            # j2_wrist_dist (the line segment between the midpoint/origin of joint_2 and wrist center) is the only
            # side of said triangle not actually measured along links of the manipulator.
            j2_wrist_dist = sqrt(delta_s ** 2 + delta_z ** 2)  # distance from joint_2 to wrist center; side b length.

            # Triangle with
            A = acos((j2_wrist_dist ** 2 + 1.25 ** 2 - 1.501 ** 2) / (2 * j2_wrist_dist * 1.25))
            B = acos((1.501 ** 2 + 1.25 ** 2 - j2_wrist_dist ** 2) / (2 * 1.501 * 1.25))

            theta2 = (np.pi / 2 - j2_pitch - A)
            theta3 = np.pi / 2 - (B + 0.036)  # j4_correction = 0.036

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

        #     # Calculate end-effector positioning error for this pose via FK comparison
        #     x_err, y_err, z_err, abs_err = compute_EE_position_error(joint_trajectory_point.positions, pose)
        #     # Record error for end-of-request summary
        #     err_dict['x'].append(x_err)
        #     err_dict['y'].append(y_err)
        #     err_dict['z'].append(z_err)
        #     err_dict['abs'].append(abs_err)
        #
        # # This section contains some elementary error logging/monitoring
        # x_err_max = max(err_dict['x'])
        # x_err_mean = np.mean(err_dict['x'])
        #
        # y_err_max = max(err_dict['y'])
        # y_err_mean = np.mean(err_dict['y'])
        #
        # z_err_max = max(err_dict['z'])
        # z_err_mean = np.mean(err_dict['z'])
        #
        # abs_err_max = max(err_dict['abs'])
        # abs_err_mean = np.mean(err_dict['abs'])

        rospy.loginfo('Error Maxima\n x: {}\n y: {}\n z: {}\n abs: {}\n'.format(x_err_max, y_err_max, z_err_max, abs_err_max))
        rospy.loginfo('Error Means\n x: {}\n y: {}\n z: {}\n abs: {}\n'.format(x_err_mean, y_err_mean, z_err_mean, abs_err_mean))
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
    IK_server()
