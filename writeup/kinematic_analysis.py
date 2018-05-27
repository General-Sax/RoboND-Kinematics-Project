import collections

import numpy as np
import sympy as sp
import matplotlib.pyplot as plt

from mpmath import degrees, radians
from sympy import cos, sin, sqrt, symbols, Symbol, simplify, pi, atan2
from sympy.matrices import Matrix, eye



def column(spatial_components, homogeneous=False):
    '''
    Convenience function that ensures consistent use of appropriate column-aligned sympy vectors.
    :param spatial_components: ordered iterable of objects compatible with sympy matrices
    :param homogeneous: option to generate homogeneous coordinate vector by
     appending a 4th row containing the alue '1'.
    '''
    column_matrix = [[item] for item in spatial_components]
    if homogeneous:
        column_matrix.append([1])
    return Matrix(column_matrix)


def sym_rot_x(q):
    '''
    Construct a symbolic matrix corresponding to a rotatation about the x-axis
    '''
    R_x = Matrix([
        [1,      0,       0],
        [0, cos(q), -sin(q)],
        [0, sin(q),  cos(q)],
    ])
    return R_x


def sym_rot_y(q):
    R_y = Matrix([
        [ cos(q), 0, sin(q)],
        [      0, 1,      0],
        [-sin(q), 0, cos(q)],
    ])
    return R_y


def sym_rot_z(q):
    R_z = Matrix([
        [cos(q), -sin(q), 0],
        [sin(q),  cos(q), 0],
        [     0,       0, 1],
    ])
    return R_z



def compose_sym_rotations(rotations):
    '''
    Compose a sympy rotation matrix corresponding to the sequence of rotations given.
    rotations: ordered iterable (of arbitrary length), which contains iterables of form (axis, radian_angle), 
        where axis is an element of {'x', 'y', 'z'}.
    '''
    
    assert isinstance(rotations, collections.Iterable)
    assert all(len(rot)==2 for rot in rotations)
    assert all(rot[0] in {'x', 'y', 'z'} for rot in rotations)
    
    transform_about = {
        'x': sym_rot_x,
        'y': sym_rot_y,
        'z': sym_rot_z,
    }
    
    iteratively_composed_matrix = eye(3)
    
    for rotation in rotations:
        rotation_axis, rotation_angle = rotation[0], rotation[1]
        new_transform = transform_about[rotation_axis](rotation_angle)
        iteratively_composed_matrix = new_transform * iteratively_composed_matrix
        
    return iteratively_composed_matrix


def sym_tf_matrix(rotation_list, translation):
    '''
    :param rotation_list: rotation sequence as specified by compose_sym_rotations function
    :param translation: iterable of cartesian coords of cumulative translation (x, y, z)
    :return: 4x4 sympy Matrix representing dh transform
    '''
    rot_matrix = compose_sym_rotations(rotation_list)
    return rot_matrix.col_join(Matrix([[0, 0, 0]])).row_join(
        column(translation), homogeneous=True)


def single_dh_transform_step(alpha, a, d, q):
    # alpha, a, d, q = twist_angle, link_length, joint_angle, link_offset
    sub_transform_x = sym_tf_matrix([('x', alpha)], (a, 0, 0))
    sub_transform_z = sym_tf_matrix([('z', q)], (0, 0, d))
    return simplify(sub_transform_x * sub_transform_z)


def harvest_dh_transform_matrix(dh_transform_matrix):
    rotation_sub_matrix = dh_transform_matrix[0:3, 0:3]
    translation_sub_matrix = dh_transform_matrix[0:3, 3]
    return rotation_sub_matrix, translation_sub_matrix


def rotation_sub_matrix(dh_transform_matrix):
    return dh_transform_matrix[0:3, 0:3]


def translation_sub_matrix(dh_transform_matrix):
    return dh_transform_matrix[0:3, 3]


def vector_cross_angle(first_vector, second_vector):
    # The vector arguments should be Matrix objects with three elements
    assert len(first_vector) == len(second_vector) == 3
    assert isinstance(first_vector, Matrix)
    assert isinstance(second_vector, Matrix)
    unit_1 = first_vector.normalized()
    unit_2 = second_vector.normalized()
    cross_vector = unit_1.cross(unit_2)
    return N(asin(cross_vector.norm()))


def vector_cross_unit(first_vector, second_vector):
    # The vector arguments should be Matrix objects with three elements
    assert len(first_vector) == len(second_vector) == 3
    assert isinstance(first_vector, Matrix)
    assert isinstance(second_vector, Matrix)
    unit_1 = first_vector.normalized()
    unit_2 = second_vector.normalized()
    cross_vector = unit_1.cross(unit_2)
    return cross_vector.normalized()

###################################################################################################
# THE NUMERICAL ZONE


# Assign alternate names for numpy versions of relevant functions
_cos = np.cos
_sin = np.sin
_sqrt = np.sqrt
_atan2 = np.arctan2
_pi = np.pi

# The eager_num_rot_ pattern of functions quickly calculate 
# the numerical matrices for evaluating forward and reverse
# kinematics outcomes for given inputs

def num_rot_x(q):
    '''
    Compose a 3d non-homogeneous rotation matrix about x-axis, by q radians.
    '''
    r_x = np.array([
        [1,       0,        0],
        [0, _cos(q), -_sin(q)],
        [0, _sin(q),  _cos(q)],
    ])
    return r_x


def num_rot_y(q):
    '''
    Generate a non-homogeneous rotation matrix in 3d about y-axis;
    matrix values correspond to a rotation by q radians.
    :param q: angular displacement to be calculated, in radians
    :return: a 2d (3x3) numpy.ndarray containing the numerical
    '''
    r_y = np.array([
        [ _cos(q), 0, _sin(q)],
        [      0,  1,       0],
        [-_sin(q), 0, _cos(q)],
    ], dtype=np.float32)
    return r_y


def num_rot_z(q):
    '''
    representing a rotat 3d, non-homogeneous rotation matrix about z-axis, by q radians.
    :return: a 2d (3x3) numpy.ndarray containing numerical outputs of all trig operations
    '''
    r_z = np.array([
        [_cos(q), -_sin(q), 0],
        [_sin(q),  _cos(q), 0],
        [      0,        0, 1],
    ])
    return r_z

def num_wrist_center(angles):
    assert len(angles) == 3
    (q1, q2, q3) = angles
    # x = (1.25 * _sin(q2) - 0.054 * _sin(q2 + q3) + 1.5 * _cos(q2 + q3) + 0.35) * _cos(q1)
    # y = (1.25 * _sin(q2) - 0.054 * _sin(q2 + q3) + 1.5 * _cos(q2 + q3) + 0.35) * _sin(q1)
    # z = -1.5 * _sin(q2 + q3) + 1.25 * _cos(q2) - 0.054 * _cos(q2 + q3) + 0.75
    return [
        (1.25 * _sin(q2) - 0.054 * _sin(q2 + q3) + 1.5 * _cos(q2 + q3) + 0.35) * _cos(q1),
        (1.25 * _sin(q2) - 0.054 * _sin(q2 + q3) + 1.5 * _cos(q2 + q3) + 0.35) * _sin(q1),
        -1.5 * _sin(q2 + q3) + 1.25 * _cos(q2) - 0.054 * _cos(q2 + q3) + 0.75,
    ]
    
def num_fk_EE(angles):
    assert len(angles) == 6
    (q1, q2, q3, q4, q5, q6) = angles
    '''
    fk_EE is the translation vector from the total homogeneous transform
    between the base and the end effector; this representation has frame error
    correction baked in!
    '''
    return [
        -0.303 * (_sin(q1) * _sin(q4) + _sin(q2 + q3) * _cos(q1) * _cos(q4)) * _sin(q5) + (1.25 * _sin(q2) - 0.054 * _sin(q2 + q3) + 1.5 * _cos(q2 + q3) + 0.35) * _cos(q1) + 0.303 * _cos(q1) * _cos(q5) * _cos(q2 + q3),
        -0.303 * (_sin(q1) * _sin(q2 + q3) * _cos(q4) - _sin(q4) * _cos(q1)) * _sin(q5) + (1.25 * _sin(q2) - 0.054 * _sin(q2 + q3) + 1.5 * _cos(q2 + q3) + 0.35) * _sin(q1) + 0.303 * _sin(q1) * _cos(q5) * _cos(q2 + q3),
        -0.303 * _sin(q5) * _cos(q4) * _cos(q2 + q3) - 0.303 * _sin(q2 + q3) * _cos(q5) - 1.5 * _sin(q2 + q3) + 1.25 * _cos(q2) - 0.054 * _cos(q2 + q3) + 0.75
    ]

###################################################################################################



#####################################################################################################################
# from IK_server


def construct_R_EE(orientation):
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
        [-0.303 * (sin(q1n) * sin(q4n) + sin(q2n + q3n) * cos(q1n) * cos(q4n)) * sin(q5n) + (
                1.25 * sin(q2n) - 0.054 * sin(q2n + q3n) + 1.5 * cos(q2n + q3n) + 0.35) * cos(q1n) + 0.303 * cos(
            q1n) * cos(q5n) * cos(q2n + q3n)],
        [-0.303 * (sin(q1n) * sin(q2n + q3n) * cos(q4n) - sin(q4n) * cos(q1n)) * sin(q5n) + (
                1.25 * sin(q2n) - 0.054 * sin(q2n + q3n) + 1.5 * cos(q2n + q3n) + 0.35) * sin(q1n) + 0.303 * sin(
            q1n) * cos(q5n) * cos(q2n + q3n)],
        [-0.303 * sin(q5n) * cos(q4n) * cos(q2n + q3n) - 0.303 * sin(q2n + q3n) * cos(q5n) - 1.5 * sin(
            q2n + q3n) + 1.25 * cos(q2n) - 0.054 * cos(q2n + q3n) + 0.75]],
        dtype=np.float64)
    
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

#####################################################################################################################
# From IK_debug

############################################################################
# angles = tuple(handle_calculate_IK(req, debug=number + 1)[0].positions)
# theta1, theta2, theta3, theta4, theta5, theta6 = angles

def check_joint_limits(theta_list, raise_exception=False):
    assert isinstance(theta_list, collections.Iterable)
    assert len(theta_list) == 6
    joint_range_limits = {
        1: (-3.2288591161895095, 3.2288591161895095),
        2: (-0.7853981633974483, 1.4835298641951802),
        3: (-3.6651914291880923, 1.1344640137963142),
        4: (-6.1086523819801535, 6.1086523819801535),
        5: (-2.181661564992912, 2.181661564992912),
        6: (-6.1086523819801535, 6.1086523819801535)
    }
    violations = []
    for i, angle in enumerate(theta_list):
        i+=1
        if not (joint_range_limits[i][0] <= angle <= joint_range_limits[i][1]):
            report = "\n*** Angle theta" + str(i) + " outside joint range! ***"
            report += "Range: " + str(joint_range_limits[i]) + " | Theta: " + str(angle) + "\n"
            if raise_exception:
                raise RuntimeError(report)
            else:
                violations.append(report)
    if violations:
        return violations


if __name__ == "__main__":
    sym_R_corr = Matrix([
        [0,  0,  1,  0],
        [0, -1,  0,  0],
        [1,  0,  0,  0],
        [0,  0,  0,  1],
    ])

    num_R_corr = np.array([
        [0,  0,  1,  0],
        [0, -1,  0,  0],
        [1,  0,  0,  0],
        [0,  0,  0,  1],
    ])

    e_x = column((1, 0, 0))
    e_y = column((0, 1, 0))
    e_z = column((0, 0, 1))

    e_x_h = column((1, 0, 0), homogeneous=True)
    e_y_h = column((0, 1, 0), homogeneous=True)
    e_z_h = column((0, 0, 1), homogeneous=True)

    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')  # theta angles
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
    # Enter table of values:
    dh_parameters = {
        alpha0: 0,        a0: 0,      d1: 0.75,   #q1: q1,
        alpha1: -pi / 2,  a1: 0.35,   d2: 0,      q2: q2 - pi / 2,
        alpha2: 0,        a2: 1.25,   d3: 0,      #q3: q3,
        alpha3: -pi / 2,  a3: -0.054, d4: 1.50,   #q4: q4,
        alpha4: pi / 2,   a4: 0,      d5: 0,      #q5: q5,
        alpha5: -pi / 2,  a5: 0,      d6: 0,      #q6: q6,
        alpha6: 0,        a6: 0,      d7: 0.303,  q7: 0
    }

    T0_1 = single_dh_transform_step(alpha0, a0, d1, q1).subs(dh_parameters)
    T1_2 = single_dh_transform_step(alpha1, a1, d2, q2).subs(dh_parameters)
    T2_3 = single_dh_transform_step(alpha2, a2, d3, q3).subs(dh_parameters)
    T3_4 = single_dh_transform_step(alpha3, a3, d4, q4).subs(dh_parameters)
    T4_5 = single_dh_transform_step(alpha4, a4, d5, q5).subs(dh_parameters)
    T5_6 = single_dh_transform_step(alpha5, a5, d6, q6).subs(dh_parameters)
    T6_7 = single_dh_transform_step(alpha6, a6, d7, q7).subs(dh_parameters)
