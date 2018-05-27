
# Pick & Place (Kinematics)

UDACITY GITHUB: https://github.com/udacity/RoboND-Kinematics-Project

MY GITHUB: https://github.com/general-sax/RoboND-Kinematics-Project

RUBRIC: https://review.udacity.com/#!/rubrics/972/view

WALKTHROUGH: https://www.youtube.com/watch?v=Gt8DRm-REt4


## What to include in your submission
You may submit your project as a zip file or with a link to a GitHub repo. The submission must include these items:
    - IK_server.py file with your code (make sure to add comments at appropriate places in your code)
    - Writeup report (md or pdf file)

## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.


## Kinematics Project: Kuka KR210 Pick & Place
*Inverse kinematics for control of the KUKA KR210 serial manipulator.*

The KUKA KR210 is an industrial robotic arm (serial manipulator) which presents an excellent testbed to explore inverse kinematics, due to its obliging geometry. It has a spherical wrist supported by coplanar system links with    with 6 joints, also referred to as axes or degrees of freedom (DOF's). In this project, the objective was to implement

---

[//]: # (Image References)

[dh_param_definition]: writeup_graphics/denavit-hartenberg-parameter-definitions-01-modified.png
[columns]: math/columns.png
[generic_dh_tf_matrix]: math/generic_dh_tf_single_step.png
[r_corr_sym]: math/rpy_not_corrected.png
[r_corr_inelegant]: math/inelegant_correction.png
[r_corr_elegant]: math/elegant_correction.png
[sym_rot_rpy]: math/roll_pitch_yaw_together.png
[sym_rot_rpy_corr]: math/corrected_R_EE_from_rpy.png
[dh_transform_0_1]: math/T0_1.png
[dh_transform_1_2]: math/T1_2.png
[dh_transform_2_3]: math/T2_3.png
[dh_transform_3_4]: math/T3_4.png
[dh_transform_4_5]: math/T4_5.png
[dh_transform_5_6]: math/T5_6.png
[dh_transform_6_7]: math/T6_7.png
[complete_transform_unreadable]: math/hideous_T0_EE.png
[R_EE_uncorrected]: math/rpy_not_corrected.png
[R_EE_corrected]: math/corrected_R_EE_from_rpy.png
[R0_3]: math/r0_3.png
[R0_3_inverse]: math/R0_3_inverse.png
[r0_EE_full]: math/R0_EE_monster.png
[R3_6_a]: math/r3_6_a.png
[R3_6_b]: math/r3_6_b.png
[solving_q4]: math/solving_q4.png
[solving_q5]: math/solving_q5.png
[solving_q5_sum_squares]: math/solving_q5_sum_squares.png
[solving_q5_intermediate]: math/solving_q5_intermediate.png
[q5_result]: math/q5_result.png
[solving_q6]: math/solving_q6.png

# Kinematic Analysis of KUKA KR210
## Joel Tiura
***
## Introduction / Abstract
## Forward Kinematics Functionality as Design Prerequisite
### General DH Transform Matrices, Associated Utilities
## Forward Kinematics Tool
## Analysis of Robot Geometry and Analytical Model Definition
***
### Denavit-Hartenberg Parameterization
#### URDF File Links
##### Link Lengths
##### Link Geometry
##### Link Origins
#### URDF File Joints
##### Joint Types
##### Joint Coordinates
##### Joint Ranges

#### Composing DH-Compatible Geometric Model
![annotated dh parameter illustration][dh_param_definition]
*O(i-1): link i-1 origin, O(i): link i origin, alpha(i-1): *


#### DH Parameter Table

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 |   0.35 |      0 | q2 - pi/2
2->3 | 0 |   1.25 |      0 | q3
3->4 | - pi/2 | -0.054 |   1.50 | q4
4->5 | pi/2 |      0 |      0 | q5
5->6 | - pi/2 |      0 |      0 | q6
6->EE | 0 |      0 |  0.303 | 0


#### Generate DH FK Transform Matrices from Table
```python
import sympy as sp
from sympy import pi, cos, sin, sqrt, atan2, acos, simplify 
from sympy.matrices import Matrix, eye


def column(spatial_components, homogeneous=False):
    '''
    Convenience function that ensures consistent use of appropriate column-aligned sympy vectors.

    :param spatial_components: ordered iterable of objects compatible with sympy matrices
    :param homogeneous: bool; generate homogeneous coordinate vector by adding a 4th row containing a 1
    '''
    column_matrix = [[item] for item in spatial_components]
    if homogeneous:
        column_matrix.append([1])
    return Matrix(column_matrix)

def sym_rot_x(q):
    return Matrix([
        [ 1,      0,       0],
        [ 0, cos(q), -sin(q)],
        [ 0, sin(q),  cos(q)]])

def sym_rot_y(q):
    return Matrix([
        [  cos(q), 0, sin(q)],
        [       0, 1,      0],
        [ -sin(q), 0, cos(q)]])

def sym_rot_z(q):
    return Matrix([
        [ cos(q), -sin(q), 0],
        [ sin(q),  cos(q), 0],
        [      0,       0, 1]])
```

```python
# Elementary basis vectors x, y, z (non-homogeneous representation)
e_x = column((1, 0, 0))
e_y = column((0, 1, 0))
e_z = column((0, 0, 1))

# Homogeneous elementary basis vectors x, y, z
e_x_h = column((1, 0, 0), homogeneous=True)
e_y_h = column((0, 1, 0), homogeneous=True)
e_z_h = column((0, 0, 1), homogeneous=True)

((e_x, e_y, e_z), (e_x_h, e_y_h, e_z_h))
```

![column output test][columns]

```python
def compose_sym_rotations(rotations):
    '''
    Compose a sympy rotation matrix corresponding to the sequence of rotations given.
    rotations: ordered iterable (of arbitrary length), which contains iterables of form (axis, radian_angle),
        where axis is an element of {'x', 'y', 'z'}.
    '''
    # This dictionary maps axis strings to function identifiers which return corresponding rotation matrices 
    transform_about = {
        'x': sym_rot_x,
        'y': sym_rot_y,
        'z': sym_rot_z,
    }
    # initial identity matrix to prime the composition loop
    iteratively_composed_matrix = eye(3)
    for rotation in rotations:
        rotation_axis, rotation_angle = rotation[0], rotation[1]
        new_transform = transform_about[rotation_axis](rotation_angle)
        iteratively_composed_matrix = new_transform * iteratively_composed_matrix

    return iteratively_composed_matrix
```


```python
def symbolic_homogeneous_transform(rotation_list, translation):
    '''
    :param rotation_list: rotation sequence as specified by compose_sym_rotations function
    :param translation: iterable of cartesian coords of cumulative translation (x, y, z)
    :return: 4x4 sympy Matrix representing dh transform
    '''
    rot_matrix = compose_sym_rotations(rotation_list)
    return rot_matrix.col_join(Matrix([[0, 0, 0]])).row_join(column(translation, homogeneous=True))
```




```python
# Invoke generalized dh parameter symbols:
# alpha, a, d, q = twist_angle, link_length, joint_angle, link_offset
a, q, d, alpha = sp.symbols('a, q, d, alpha')

def single_dh_transform_step(alpha, a, d, q):
    '''Composes the Denavit-Hartenberg homogeneous transform matrix for the
    given values of alpha, a, d, and q.
    
    This matrix is produced by creating two precursors, each representing one 
    rotational and one translational component of the DH transform step, then
    composes them into a single transform and simplifies the result.
    '''
    sub_transform_x = symbolic_homogeneous_transform([('x', alpha)], (a, 0, 0))
    sub_transform_z = symbolic_homogeneous_transform([('z', q)], (0, 0, d))
    return sp.simplify(sub_transform_x * sub_transform_z)

generic_dh_tf_matrix = single_dh_transform_step(a, q, d, alpha)
```

![][generic_dh_tf_matrix]


```python
# Create symbols for DH parameters
q1, q2, q3, q4, q5, q6, q7 = sp.symbols('q1:8')  # theta angles
d1, d2, d3, d4, d5, d6, d7 = sp.symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = sp.symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = sp.symbols('alpha0:7')

# symbols for end effector target coordinates in base_link reference frame
x_target, y_target, z_target = sp.symbols('x_target y_target z_target')

# Create symbols for wrist center location in base frame
z_wrist, y_wrist, z_wrist = sp.symbols('wrist wrist z_wrist')

# Create Modified DH parameters
# Define the symbol dictionary which contains lookup values for the system
# ATTENTION! Note that joints 4-6 share a common origin, actually located at the wrist center,
# at the intersection of their Z axes.
dh_parameters = {alpha0:       0, a0:      0, d1: 0.75,
                 alpha1: -pi / 2, a1:   0.35, d2: 0,      q2: q2 - pi / 2,
                 alpha2:       0, a2:   1.25, d3: 0,
                 alpha3: -pi / 2, a3: -0.054, d4: 1.50,
                 alpha4:  pi / 2, a4:      0, d5: 0,
                 alpha5: -pi / 2, a5:      0, d6: 0,
                 alpha6:       0, a6:      0, d7: 0.303,  q7: 0
                 }

# Create individual transformation matrices
T0_1 = single_dh_transform_step(alpha0, a0, d1, q1).subs(dh_parameters)
T1_2 = single_dh_transform_step(alpha1, a1, d2, q2).subs(dh_parameters)
T2_3 = single_dh_transform_step(alpha2, a2, d3, q3).subs(dh_parameters)
T3_4 = single_dh_transform_step(alpha3, a3, d4, q4).subs(dh_parameters)
T4_5 = single_dh_transform_step(alpha4, a4, d5, q5).subs(dh_parameters)
T5_6 = single_dh_transform_step(alpha5, a5, d6, q6).subs(dh_parameters)
T6_7 = single_dh_transform_step(alpha6, a6, d7, q7).subs(dh_parameters)
```
**T0_1:**

![T0_1][dh_transform_0_1]

**T1_2:**

![T1_2][dh_transform_1_2]

**T2_3:**

![T2_3][dh_transform_2_3]

**T3_4:**

![T3_4][dh_transform_3_4]

**T4_5:**

![T4_5][dh_transform_4_5]

**T5_6:**

![T5_6][dh_transform_5_6]

**T6_7:**

![T6_7][dh_transform_6_7]



```python
T0_EE = sp.simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_7)
```

**T0_EE**

![T0_EE][complete_transform_unreadable]


I intend to keep this monstrosity out of any live code if I can help it.


```python
r, p, y = sp.symbols('r p y')
R_corr_sym = sym_rot_z(y) * sym_rot_y(p)
```

![][r_corr_sym]

```python
# Walkthrough method
R_corr = sym_rot_z(y).subs(y, radians(180)) * sym_rot_y(p).subs(p, radians(-90))
```
![][r_corr_inelegant]

```python
# Pure symbolic method
# Conversion Factors - Note that this is using the symbolic definition of pi from SymPy
rtd = 180 / sp.pi  # radians to degrees
dtr = sp.pi / 180  # degrees to radians

R_corr = R_corr_sym.subs({y: dtr * 180, p: dtr * -90})
```

![][r_corr_elegant]



```python
R0_1 = T0_1[0:3, 0:3]
R1_2 = T1_2[0:3, 0:3]
R2_3 = T2_3[0:3, 0:3]
R3_4 = T3_4[0:3, 0:3]
R4_5 = T4_5[0:3, 0:3]
R5_6 = T5_6[0:3, 0:3]
R6_7 = T6_7[0:3, 0:3]

R0_3 = sp.simplify(R0_1 * R1_2 * R2_3)
```

![][r0_3]


```python
R0_EE = sp.simplify(R0_3 * R3_4 * R4_5 * R5_6 * R6_7)
```

As it happens, the result of this calculation is the same whether or not R6_7 is included.
It could be safely ignored, since the gripper is fixed relative to link_6.
I include it just to be thorough. In any case, this composed sequence of rotations is a mess.
This is another entity I choose to quarantine in the world of theory.


![][r0_EE_full]


```python
R_roll = sym_rot_x(r)
R_pitch = sym_rot_y(p)
R_yaw = sym_rot_z(y)
```
**R_roll, R_pitch, R_yaw:**

![][sym_rot_rpy]


```python
R_EE_uncorrected = R_yaw * R_pitch * R_roll
```


![][R_EE_uncorrected]

```python
R_EE = R_EE_uncorrected * R_corr
```

![][R_EE_corrected]

```python
R0_3_inverse = sp.simplify(R0_3.inv('LU'))
```

![][R0_3_inverse]

This result is totally independent of the pose in question; so long as we can calculate q1, q2, and q3 values, we can determine the numerical-valued form of R0_3_inverse with nothing more than a substitution.


```python
# If we've done this right, the following should produce a 3x3 identity matrix:
eye3_if_good = simplify(R0_3 * R0_3_inverse)
assert eye3_if_good == eye(3)
```



```python
R3_6 = R0_3_inverse * R0_EE
```

![][R3_6_a]


I immediately suspected that R3_6[2, 2] was not as simplified as it could have been.
I compared the previous result with several trig identities, was able to show it was equivalent to the product manually inserted into R3_6 below: 

```python
R3_6 = Matrix([
    [-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -cos(q4)*sin(q5)],
    [                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5)],
    [-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5)]])
```

![][R3_6_b]

```python
r_11 = R3_6[0, 0]
r_12 = R3_6[0, 1]
r_13 = R3_6[0, 2]
r_21 = R3_6[1, 0]
r_22 = R3_6[1, 1]
r_23 = R3_6[1, 2]
r_31 = R3_6[2, 0]
r_32 = R3_6[2, 1]
r_33 = R3_6[2, 2]
```



The objective here is to come up with formulas for q4, q5, and q6 in terms of particular indexed elements of a 3x3 matrix.
The rotation matrix R3_6_edit, above, is defined in terms of DH parameters, which we are solving for, but the matrix we will draw values from
is the numerical matrix R3_6 = R0_3_inverse.dot(r_EE), where R0_3_inverse is determined by q1-q3, and r_EE is determined by the rpy euler angles from the pose.

In a sense, the above 'r_ij' variables can be considered placeholders for numerical values that will be mapped to joint angles through relationships this matrix encodes.


#### Solving q4
```python
r_33, r_13, r_33 / -r_13
```

![][solving_q4]




#### Solving q5
```python
r_13, r_33, r_23
```

![][solving_q5]

```python
r_13 ** 2 + r_33 ** 2
```

![][solving_q5_sum_squares]

```python
sp.factor(r_13 ** 2 + r_33 ** 2), sp.simplify(r_13 ** 2 + r_33 ** 2)
```

![][solving_q5_intermediate]


```python
sp.sqrt(sp.simplify(r_13 ** 2 + r_33 ** 2)) / r_23, sin(q5)/cos(q5)
```


#### Solving q6
```python
r_22, r_21, -r_22 / r_21
```

![][solving_q6]



#### Aggregated Results:
```python
theta5 = atan2(sqrt(R3_6[0, 2] * R3_6[0, 2] + R3_6[2, 2] * R3_6[2, 2]), R3_6[1, 2]) # r_13**2 + r_33**2, r_23
theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])
```



**=============================================================================================**

**CRITERIA: Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.**

**MEETS SPECIFICATIONS: Your writeup should contain individual transform matrices about each joint using the DH table and a homogeneous transform matrix from base_link to gripper_link using only the position and orientation of the gripper_link. These matrices can be created using any software of your choice or hand written. Also include an explanation on how you created these matrices.**

**=============================================================================================**




**=============================================================================================**

**CRITERIA: Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.**

**MEETS SPECIFICATIONS: Based on the geometric Inverse Kinematics method described here, breakdown the IK problem into Position and Orientation problems. Derive the equations for individual joint angles. Your writeup must contain details about the steps you took to arrive at those equations. Add figures where necessary. If any given joint has multiple solutions, select the best solution and provide explanation about your choice (Hint: Observe the active robot workspace in this project and the fact that some joints have physical limits).**

**=============================================================================================**

## Module Implementation: IK_server.py
### Design Overview
In my preparatory analysis and early experiments developing a set of inverse kinematics equations, I noticed that most of the problem was completely general and invariant over all poses, and only needed to be derived once in a theoretical context. That context was using SymPy in Jupyter notebooks, where I could leverage the power of the notebooks' cell-based execution to streamline exploratory work. 

SymPy was a delight to use, and helped me make some clever optimizations. By using its robust computer algebra system to compute exact symbolic relations, I was able to derive a coordinate system correction matrix which was not to make performance improvements in code that di, and using

As I started implementing a solution, I found that I didn't like the suggested organization of the handle_calculate_IK function. It struck me as inelegant to be programmatically constructing and manipulating invariant constructs solution inside the working server code. and as such, I sought to  as many features underlying problem structure as possible. much of the process of deriving the components of a working solution was  to distill the inverse kinematics problem down to its essence, and do my best to refine the IK_server loop into a representation of a solution to that problem.
weed out redundant calculations, the minimum number of 'moving parts'

### Function: handle_calculate_IK
r_EE is the rotational component of the transformation matrix between the base coord
roll, pitch, and yaw specify end-effector orientation relative to the base coordinate system,
but only when the corresponding sequence of rotations is applied with respect to static coordinates, and
in an XYZ axis order.

This sequence and convention is not a coincidence; it's the is the default behavior of the
tf.transformations.euler_from_quaternion() function.

```
(roll, pitch, yaw) = tf.transformations.euler_from_quaternion([
    req.poses[x].orientation.x,
    req.poses[x].orientation.y,
    req.poses[x].orientation.z,
    req.poses[x].orientation.w
    ])
```
These values of roll, pitch, and yaw are calculated using the 'sxyz' convention defined in the tf library,
where *'s'* denotes static frame, and *'xyz'* gives an axis sequence for the rotations.

while the wrist center's location is not specified explicitly, it is already determined
*implicitly* by the specified end effector orientation.
     

`r_EE` is the general symbolic rotation matrix for the end effector orientation, defined outside loop.
r_EE represents this matrix for a given pose, substituting in the roll, pitch, and yaw variables
extracted from the pose orientation quaternion.


### Function: construct_R_EE
#### Derivation
#### Implementation
```
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
        [sin_p * cos_r * cos_y + sin_r * sin_y,    -sin_p * sin_r * cos_y + sin_y * cos_r,    cos_p * cos_y],
        [sin_p * sin_y * cos_r - sin_r * cos_y,    -sin_p * sin_r * sin_y - cos_r * cos_y,    sin_y * cos_p],
        [                        cos_p * cos_r,                            -sin_r * cos_p,          -sin_p]],
        dtype=np.float64)
    
    return R_EE
```

As with most of my work on this project, I designed this function to be fast and to encode as much of the problem's
invariant features into its literal structure as possible, factoring the theoretical analysis out of the engineered
solution.

This implementation opts to eliminate the redundant, invariant procedure of instantiating a matrix object to 
represent the coordinate system correction matrix (R_corr), adding more code and computational load,

error correction transform  for the coordinate system mismatch by evaluating substitutions numerically, which leads
to small but avoidable error, I opted to solve it symbolically and just multiply it into the total end effector 
transform which is then represented as exact symbolic formulae, cleanly expressed in static code, using numpy
trig functions in place of their sympy equivalents.

Includes coordinate correction implicitly by having multiplied in the following matrix during the derivation of the complete transform:
```
R_corr = Matrix([[ 0,  0,  1],
                 [ 0, -1,  0],
                 [ 1,  0,  0]])
```
Where it has been demonstrated that given:
```
x = Matrix([[1], [0], [0]]) # == column((1, 0, 0));
y = Matrix([[0], [1], [0]]) # == column((0, 1, 0));
z = Matrix([[0], [0], [1]]) # == column((0, 0, 1));
```
The following desired equalities hold:
```
R_corr * x == Matrix([[0], [ 0], [1]]) ==  z
R_corr * y == Matrix([[0], [-1], [0]]) == -y
R_corr * y == Matrix([[1], [ 0], [0]]) ==  x
```
So it is certainly the correct transform.


### Function: compute_EE_position_error
#### Derivation
#### Implementation
```
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
```
It is sufficient to evaluate only this three-element slice of the transform matrix, so the complete matrix is not 
represented in this function, but was tested and shown to be a valid model of the KR210 during the design process.



### Function: construct_R0_3_inverse
#### Derivation


#### Implementation
```
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
```

This condensed, direct invocation of the rotation matrix inverse is an attempt at elegant minimalism, along
the lines of 'less code, less trouble.' Because so many properties of the system are invariant between poses,
it's possible to abstract a lot of theory out of the implementation. This array creation expression was derived
symbolically using sympy and then translated into a numpy ndarray constructor and numpy trig functions.

Part of the appeal of this implementation for me is it's further potential for vectorization. While the rest of the
code base would need adjusting before this would bear fruit, it should not be too difficult to pass in vector args,
and return an array of transform matrices calculated with even more efficiency.


#### Design Considerations and Philosophy
##### Sympy is for Theory; Numpy is for Practice
##### Baking in Invariant Quantities and Computations
##### IK_debug.py Integration
##### Position Error Calculation and Logging
##### Error Visualization - Automated Image Output with Matplotlib

#### Key Features and Selected Highlights
#### Extensions, Future Plans, Unfinished Features
- re-implement sympy matrix derivations from jupyter notebooks as automatic constructor functions;
- include functionality to pickle these so that they can be loaded faster and into other code.
- enhances reproducibility and transparency of algorithm.

**=============================================================================================**

**CRITERIA: Fill in the IK_server.py file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. A screenshot of the completed pick and place process is included.**

**MEETS SPECIFICATIONS: IK_server.py must contain properly commented code. The robot must track the planned trajectory and successfully complete pick and place operation. Your writeup must include explanation for the code and a discussion on the results, and a screenshot of the completed pick and place process.**

**=============================================================================================**
### IK_debug.py
#### Direct `import handle_ik_request` from IK_server.py
Avoids repetitive and error-prone process of manually copy-pasting into IK_debug.py
For simplicity/operability, moved IK_debug.py to scripts folder
This required only minimal modifications to IK_server, now tested and stable.

#### Added Test Case 0 (Neutral Pose) 
corresponding to the neutral/zero configuration of the arm, as specified by the DH parameters

#### Improved Output Text Formatting
Added more detail, visual cuing, etc.
because it's nice to be able to find

#### Added Printing of Date/Time of Run Start for
but could really have done better here by effective use of logging
Added more detail, visual cuing, etc.

#### Added forward kinematics for joint value verification (as recommended)

#### Modified script to run all test cases in a batch rather than manually specifying
would be nice to have the option to specify a test case, but it was more useful to me to get the whole rundown every time than to bother implementing that feature...

## Inverse Kinematics Steps
    1. Determine wrist center coordinates from target end effector position/orientation
    1. Calculate joint 1 angle (rotate arm's working plane to contain wc)
    1. Calculate cylindrical radius of wc from Z1
    1. 