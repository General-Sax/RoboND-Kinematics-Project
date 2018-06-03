# KR210 Serial Manipulator Kinematics
**Udacity Robotics Software Engineering Nanodegree Project 2**
*Joel Tiura*

***

The KUKA KR210 is an industrial robotic arm (serial manipulator) which presents an excellent testbed to explore inverse kinematics, due to its obliging geometry. It has a spherical wrist, an elegant design choice which  controlling the orientation of the end effector supported by a coplanar system links with    with 6 joints, also referred to as axes or degrees of freedom (DOF's). In this project, the objective was to implement

---

['consistent_error.png',
 'crashing_and_performance_info.png',
 'denavit-hartenberg-parameter-definitions-01-modified.png',
 'intercept_illustration_bkgrnd_crop.png',
 'nice_stacking.png',
 'rack_em_up.png',
 'simultaneous_solution_multiplicity_edit.png',
 'SSS_Triangle.png',
 'WATCH_FOR_THIS_CAREFULLY.png',
 'color_coded_links.jpg', 
 'dh_axes_assignment.jpg']


[//]: # (Image References)

[SSS_triangle_1]: writeup_graphics/SSS_Triangle.png
[simultaneous_solver_justification]: writeup_graphics/intercept_illustration_bkgrnd_crop.png
[simultaneous_solver_degeneracy]: writeup_graphics/simultaneous_solution_multiplicity_edit.png
[dh_param_definition]: writeup_graphics/denavit-hartenberg-parameter-definitions-01-modified.png
[color_coded_links]: writeup_graphics/color_coded_links.jpg
[dh_axes_assignment]: writeup_graphics/dh_axes_assignment.jpg
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
[complete_transform_unreadable]: math/hideous_T0_EE_bigger.png
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



***
## Introduction / Abstract


## Forward Kinematics Functionality as Design Prerequisite


## Forward Kinematics Tool



***
## Denavit-Hartenberg Parameterization
### URDF File Links
#### Link Lengths
#### Link Geometry
#### Link Origins
### URDF File Joints
#### Joint Types
#### Joint Coordinates
#### Joint Ranges

## Composing DH-Compatible Geometric Model
![annotated dh parameter illustration][dh_param_definition]

*O(i-1): link i-1 origin, O(i): link i origin, alpha(i-1): *


## DH Parameter Assignment
Before I started digging into the URDF file or doing any math, I had to get to know my robot.
Not pictured here is my extensive time playing with the forward kinematics rviz demo, and forming a
visual, spatial impression of the robot's geometry and articulation. I am a visual learner and is
one of my strongest methods, and good visual intuition is critical to any design process. and out how the system 

Before doing anything else, I pl


### Follow the Algorithm!
From the lesson, 
1. Label all joints from {1, 2, … , n}.
2. Label all links from {0, 1, …, n} starting with the fixed base link as 0.
3. Draw lines through all joints, defining the joint axes.
4. Assign the Z-axis of each frame to point along its joint axis. 
5. Identify the common normal between each frame Z_i-1 and frame Z_i.
6. The endpoints of "intermediate links" (i.e., not the base link or the end effector) are associated with two joint axes, {i} and {i+1}. For i from 1 to n-1, assign the X_i vector to be …
    - For skew axes, along the normal between Z_i and Z_i+1 and pointing from {i} to {i+1}.
    - For intersecting axes, normal to the plane containing Z_i and Z_i+1
    - For parallel or coincident axes, the assignment is arbitrary; look for ways to make other DH parameters equal to zero.
7. For the base link, always choose frame {0} to be coincident with frame {1} when the first joint variable (theta_1 or d_1) is equal to zero. This will guarantee that alpha_0 = a_0 = 0, and, if joint 1 is a revolute, d_1 = 0. If joint 1 is prismatic, then theta_1 = 0.
8. For the end effector frame, if joint n is revolute, choose X_n to be in the direction of X_n-1 when theta_n = 0 and the origin of frame {n} such that d_n = 0.


Well, in a nutshell, that's what I did.

First I layed out a few schematic sketches of the relative DH axes:

![relative DH axis schematic diagram][dh_axes_assignment]

Color-coding links to highlight joint positions/functions, which in turn helped my brain parse the ground-truth geometry.

![color coding the links to highlight joint positions/functions][color_coded_links]


The early phase involves a lot of scratch work exploring the problem space. Not all of it is worth explaining, but I'll include some for flavor:



but simultaneously, I was paying far more attention to the parameter definitions, and to the mathematical architecture of the transform that I was trying to encode.



### Leveraging Special Cases: Symmetries are Powerful
In the lesson, four special geometric cases were discussed which imply necessary constraints on the dh parameters of the transform. 
These relationships are intuitive; for each particular geometrie and its corresponding constraint(s), there is a verbal description that explains it:
- collinear lines: alpha = 0 and a = 0: 
"If two joint axes are collinear..."
    - "...the magnitude of the twist angle between them must be zero, since otherwise this would imply non-collinearity."
    - "...the magnitude of the link length between them must be zero, since this quantity represents a displacement orthogonal to the collinear axis."
- parallel lines: alpha = 0 and a != 0: 
"If two joint axes are parallel (but *not* collinear)..."
    - "...the magnitude of the twist angle between them must be zero, since otherwise this would imply they were not parallel."
    - "...the magnitude of the link length between them must be nonzero, since this quantity represents the necessarily nonzero minimum separation between non-collinear axes."
- intersecting lines: alpha != 0 and a = 0:
"If two joint axes intersect..."
    - "...the magnitude of the twist angle between them must be nonzero, since otherwise this would imply they were parallel or collinear."
    - "...the magnitude of the link length between them must be zero, since this quantity represents the necessarily zero minimum separation between intersecting axes."
- if the common normal intersects Z_i at the origin of frame i, then d_i = 0



When I first began to analyze the inverse kinematics of the KR210 manipulator, I wanted to break down its geometry in 
detail, so I could rebuild the problem from the base link up. And the base link seemed like the right place to start, 
since the whole manipulator and all of the non-static reference frames pivot around it. The origin of the base_link 
coordinate system naturally lies in the axis of rotation for joint_1.

By comparing the slider bounds of the joint_state_publisher to the urdf, I observe that they are accurate 
representations of the urdf joint limits, as expected. However, the slider values are presented in radians rather than
degrees, as in the URDF file.

### URDF Arm Joint Information

```xml
  <!-- joints -->
  <joint name="fixed_base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
  <joint name="joint_1" type="revolute">
    <origin xyz="0 0 0.33" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="link_1"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-185*deg}" upper="${185*deg}" effort="300" velocity="${123*deg}"/>
  </joint>
  <joint name="joint_2" type="revolute">
    <origin xyz="0.35 0 0.42" rpy="0 0 0"/>
    <parent link="link_1"/>
    <child link="link_2"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-45*deg}" upper="${85*deg}" effort="300" velocity="${115*deg}"/>
  </joint>
  <joint name="joint_3" type="revolute">
    <origin xyz="0 0 1.25" rpy="0 0 0"/>
    <parent link="link_2"/>
    <child link="link_3"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-210*deg}" upper="${(155-90)*deg}" effort="300" velocity="${112*deg}"/>
  </joint>
  <joint name="joint_4" type="revolute">
    <origin xyz="0.96 0 -0.054" rpy="0 0 0"/>
    <parent link="link_3"/>
    <child link="link_4"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-350*deg}" upper="${350*deg}" effort="300" velocity="${179*deg}"/>
  </joint>
  <joint name="joint_5" type="revolute">
    <origin xyz="0.54 0 0" rpy="0 0 0"/>
    <parent link="link_4"/>
    <child link="link_5"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-125*deg}" upper="${125*deg}" effort="300" velocity="${172*deg}"/>
  </joint>
  <joint name="joint_6" type="revolute">
    <origin xyz="0.193 0 0" rpy="0 0 0"/>
    <parent link="link_5"/>
    <child link="link_6"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-350*deg}" upper="${350*deg}" effort="300" velocity="${219*deg}"/>
  </joint>
```

### URDF Gripper Joint Information

```xml
  <joint name="right_gripper_finger_joint" type="prismatic">
    <origin rpy="0 0 0" xyz="0.15 -0.0725 0" />
    <parent link="gripper_link" />
    <child link="right_gripper_finger_link" />
    <axis xyz="0 1 0" />
    <limit effort="100" lower="-0.01" upper="0.06" velocity="0.05" />
    <dynamics damping="0.7" />
  </joint>
  <joint name="left_gripper_finger_joint" type="prismatic">
    <origin rpy="0 0 0" xyz="0.15 0.0725 0" />
    <parent link="gripper_link" />
    <child link="left_gripper_finger_link" />
    <axis xyz="0 -1 0" />
    <limit effort="100" lower="-0.01" upper="0.06" velocity="0.05" />
    <dynamics damping="0.7" />
  </joint>
  <joint name="gripper_joint" type="fixed">
    <parent link="link_6"/>
    <child link="gripper_link"/>
    <origin xyz="0.11 0 0" rpy="0 0 0"/><!--0.087-->
    <axis xyz="0 1 0" />
  </joint>
```


Joint | URDF Origin ("x y z") | URDF Type
--- | --- | --- 
fixed_base_joint | "0 0 0" | "fixed"
joint_1 | "0 0 0.33" | "revolute"
joint_2 | "0.35 0 0.42" | "revolute"
joint_3 | "0 0 1.25" | "revolute"
joint_4 | "0.96 0 -0.054" | "revolute"
joint_5 | "0.54 0 0" | "revolute"
joint_6 | "0.193 0 0" | "revolute"
gripper_joint | "0.11 0 0" | "fixed"
left_gripper_finger_joint | "0.15 0.0725 0" | "prismatic"
right_gripper_finger_joint | "0.15 -0.0725 0" | "prismatic" 







### DH Parameter Table

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 |   0.35 |      0 | q2 - pi/2
2->3 | 0 |   1.25 |      0 | q3
3->4 | - pi/2 | -0.054 |   1.50 | q4
4->5 | pi/2 |      0 |      0 | q5
5->6 | - pi/2 |      0 |      0 | q6
6->EE | 0 |      0 |  0.303 | 0


NOTE: the value of d7 = 0.303 is the distance to the gripper origin, not the grasping center of the end effector!
The 0.303 value is given in the problem description and intro, but seems to me a less natural choice than 0.453,
which is the actual distance between the target point and the wrist center.


***

## Decoupling and Solution Strategy

While the position and orientation of the wrist center may be decoupled problems, they have a subtle interdependence.

I am a very spatial and visual thinker, and I often benefit profoundly from attempting to break down a problem into stages or steps with clear geometric analogs. I appreciate how powerful the matrix representations are for calculating these transformations efficiently, and I am happy to employ them as a computational shorthand, but when I'm learning a to solve a new problem I prefer to map it out with geometric or other graphical models first. I want to find useful symmetries, make diagrams to illustrate relationships, and generally let my visual imagination explore the shape of the solution.


## Inverse Kinematics Steps
1. Determine wrist center coordinates from target end effector position/orientation
1. Calculate joint 1 angle (rotate arm's working plane to contain wc)
1. Calculate cylindrical radius of wc from Z1
1. 

## Joints 1, 2, and 3: Inverse Position Kinematics
### Boundary Conditions: The Positioning Plane

While scrutinizing the coordinates of the link reference frames in RViz and comparing them with the joint origins in 
the URDF file, I immediately noticed that (when joint variables are zeroed) the entire manipulator is essentially 
embedded in a plane. This plane is coincident with the xz-plane in the zeroed pose, and it rotates about the fixed
Z_0 axis. The plane's rotation angle none other than theta1, the dh parameter corresponding to joint 1. 

Since joints 0-5 are confined to this same vertical plane, and these are sufficient to solve the inverse positioning problem, 
I will call said plane the "positioning plane."

Joint limits on joint 1 give it a symmetrical CW/CCW range of motion, capable of rotation left or right by 185 degrees 
from zero (aligned with the base x-axis). This allows the robot to rotate through a maximum of 370 degrees in a single 
turn, giving a nice cushion of flexibility to a fully circular work envelope.

The wrist center must lie in this plane, so if the wrist center's position can be determined from information in the pose 
request, it should be straightforward to work back to theta1.

The positioning plane is a symmetry of this problem with practical use, as it allows us to factor the inverse positioning 
problem further into two independent components: theta1, as constrained by the (x, y) coordinates of the wrist center, 
and theta2 & theta3, which are jointly determined by the z coordinate of the wrist center and its radial distance from the
base link within the positioning plane.

Since the DH Z-axes of joints 2 and 3 are orthogonal to said plane, they are only free to move the arm within this plane. 

Furthermore, since joint 4 is a coaxial revolute, it cannot move joint 5 out of the plane either - it can only indirectly 
alter the location of the end effector by rotating the direction at which link 5 may leave this plane.


### Boundary Conditions: Total Constraint of the Wrist Center

For any viable pose request, there is a specific point target which serves as a boundary condition for an IK solution. 
By itself, however, that position vector wouldn't be a very rigid constraint; the IK process would need to somehow 
select a valid orientation of approach. Since a pose also provides an orientation quaternion, it solves this problem for
us. 

This orientation directly determines the location of the wrist center; the distance between the wrist center and the 
end effector is a constant 0.303m, thus, there is a unique wrist center displaced along the orientation vector by 
precisely this distance.

As I have established, all joint origins from the base link up to (and including) the wrist center are coplanar.
While joint_1 establishes the angle of the positioning plane about the Z_0/Z_1 axis, joint_2 and joint_3 together establish 
2-dimensional freedom within this vertical plane. One of those dimensions is the unaltered world z-axis dimension. 
The other is a radial, cylindrical coordinate, which I have called 's' in accordance with 
If we project the wrist center's position vector onto the xy-plane representing the ground, theta1 is the angle between
the world-frame x unit vector and this projection.

*![][picture goes here]*


 
Once the uniquely-determined wrist center is extracted from the pose request, we can establish theta1: the orientation 
of the positioning plane. 
(I have chosen to refer to this as the 'positioning plane') lie in the same plane;

The assertion that the end effector must be at this specific orientation may be more rigid than necessary to ensure smooth operation, but some specific orientation must be chosen to constrain the problem, and aligning the end effector with a surface normal vector on the shelf's openings should go a long way towards maximizing the accessible volume of any shelf (if not perfectly optimizing access, which I believe it will, in fact, do - but I haven't tried to prove this).

Furthermore, with our end effector aligned along the depth dimension of the shelves, it is natural enough to specify a location on the opening's 'surface' at which to engage the automatic reaching and grasping routines.

Since joints 0-5 are confined to the same vertical plane, and these are sufficient to solve the inverse positioning problem, I will call said plane the 'positioning plane.' With the wrist center determined, this plane's orientation about the Z1 axis is fully constrained. This orientation is determined by the value of theta_1, which is calculated below:



### Joints 2 & 3: The SSS Triangle Method
There is a slight hiccup here; a slight offset between the axes of joints 3 and 4. This of 
Since the articulation of joint 4 is revolute about the axis between itself and joint 5 (a.k.a. the wrist center), and since both joint origins lie in plane with joint 3 AND joint 2, finding ain plane with the wrist center - **BUT ONLY APPROXIMATELY; SEE CORRECTION!** - the 
The origins of joints 2, 3, and 5 all lie in the
Let a, b, c represent sides of the triangle 


![alt text][SSS_triangle_1]



### Explored Alternative Method for Joints 2 & 3: Simultaneous Circle Solver

The SSS Triangle method is elegant, and performs well, generally, but it only finds one of the two possible solutions for most configurations.

Another solution model which I investigated essentially amounts to solving for the the intersections of the sets of allowed locations for joint 3 given fixed locations for joint 2 and 5.


![alt text][simultaneous_solver_justification]

![alt text][simultaneous_solver_degeneracy]


Before enforcing the URDF model's joint range limits, joint 2 is free to move joint 3 to any location on a circle of radius 1.25m about its Z-axis, the length of link 2. This circle lies in the plane containing the origins of joints 0-5, the 'positioning plane.' This circle is the locus of all points 

In an analogous (but inverted) manner, joint 5 is the center may be accessed by joint 3 from any point

Joint 2 is taken to be the natural origin for solving this step in the problem.
For the sake of cognitive simplicity, we will consider the next steps as purely 2-dimensional.
In that frame of reference, Joint 5 is located at (s5, z5)



Regardless of our choice of method, we will need the following values to be loaded:

```python
# radius_j2_j3 = dh_parameter[a2]
radius_j2_j3 = dh_parameter[a2]

# The magnitude of the vector pointing from joint_3 to joint_5, although our DH parameter table
# files this location under the transformation to joint_4 coordinates, since joints 4 and up 
# share an origin at the wrist center, coincident with joint 5.

# radius_j5_j3 = sqrt(dh_parameter[a3]**2 + dh_parameter[d4]**2)
# displacements in the positioning plane between the wrist center and joint_2.
# delta_s = wrist_s - dh_parameter[a1] # dh_parameter[a1] = 0.35 (meters)
# delta_z = wrist_z - dh_parameter[d1] # dh_parameter[d1] = 0.75 (meters)

# radius_j5_j3, delta_s, delta_z
```






The differences between the coordinates of joint_2 and joint_5 are important parameters for solving the SSS triangle, ABC, described above. These differences are also the x and y coordinates of the wrist center in the joint 2 reference frame. The values are denoted here as delta_s and delta_z.

With the arm rotated into the positioning plane, the cylindrical radius to joint_2 is a constant 0.35m along s.
Similarly, joint_2 is always a constant 0.75m above the base link's z=0 plane, regardless of any other parameters.

Together, this means that joint_2 traverses a circle of radius 0.35m, fixed at 0.75m off the ground plane.
Setting theta_1 and thus the orientation of s effectively selects a point for j2 on this circle, and that is the origin for the coordinates of the SSS Triangle method.



the x-axis of the joint_1 reference frame, called here 's.' The s-axis is determined by the wrist center: it is parallel to the projection of a vector from the base_link origin to the wrist center. Referring to this dimension or its values as 's' may be unnecessary complication, but it emphasizes the profound cylindrical symmetry of manipulator configurations.

Determining theta_1 doesn't simply move us closer to solving for all the joint angles; it permits us to rotate the coordinate system the appropriate amount to locate it into the sz-plane required by the wrist center.



***

## Joints 4, 5, and 6: Inverse Orientation Kinematics

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

Joint Range Enforcement
Before going forward, we should vet any and all solutions to ensure that they are viable commands for the robot. trying to choose between two solutions, or for that matter, accepting the miraculous appearance of a unique solution, First, we must resolve the value of theta_3. This must be calculated from gamma_5 and geometric methods.

Discriminating Between Multiple Viable Solutions
From a control and integrity standpoint, it makes sense to keep the robot's center of mass as near to the Z1 axis as possible, which could be calculated more delicately using the inertial information in the URDF file. This would be a bit of an undertaking, to say the least.

Another general consideration is similarity to the current pose. This is much easier to assess.

Joints 4-6: The Orientation Problem
The process of conforming the wrist links onto the now-constrained arm links breaks down into a few separate elements.

- Account for the slight misalignment between j3-j5 axis and rotation axis of joint_4
- Find theta_4 such that joint_5's DoF can access the target point.
- Find theta_6 such that the end effector is correctly oriented relative to the target.
We do, however, have some parts of this process partially complete! The angles of joints 4 and 5 are ,


Initially we have all side lengths, so let's prepare a dict:

```python
# Let A, B, C name the vertices of the triangle described by the origins of joints 2, 3, and 4.
# let the symbols A and B represent the interior angles of triangle ABC at their synonymous vertices.
# A, B, C = symbols('A B C')
# C is not strictly needed, but it may be useful for appraising whether a given pose is possible.
# If C comes back imaginary, this would indicate an unviable solution.

# Note that vertex angle C is not given a symbol; it simply isn't necessary, as it wouldn't be used in any later step.
# A, B = symbols('A B')

# let a, b, c represent side lengths of ABC, named conventionally (opposite their angles)
# a, b, c = symbols('a b c')

# Here, A is the vertex located at joint_2, B at joint_3, and C at joint_5 (shared origin with joints 4-6)
# ...but only one leg has a known location/orientation: side b, which is coincident with a vector pointing from j2 to j5.
# This vector ( we'll just call it "b_vector") ultimately determines the existence and nature of any solutions.
# The side length b is the magnitude of b_vector:

sides = {}
b = sqrt(delta_s**2 + delta_z**2)

# sides[b] = sqrt(delta_s**2 + delta_z**2).evalf(subs=t)
sides[b] = sqrt(delta_s**2 + delta_z**2)

# a and c are fixed distances, a the distance between joints 3 and 5 (=1.25m) and b between 2 and 3.
# These are both defined above, as radius_j5_j3 and radius_j2_j3 respectively
sides[a] = radius_j5_j3
sides[c] = dh_parameter[a2]

sides


# Solving SSS Triangles is fun and simple!
A = acos((b**2 + c**2 - a**2)/(2 * b * c)).subs(sides)
B = acos((a**2 + c**2 - b**2)/(2 * a * c)).subs(sides)
C = acos((a**2 + b**2 - c**2)/(2 * a * b)).subs(sides)
A, B

A = A.evalf(subs=t)
B = B.evalf(subs=t)
C = C.evalf(subs=t)
```

Calculate numerical error between expected sum of interior angles and the sum of our results.
num_error = N(pi - (A + B + C))

If A, B, or C are imaginary or complex, there is no physical solution.
If num_error is not approximately zero, then there is likely an error upstream. 
A, B, C, num_error


The rotation of b_vector about Z2 (common normal of s and Z1) sets the orientation of the triangle ABC, 
which in turn determines various angular terms in the formulas for theta_2 and theta_3.
One such term is wc_pitch, the angle between b_vector and the s-axis:
wc_pitch = atan2(delta_z, delta_s)
wc_pitch, wc_pitch.evalf(subs=t)


Since angles A, B, C are positive definite, and wc_pitch is measured CCW from the horizontal (opposite sign to theta_2),
the angle (B + wc_pitch) is effectively wc_pitch, with some as-yet-undetermined positive phase shift.

Theta_2 is initially offset by a phase of 90 degrees in the positive gamma direction

gamma_5 is an intermediate angle in this calculation.
For now, it represents the deflection of the joint 5 radiant from the horizontal,
as shown in illustration. 
Behaves identically to theta_2 for ease of explanation. 
This is hard to visualize, add illustration!





### Generate DH FK Transform Matrices from Table

### General DH Transform Matrices, Associated Utilities

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
# alpha, a, d, q = twist_angle, link_length, link_offset, joint_angle
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

### Create Individual DH Transform Matrices

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

### Complete DH Transform Matrix

To compute the complete transform over the arm, we compose the individual matrices in order.

```python
T0_EE = sp.simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_7)
```

This matrix is HUGE. It's a mess. From the moment I saw this monstrosity, I fully intended to keep it out of any live server code if I could help it - and succeeded in that effort.

It probably isn't a good use of anyone's time to try making sense of it by visual inspection, but the rubric said to include it, so here it is:

**T0_EE**

![T0_EE][complete_transform_unreadable]


```python
Matrix([
[((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) - (-sin(q1)*cos(q4) + sin(q4)*sin(q2 + q3)*cos(q1))*sin(q6), -((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*cos(q6), -(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3), -0.303*sin(q1)*sin(q4)*sin(q5) + 1.25*sin(q2)*cos(q1) - 0.303*sin(q5)*sin(q2 + q3)*cos(q1)*cos(q4) - 0.054*sin(q2 + q3)*cos(q1) + 0.303*cos(q1)*cos(q5)*cos(q2 + q3) + 1.5*cos(q1)*cos(q2 + q3) + 0.35*cos(q1)],
[ ((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*sin(q6), -((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*cos(q6), -(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3),  1.25*sin(q1)*sin(q2) - 0.303*sin(q1)*sin(q5)*sin(q2 + q3)*cos(q4) - 0.054*sin(q1)*sin(q2 + q3) + 0.303*sin(q1)*cos(q5)*cos(q2 + q3) + 1.5*sin(q1)*cos(q2 + q3) + 0.35*sin(q1) + 0.303*sin(q4)*sin(q5)*cos(q1)],
[                                                                -(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) - sin(q4)*sin(q6)*cos(q2 + q3),                                                                  (sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) - sin(q4)*cos(q6)*cos(q2 + q3),                                     -sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5),                                                                                 -0.303*sin(q5)*cos(q4)*cos(q2 + q3) - 0.303*sin(q2 + q3)*cos(q5) - 1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75],
[                                                                                                                                                            0,                                                                                                                                                             0,                                                                                        0,                                                                                                                                                                                                              1]])
```





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

This result LOOKS pretty, and is almost numerically equivalent to the previous method.
To confirm that it was actually performing the correct transformations, I applied it to some unit vectors.

Given this matrix for R_corr, and the following elementary basis vector definitions:

```python
R_corr = Matrix([[ 0,  0,  1],
                 [ 0, -1,  0],
                 [ 1,  0,  0]])

# Elementary basis vectors x, y, z (non-homogeneous representation)
e_x = column((1, 0, 0)) # == Matrix([[1], [0], [0]])
e_y = column((0, 1, 0)) # == Matrix([[0], [1], [0]])
e_z = column((0, 0, 1)) # == Matrix([[0], [0], [1]])
```

The following desired equalities hold:

```python
R_corr * e_x == Matrix([[0], [ 0], [1]]) ==  e_z
R_corr * e_y == Matrix([[0], [-1], [0]]) == -e_y
R_corr * e_z == Matrix([[1], [ 0], [0]]) ==  e_x
```

So it is certainly the correct transform.


***

***

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
In my preparatory analysis and early experiments developing a set of inverse kinematics equations, I noticed that most 
of the problem was completely general and invariant over all poses, and only needed to be derived once in a theoretical 
context. That context was using SymPy in Jupyter notebooks, where I could leverage the power of the notebooks' 
cell-based execution to streamline exploratory work. 

SymPy was a delight to use, and helped me make some clever optimizations. By using its robust computer algebra system 
to compute exact symbolic relations, I was able to derive a coordinate system correction matrix which was not to make 
performance improvements in code that di, and using

As I started implementing a solution, I found that I didn't like the suggested organization of the handle_calculate_IK 
function. It struck me as inelegant to be programmatically constructing and manipulating invariant constructs solution 
inside the working server code. and as such, I sought to  as many features underlying problem structure as possible. 
much of the process of deriving the components of a working solution was  to distill the inverse kinematics problem 
down to its essence, and do my best to refine the IK_server loop into a representation of a solution to that problem.
weed out redundant calculations, the minimum number of 'moving parts'

### Function: handle_calculate_IK

while the wrist center's location is not specified explicitly, it is already determined
*implicitly* by the specified end effector orientation.
     


### Function: construct_R_EE
`r_EE` is the general symbolic rotation matrix for the end effector orientation, defined outside loop.
r_EE represents this matrix for a given pose, substituting in the roll, pitch, and yaw variables
extracted from the pose orientation quaternion.
r_EE is the rotational component of the transformation matrix between the base coord
roll, pitch, and yaw specify end-effector orientation relative to the base coordinate system,
but only when the corresponding sequence of rotations is applied with respect to static coordinates, and
in an XYZ axis order.

This sequence and convention is not a coincidence; it's the is the default behavior of the
tf.transformations.euler_from_quaternion() function.

```python
(roll, pitch, yaw) = tf.transformations.euler_from_quaternion([
    req.poses[x].orientation.x,
    req.poses[x].orientation.y,
    req.poses[x].orientation.z,
    req.poses[x].orientation.w
    ])
```

These values of roll, pitch, and yaw are calculated using the 'sxyz' convention defined in the tf library,
where *'s'* denotes static frame, and *'xyz'* gives an axis sequence for the rotations.

```python
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

Includes coordinate correction implicitly by having multiplied in the following matrix during the derivation of the 
complete transform:



### Error Calculation and Monitoring

```python
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
```python
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


## IK_debug.py
### Direct `import handle_ik_request` from IK_server.py
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
