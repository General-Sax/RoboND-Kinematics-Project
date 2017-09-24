from sympy import *
from time import time, strftime
from mpmath import radians
import tf
import numpy as np

from IK_server import handle_calculate_IK


'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

# single pose test cases
test_cases = {0:[[[2.1529, 0, 1.9465],
                  [0.0, -0.0, 0.0, 1.0]],
                  [1.85, 0, 1.946],
                  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]],
              1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()

    ############################################################################
    # IK function 'handle_calculate_IK' imported from

    angles = tuple(handle_calculate_IK(req, debug_return=True)[0].positions)
    theta1, theta2, theta3, theta4, theta5, theta6 = angles

    jointLimits = {1: (-3.2288591161895095, 3.2288591161895095),
                   2: (-0.7853981633974483, 1.4835298641951802),
                   3: (-3.6651914291880923, 1.1344640137963142),
                   4: (-6.1086523819801535, 6.1086523819801535),
                   5: (-2.181661564992912, 2.181661564992912),
                   6: (-6.1086523819801535, 6.1086523819801535)}

    for i in range(1, 7):
        if angles[i-1] > jointLimits[i][0] and angles[i-1] < jointLimits[i][1]:
            pass
        else:
            print "\n*** Angle theta"+str(i)+" outside joint range! ***"
            print "Range:", jointLimits[i], "Theta:", angles[i-1], "\n"

    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')  # theta angles

    fk_wrist = Matrix([
        [(1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*cos(q1)],
        [(1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*sin(q1)],
        [         -1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75]])
    fk_wrist = fk_wrist.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

    # fk_EE is the translation vector from the total homogeneous transform
    # between the base and the end effector; this representation has frame error
    # correction baked in!
    fk_EE = Matrix([
        [-0.303*(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*cos(q1) + 0.303*cos(q1)*cos(q5)*cos(q2 + q3)],
        [-0.303*(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*sin(q1) + 0.303*sin(q1)*cos(q5)*cos(q2 + q3)],
        [-0.303*sin(q5)*cos(q4)*cos(q2 + q3) - 0.303*sin(q2 + q3)*cos(q5) - 1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75]])

    end_effector = fk_EE.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = list(fk_wrist) # <--- Load your calculated WC values in this array
    # your_ee = [fk_EE[0], fk_EE[1], fk_EE[2]] # <--- Load your calculated end effector value from your forward kinematics
    your_ee = list(end_effector)
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1 - test_case[2][0])
    t_2_e = abs(theta2 - test_case[2][1])
    t_3_e = abs(theta3 - test_case[2][2])
    t_4_e = abs(theta4 - test_case[2][3])
    t_5_e = abs(theta5 - test_case[2][4])
    t_6_e = abs(theta6 - test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print "Theta 1 Calculated : %04.2f" % theta1, " Actual : ", test_case[2][0]
    print "Theta 2 Calculated : %04.3f" % theta2, " Actual : ", test_case[2][1]
    print "Theta 3 Calculated : %04.2f" % theta3, " Actual : ", test_case[2][2]
    print "Theta 4 Calculated : %04.2f" % theta4, " Actual : ", test_case[2][3]
    print "Theta 5 Calculated : %04.2f" % theta5, " Actual : ", test_case[2][4]
    print "Theta 6 Calculated : %04.2f" % theta6, " Actual : ", test_case[2][5]

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)


if __name__ == "__main__":
    # print "\n\nTest initializing at time: {}".format(strftime('%H:%M:%S'))
    print "\n"+(40 * '*v')+" BEGIN"
    print "Test initializing at time: {}".format(strftime('%c'))
    # Change test case number for different scenarios
    for number in sorted(test_cases.keys()):
        if test_cases[number]:
            print "\n" + 40 * "- "+ str(number)
            print "Running test case {}...".format(number)
            test_code(test_cases[number])
    print "\nTest completed at time: {}".format(strftime('%H:%M:%S'))
    print 40 * '*^' + " END \n"

# if __name__ == "__main__":
#     # Change test case number for different scenarios
#     test_case_number = 1

#     test_code(test_cases[test_case_number])
