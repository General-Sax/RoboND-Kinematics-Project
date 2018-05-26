'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

from pprint import pprint
from time import time, strftime

from sympy import *
from IK_server import handle_calculate_IK


# single pose test cases
test_cases = {
    0: [
        [[2.1529, 0, 1.9465], [0.0, -0.0, 0.0, 1.0]], # Inputs ([EE position], [EE orientation as quaternions])
        [1.85, 0, 1.946], # Resulting WC location
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], # Resulting Theta Values
    1: [
        [[2.16135, -1.42635,1.55109], [0.708611,0.186356, -0.157931,0.661967]],
        [1.89451, -1.44302, 1.69366],
        [-0.65,0.45,-0.36,0.95,0.79,0.49]],
    2: [
        [[-0.56754,0.93663,3.0038], [0.62073, 0.48318,0.38759,0.480629]],
        [-0.638,0.64198,2.9988],
        [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
    3: [
        [[-1.3863,0.02074,0.90986], [0.01735,-0.2179,0.9025,0.371016]],
        [-1.1669,-0.17989,0.85137],
        [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
}


def check_joint_limits(theta_list, stdout=False, raise_exception=False):
	'''
	
	:param theta_list:
	:param stdout:
	:param raise_exception:
	:return:
	'''
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
			elif stdout:
				pprint(report)
			else:
				violations.append(i)
	return violations
 

########################################################################################
# Define dummy objects to mimick the structure of an actual pose request
class Position:
	def __init__(self, EE_pos):
		self.x = EE_pos[0]
		self.y = EE_pos[1]
		self.z = EE_pos[2]

class Orientation:
	def __init__(self, EE_ori):
		self.x = EE_ori[0]
		self.y = EE_ori[1]
		self.z = EE_ori[2]
		self.w = EE_ori[3]

class Combine:
	def __init__(self, position, orientation):
		self.position = position
		self.orientation = orientation

class Pose:
	def __init__(self, comb):
		self.poses = [comb]

########################################################################################

def debug_code(test_case):
	position = Position(test_case[0][0])
	orientation = Orientation(test_case[0][1])
	comb = Combine(position, orientation)
	req = Pose(comb)
 
	start_time = time()
	
	########################################################################################
	# IK function 'handle_calculate_IK' imported from IK_server.py directly
	ik_angles = handle_calculate_IK(req)[0].positions

	# immediately report time-to-result
	print ("Total run time to calculate joint angles from pose is %04.8f seconds" % (time() - start_time))

	########################################################################################
	## Error analysis
	# Ensure that calculated angles are within the ranges specified in the URDF file
	violations = check_joint_limits(ik_angles, stdout=False, raise_exception=False)
	for report in violations:
		print report

	# Unpack calculated angles
	theta1, theta2, theta3, theta4, theta5, theta6 = ik_angles

	## Forward kinematics implementation to verify results:
	q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')  # theta angles

	# pre-derived wrist center position vector in terms of q1, q2, and q3 for comparison with
	fk_wrist = Matrix([
		[(1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*cos(q1)],
		[(1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*sin(q1)],
		[         -1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75],
    ])
	   
	fk_wrist = fk_wrist.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

	# fk_EE is the additive, translational component of the total homogeneous transform between the base
    # and the end effector; this column vector representation has frame error correction baked in!
	fk_EE = Matrix([
		[-0.303*(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*cos(q1) + 0.303*cos(q1)*cos(q5)*cos(q2 + q3)],
		[-0.303*(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*sin(q1) + 0.303*sin(q1)*cos(q5)*cos(q2 + q3)],
		[-0.303*sin(q5)*cos(q4)*cos(q2 + q3) - 0.303*sin(q2 + q3)*cos(q5) - 1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75]])
	fk_EE = fk_EE.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})

	########################################################################################
	## For error analysis, the following names are given to the locations of the wrist center (wc) and
	## end effector (ee) in the format of [x, y, z]
	calculated_wc = list(fk_wrist) # <--- The calculated wc coordinates
	# your_ee = [fk_EE[0], fk_EE[1], fk_EE[2]] # <--- Load your calculated end effector value from your forward kinematics
	calculated_ee = list(fk_EE)
	########################################################################################
	## Error analysis
	# calculate theta errors
	t_1_e = abs(theta1 - test_case[2][0])
	t_2_e = abs(theta2 - test_case[2][1])
	t_3_e = abs(theta3 - test_case[2][2])
	t_4_e = abs(theta4 - test_case[2][3])
	t_5_e = abs(theta5 - test_case[2][4])
	t_6_e = abs(theta6 - test_case[2][5])
	
	print ("theta1 error: {:.5f};  ( Calculated / Actual ): ( {:6.3f} / {:6.3f} )".format(t_1_e, theta1, test_case[2][0]))
	print ("theta2 error: {:.5f};  ( Calculated / Actual ): ( {:6.3f} / {:6.3f} )".format(t_2_e, theta2, test_case[2][1]))
	print ("theta3 error: {:.5f};  ( Calculated / Actual ): ( {:6.3f} / {:6.3f} )".format(t_3_e, theta3, test_case[2][2]))
	print ("theta4 error: {:.5f};  ( Calculated / Actual ): ( {:6.3f} / {:6.3f} )".format(t_4_e, theta4, test_case[2][3]))
	print ("theta5 error: {:.5f};  ( Calculated / Actual ): ( {:6.3f} / {:6.3f} )".format(t_5_e, theta5, test_case[2][4]))
	print ("theta6 error: {:.5f};  ( Calculated / Actual ): ( {:6.3f} / {:6.3f} )".format(t_6_e, theta6, test_case[2][5]))

	# Find FK EE error
	if not(sum(calculated_ee) == 3):
		x_err = abs(calculated_ee[0]-test_case[0][0][0])
		y_err = abs(calculated_ee[1]-test_case[0][0][1])
		z_err = abs(calculated_ee[2]-test_case[0][0][2])
		ee_offset = sqrt(x_err**2 + y_err**2 + z_err**2)
		print ("\nEnd effector error for x position is: %04.8f" % x_err)
		print ("End effector error for y position is: %04.8f" % y_err)
		print ("End effector error for z position is: %04.8f" % z_err)
		print ("Overall end effector offset is: %04.8f meters" % ee_offset)

	# Find WC error
	if not(sum(calculated_wc)==3):
		wc_x_e = abs(calculated_wc[0] - test_case[1][0])
		wc_y_e = abs(calculated_wc[1] - test_case[1][1])
		wc_z_e = abs(calculated_wc[2] - test_case[1][2])
		wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
		print ("\nWrist error for x position is: %04.8f" % wc_x_e)
		print ("Wrist error for y position is: %04.8f" % wc_y_e)
		print ("Wrist error for z position is: %04.8f" % wc_z_e)
		print ("Overall wrist offset is: %04.8f meters" % wc_offset)


if __name__ == "__main__":
	print "\n" + (40 * '*v') + " BEGIN"
	print "Test initializing at time: {}".format(strftime('%c'))
	# Cycle through all provided debug requests
	for number in sorted(test_cases.keys()):
		if test_cases[number]:
			print "\n" + 40 * "- " + str(number)
			print "Running test case {}...".format(number)
			debug_code(test_cases[number])
	print "\nTest completed at time: {}".format(strftime('%H:%M:%S'))
	print 40 * '*^' + " END \n"
