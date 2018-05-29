import os
import time
#
# import rospy
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import numpy as np
import matplotlib.pyplot as plt

sin = np.sin
cos = np.cos
sqrt = np.sqrt


class ErrorHandler:
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
		
		return means, maxes
	
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
		
		output_fig = plt.figure(figsize=ErrorHandler.figure_dims)
		
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

#
# def IK_server():
# 	# initialize node and declare calculate_ik service
#
# 	rospy.init_node('error_handler', anonymous=True)
# 	rospy.Subscriber('pose_angles', JointTrajectoryPoint, handle_calculate_IK)
# 	print
# 	"Ready to receive an IK request\n"
# 	rospy.spin()
#
#
# if __name__ == "__main__":
# 	IK_server()
# # monitor = ErrorHandler(show_figures=True, save_figures=False)
