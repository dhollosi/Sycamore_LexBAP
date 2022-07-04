import numpy as np
from copy import deepcopy
import os
from matplotlib import pyplot as plt
# from choirbot_interfaces.msg import PositionTask, PositionTaskArray
from scipy.optimize import linear_sum_assignment


class ChoiRbotLSAP:

	def __init__(self, task_data , *, agent_dim = int):

		self.agent_dim = agent_dim
		self.task_dim = len(task_data[0])
		if self.agent_dim != self.task_dim:
			raise TypeError('Warning: task dimension does not match agent dimension')

		self.agent_pos = None
		self.task_pos = task_data
		self.init_agent_pos = None
		self.init_task_pos = None

		self.x_agent_array = []
		self.y_agent_array = []

		self.lower_bound = -3
		self.upper_bound = 3

		self.cost = None
		self.init_cost = None


	def get_agent_pos_from_dict(self, global_dict):

		agent_pos_x = np.ndarray(shape=(self.agent_dim,1))
		agent_pos_y = np.ndarray(shape=(self.agent_dim,1))
		for i in range(0,self.agent_dim):
			current_pose_x = global_dict['agent_{}'.format(i)]['pose_x']
			current_pose_y = global_dict['agent_{}'.format(i)]['pose_y']
			agent_pos_x[i] = current_pose_x
			agent_pos_y[i] = current_pose_y

		self.agent_pos = [agent_pos_x, agent_pos_y]


	def plot_solution(self, counter):
		x_match_lex, y_match_lex = self.get_agent_to_task_match()
		fig, ax = plt.subplots()

		for i in range(0, self.task_dim):
			x_new = [x_match_lex[0][i], x_match_lex[1][i]]
			y_new = [y_match_lex[0][i], y_match_lex[1][i]]
			plt.plot(x_new, y_new, marker='o', markevery=[0])

		plt.legend()
		plt.grid()
		ax.set(xlim=(2 * self.lower_bound, 2 * self.upper_bound), ylim=(2 * self.lower_bound, 2 * self.upper_bound))
		ax.set_aspect('equal')
		plt.xlabel("X-distance [m]")
		plt.ylabel("Y-distance [m]")
		plt.title('Lexicographic BAP')

		save_dir = os.path.join('/home/dimitri/Documents/EPFL/Courses/Sycamore/Analysis/LSAP/Assignments')
		my_plot = 'plot_0{}'.format(counter)
		plt.savefig(os.path.join(save_dir, my_plot))


	def plot_solution_history(self, counter, timestep):
		x_match_lex, y_match_lex = self.get_agent_to_task_match()
		fig, ax = plt.subplots()
		timestep_list = []

		for j in range(0, timestep):
			timestep_list.append(j)

		current_x_history = []
		current_y_history = []

		for i in range(0, self.agent_dim):
			for j in range(0, len(timestep_list)):
				current_x_history.append(self.x_agent_array[j][i])
				current_y_history.append(self.y_agent_array[j][i])

			x_new = [x_match_lex[0][i], x_match_lex[1][i]]
			y_new = [y_match_lex[0][i], y_match_lex[1][i]]
			p = plt.plot(x_new, y_new, marker='o', markevery=[0])
			plt.plot(current_x_history, current_y_history, color=p[0].get_color(), linestyle='dotted')

			current_x_history = []
			current_y_history = []

		plt.legend()
		plt.grid()
		ax.set(xlim=(2 * self.lower_bound, 2 * self.upper_bound), ylim=(2 * self.lower_bound, 2 * self.upper_bound))
		ax.set_aspect('equal')
		plt.xlabel("X-distance [m]")
		plt.ylabel("Y-distance [m]")
		plt.title('Linear Sum Assignment Problem')

		save_dir = os.path.join('/home/dimitri/Documents/EPFL/Courses/Sycamore/Analysis/LSAP/History')
		my_plot = 'plot_0{}'.format(counter)
		plt.savefig(os.path.join(save_dir, my_plot))


	def generate_cost(self):
		X_targets, X_agents = np.meshgrid(self.task_pos[0], self.agent_pos[0])
		Y_targets, Y_agents = np.meshgrid(self.task_pos[1], self.agent_pos[1])
		self.cost = np.sqrt((X_targets - X_agents) ** 2 + (Y_targets - Y_agents) ** 2)

	def hungarian_solver(self):
		self.generate_cost()
		agent_idx, task_idx = linear_sum_assignment(self.cost)
		return agent_idx, task_idx

	def get_agent_to_task_match(self):
		agent_idx, task_idx = self.hungarian_solver()
		# print('my row ind', row_ind, 'col index', col_ind)
		x_match = [self.agent_pos[0][agent_idx], self.task_pos[0][task_idx]]  # , x_targets[index_target[i]]]
		y_match = [self.agent_pos[1][agent_idx], self.task_pos[1][task_idx]]
		return x_match, y_match



