import numpy as np
from ament_index_python.packages import get_package_share_directory
import os


class AssignmentProblem():
	def __init__(self,  m_agents, n_tasks, d_safety,*, lower_bound = int, upper_bound = int, isRobust = bool):
		self.agent_dim = m_agents
		self.task_dim = n_tasks
		self.rob_margin = d_safety
		self.lower_bound = lower_bound
		self.upper_bound = upper_bound
		self.robust_setup = isRobust
		self.agent_pos_lex , self.task_pos_lex = self.intialise_positions()
		self.agent_pos_cb = None
		self.task_pos_cb = None


	def intialise_positions(self):
		# self.set_init_pos_choirbot()
		isTrue = True

		x_targets = np.random.randint(self.lower_bound, self.upper_bound, 1) * np.random.rand(self.task_dim, 1) * 2
		y_targets = np.random.randint(self.lower_bound, self.upper_bound, 1) * np.random.rand(self.task_dim, 1) * 2

		x_agents = np.random.randint(self.lower_bound, self.upper_bound, 1) * np.random.rand(self.agent_dim, 1) * 2
		y_agents = np.random.randint(self.lower_bound, self.upper_bound, 1) * np.random.rand(self.agent_dim, 1) * 2

		# recompute if either array used 0 as an integer...

		if x_targets[0][0] == 0.0:
			x_targets = np.random.randint(self.lower_bound, self.upper_bound, 1) * np.random.rand(self.task_dim, 1) * 2
		if y_targets[0][0] == 0.0:
			y_targets = np.random.randint(self.lower_bound, self.upper_bound, 1) * np.random.rand(self.task_dim, 1) * 2

		if x_agents[0][0] == 0.0:
			x_agents = np.random.randint(self.lower_bound, self.upper_bound, 1) * np.random.rand(self.agent_dim, 1) * 2
		if y_agents[0][0] == 0.0:
			y_agents = np.random.randint(self.lower_bound, self.upper_bound, 1) * np.random.rand(self.agent_dim, 1) * 2

		if x_agents[0][0] or x_targets[0][0] or y_agents[0][0] or y_targets[0][0] == 0.0:

			x_targets = np.random.randint(self.lower_bound, self.upper_bound, 1) * np.random.rand(self.task_dim, 1) * 2
			y_targets = np.random.randint(self.lower_bound, self.upper_bound, 1) * np.random.rand(self.task_dim, 1) * 2
			x_agents = np.random.randint(self.lower_bound, self.upper_bound, 1) * np.random.rand(self.agent_dim, 1) * 2
			y_agents = np.random.randint(self.lower_bound, self.upper_bound, 1) * np.random.rand(self.agent_dim, 1) * 2


		return [x_agents, y_agents], [x_targets, y_targets]


	def make_choirbot_compatible(self):

		''' Converts self.initialise_positions to a format usable for ChoirBot'''

		self.agent_pos_cb = np.zeros((self.agent_dim, 3))
		self.task_pos_cb = np.zeros((self.agent_dim, 3))

		agent_x = self.agent_pos_lex[:][0]
		agent_y = self.agent_pos_lex[:][1]

		task_x = self.task_pos_lex[:][0]
		task_y = self.task_pos_lex[:][1]

		for i in range(0,self.task_dim):
			self.agent_pos_cb[i,0] = agent_x[i][0]
			self.agent_pos_cb[i,1] = agent_y[i][0]

			self.task_pos_cb[i,0] = task_x[i][0]
			self.task_pos_cb[i,1] = task_y[i][0]

		# print('\nagent pos choirbot \n',self.agent_pos_cb, '\n task pos choirbot \n ', self.task_pos_cb)




