import numpy as np
from matplotlib import pyplot as plt
from scipy.optimize import linear_sum_assignment
import random
from copy import deepcopy
import networkx as nx
import pickle
import os

class LSAP:
	''' Solve the Linear Sum Assignment Problem with the Hungarian Algorithm
	Main class to be used with the AssignmentProblem class - gets called in the main ap_generator.py module
	'''

	def __init__(self, Problem):
		self.rob_margin = Problem.rob_margin
		self.agent_dim = Problem.agent_dim
		self.task_dim = Problem.task_dim
		self.robustsetup = Problem.robust_setup
		self.lower_bound = Problem.lower_bound
		self.upper_bound = Problem.upper_bound

		self.agent_pos = Problem.agent_pos_lex
		self.task_pos = Problem.task_pos_lex

		self.agent_pos_cb = Problem.agent_pos_cb
		self.task_pos_cb = Problem.task_pos_cb

		self.agent_pos_ordered_lsap = []
		self.task_pos_ordered_lsap = []

		self.agent_pos_ordered_cb = None
		self.task_pos_ordered_cb = None

		self.cost = None

	def initisalise(self):
		pass

	def generate_cost(self):

		X_targets, X_agents = np.meshgrid(self.task_pos[0], self.agent_pos[0])
		Y_targets, Y_agents = np.meshgrid(self.task_pos[1], self.agent_pos[1])
		self.cost = np.sqrt((X_targets - X_agents) ** 2 + (Y_targets - Y_agents) ** 2)


	def optimise(self):
		pass

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

	def colourmap(self,size):
		#method 1
		number_of_colors = size
		colour = ["#" + ''.join([random.choice('0123456789ABCDEF') for j in range(6)])
				 for i in range(number_of_colors)]

		#method 2
		colors_ = lambda n: list(map(lambda i: "#" + "%06x" % random.randint(0, 0xFFFFFF), range(size)))

		return colors_(size)

	def plot_solution(self):
		x_match, y_match = self.get_agent_to_task_match()
		colour = self.colourmap(self.agent_dim)
		fig, ax = plt.subplots()

		for i in range(0, self.task_dim):
			x_new = [x_match[0][i], x_match[1][i]]
			y_new = [y_match[0][i], y_match[1][i]]
			p = plt.plot(x_new, y_new,  marker='o', markevery=[0])
			cc = plt.Circle((self.agent_pos[0][i], self.agent_pos[1][i]), 0.09, alpha=0.8, color = p[0].get_color() )
			ax.add_artist(cc)
		for j in range(0, self.agent_dim):
			plt.scatter(self.agent_pos[0], self.agent_pos[1], color=colour[j])
		plt.legend()
		plt.grid()
		ax.set(xlim=(2 * self.lower_bound, 2 * self.upper_bound), ylim=(2 * self.lower_bound, 2 * self.upper_bound))
		ax.set_aspect('equal')
		plt.xlabel("X-distance [m]")
		plt.ylabel("Y-distance [m]")
		plt.title('Hungarian LSAP')
		plt.show()

	def graph_vis(self, *, clean = bool):
		cost = self.generate_cost()
		agent_idx, task_idx = self.hungarian_solver()
		G = nx.Graph()
		MatchGraph = deepcopy(G)
		fig, ax = plt.subplots()

		print('Agents \n x = {}'.format(agent_idx+1), ' \n Matched with tasks: \n y = {}'.format(task_idx+1))

		#Generate Agents and Task nodes at defined position
		for i in range(0,self.agent_dim):
			G.add_node('A{}'.format(i + 1), pos=(int(self.agent_pos[0][i]), int(self.agent_pos[1][i])), node_color = "green")
		for j in range(0, self.task_dim):
			G.add_node('T{}'.format(j+1), pos = (int(self.task_pos[0][j]),int(self.task_pos[1][j])))
		node_list = list(G.nodes)

		#Generate edges from cost matrix
		for i in range(0,self.agent_dim):
			for j in range(0, self.task_dim):
				G.add_edge("A{}".format(i+1), "T{}".format(j+1), weight=round(cost[i, j], 2))

		# Generate assigned edges from algorithm
		for j, task_number in enumerate(task_idx):
			MatchGraph.add_edge("A{}".format(j+1), "T{}".format(task_number+1))

		edge_list = G.edges
		matched_edges_list = MatchGraph.edges

		# Choose what kind of layout to display graph (Real vs Tight)
		if clean is True:
			pos = nx.spring_layout(G)
		else:
			pos = nx.get_node_attributes(G, 'pos')
		# nodes
		options = {"edgecolors": "tab:gray", "node_size": 800, "alpha": 0.9}
		nx.draw_networkx_nodes(G, pos, nodelist=node_list[0:self.agent_dim], node_color="tab:blue", **options)
		nx.draw_networkx_nodes(G, pos, nodelist=node_list[self.agent_dim:self.agent_dim+self.task_dim], node_color="tab:green", **options)

		# node labels
		nx.draw_networkx_labels(G, pos, font_size=15, font_family="arial")
		# # edge weight labels
		edge_labels = nx.get_edge_attributes(G, "weight")
		nx.draw_networkx_edge_labels(G, pos, edge_labels)

		nx.draw_networkx_edges(G, pos, width=1.0, alpha=0.5, edge_color="k")

		nx.draw_networkx_edges(
			G,
			pos,
			edgelist=edge_list,
			width=8,
			alpha=0.5,
			edge_color="tab:blue",
		)
		nx.draw_networkx_edges(
			MatchGraph,
			pos,
			edgelist=matched_edges_list,
			width=8,
			alpha=0.5,
			edge_color="tab:green",
		)
		plt.title('Linear Sum Assignment')
		plt.tight_layout()
		plt.show()

	def store_data_lex(self, *, store_dir=str):

		if type(store_dir) is not str:
			raise TypeError('Directory must be in string format')

		reassigned_agent_data_lsap = 'reassigned_agent_lex_format_lsap.pk'
		reassigned_task_data_lsap = 'reassigned_tasks_lex_format_lsap.pk'

		agent_file = os.path.join(store_dir, reassigned_agent_data_lsap)
		task_file = os.path.join(store_dir, reassigned_task_data_lsap)

		with open(agent_file, 'wb') as fi:
			pickle.dump(self.agent_pos_ordered_lsap, fi)

		with open(task_file, 'wb') as fi:
			pickle.dump(self.task_pos_ordered_lsap, fi)

	def store_data_cb(self, *, store_dir=str):

		if type(store_dir) is not str:
			raise TypeError('Directory must be in string format')

		reassigned_task_data_cb_lsap = 'reassigned_tasks_cb_format_lsap.pk'
		reassigned_agents_data_cb_sap = 'reassigned_agents_cb_format_lsap.pk'

		agent_file = os.path.join(store_dir, reassigned_agents_data_cb_sap)
		task_file = os.path.join(store_dir, reassigned_task_data_cb_lsap)

		with open(agent_file, 'wb') as fi:
			pickle.dump(self.agent_pos_ordered_cb, fi)

		with open(task_file, 'wb') as fi:
			pickle.dump(self.task_pos_ordered_cb, fi)


	def order_assignment(self):
		'''
		used for setting matched agend ID to task ID with corresponding bottleneck order increasing,
		i.e 1st order bottelenck --> A1 - T1, 2nd --> A2 - T2 etc...
		This is necessary for the current implementation in ChoiRbot
		'''

		agent_sequence, task_sequence = self.hungarian_solver()

		self.agent_pos_ordered_lsap = [self.agent_pos[0][agent_sequence], self.agent_pos[1][agent_sequence]]
		self.task_pos_ordered_lsap = [self.task_pos[0][task_sequence], self.task_pos[1][task_sequence]]

		self.agent_pos_ordered_cb = self.agent_pos_cb[agent_sequence, :]
		self.task_pos_ordered_cb = self.task_pos_cb[task_sequence, :]