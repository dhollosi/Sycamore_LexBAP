import numpy as np
from copy import deepcopy
import os
from matplotlib import pyplot as plt
# from choirbot_interfaces.msg import PositionTask, PositionTaskArray
import shapely.geometry as sg
import descartes
import networkx as nx


class ChoirBotLexicoBAP:

	def __init__(self, task_data , *, agent_dim = int):
		''' This class is a modified version of the main Lexico'''

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

		self.optimal_ass = None
		self.mu_k = None
		self.mu_k_array = []



		self.min_mu_k = None
		self.a_k = np.zeros((self.agent_dim,1))

	def get_ak(self):

		self.min_mu_k = min(self.mu_k_array[0])

		self.a_k[0] = self.init_cost[0][0] + self.mu_k_array[0][0] - 0.5 * (self.min_mu_k - 0.1)

		for i in range(1, self.task_dim):
			self.a_k[i] = min(self.init_cost[i][i] + self.mu_k_array[0][i] - 0.5 * (self.min_mu_k - 0.1), self.a_k[i-1])



	def get_at(self, index, timestep):

		# self.get_ak()

		if timestep > 3:
			dist_current_agent = np.sqrt(self.agent_pos[0][index]**2 + self.agent_pos[1][index]**2)
			dist_previous_agent = np.sqrt(self.x_agent_array[timestep-2][index]**2 + self.y_agent_array[timestep-2][index]**2)
			speed = dist_current_agent-dist_previous_agent
		else:
			speed = 0.05
		# print('speed is {} for agent {} '.format(speed, index))
		#0.5 for 8 agents,0.6 for 6 agents, 0.7 for 5
		speed = 0.073
		a_t = abs(speed)*timestep+0.5*(self.min_mu_k - 0.1)

		if a_t < self.a_k[index]:
			return a_t
		else:
			a_k = self.a_k[index]
			return a_k

	def get_bt(self, index, timestep):
		output = self.get_at(index, timestep)

		b_t = self.a_k[index] - output + 0.5 * (self.min_mu_k - 0.1)
		return b_t


	def get_agent_pos_from_dict(self, global_dict):

		agent_pos_x = np.ndarray(shape=(self.agent_dim,1))
		agent_pos_y = np.ndarray(shape=(self.agent_dim,1))
		for i in range(0,self.agent_dim):
			current_pose_x = global_dict['agent_{}'.format(i)]['pose_x']
			current_pose_y = global_dict['agent_{}'.format(i)]['pose_y']
			agent_pos_x[i] = current_pose_x
			agent_pos_y[i] = current_pose_y


		self.agent_pos = [agent_pos_x, agent_pos_y]

	def get_agent_to_task_match(self):
		'''
		Make two arrays with x and y positions of agents & tasks
		Useful for plotting
		'''
		row_ind = []
		col_ind = []
		for i in range(0, self.task_dim):
			agent_indx, task_indx = np.where(self.optimal_ass == i+1)
			row_ind.append(agent_indx[0])
			col_ind.append(task_indx[0])

		self.agent_list = np.array(row_ind)
		self.task_list = np.array(col_ind)

		x_match = [self.agent_pos[0][self.agent_list], self.task_pos[0][self.task_list]]
		y_match = [self.agent_pos[1][self.agent_list], self.task_pos[1][self.task_list]]

		return x_match, y_match
	# def get_agent_to_task_match(self):
	#
	# 	row_ind = []
	# 	col_ind = []
	# 	for i in range(0, self.agent_dim):
	# 		agent_indx, task_indx = np.where(self.optimal_ass == i+1)
	# 		row_ind.append(agent_indx[0])
	# 		col_ind.append(task_indx[0])
	# 	row_ind = np.array(row_ind)
	# 	col_ind = np.array(col_ind)
	# 	x_match = [self.agent_pos[0][row_ind], self.task_pos[0][col_ind]]  # , x_targets[index_target[i]]]
	# 	y_match = [self.agent_pos[1][row_ind], self.task_pos[1][col_ind]]
	#
	# 	return x_match, y_match


	def plot_solution(self, counter):
		x_match_lex, y_match_lex = self.get_agent_to_task_match()
		fig, ax = plt.subplots()

		for i in range(0, self.task_dim):
			x_new = [x_match_lex[0][i], x_match_lex[1][i]]
			y_new = [y_match_lex[0][i], y_match_lex[1][i]]
			plt.plot(x_new, y_new, marker='o', markevery=[0])#, label='Bottleneck Edge {}'.format(i + 1))

		plt.legend()
		plt.grid()
		ax.set(xlim=(2 * self.lower_bound, 2 * self.upper_bound), ylim=(2 * self.lower_bound, 2 * self.upper_bound))
		ax.set_aspect('equal')
		plt.xlabel("X-distance [m]")
		plt.ylabel("Y-distance [m]")
		plt.title('Lexicographic BAP')

		save_dir = os.path.join('/home/dimitri/Documents/EPFL/Courses/Sycamore/Analysis/LexBAP/Assignments')
		my_plot = 'plot_0{}'.format(counter)
		plt.savefig(os.path.join(save_dir, my_plot))

	def plot_solution_robust(self, counter, timestep):
		x_match_lex, y_match_lex = self.get_agent_to_task_match()
		fig, ax = plt.subplots()

		for i in range(0, self.task_dim):
			x_new = [x_match_lex[0][i], x_match_lex[1][i]]
			y_new = [y_match_lex[0][i], y_match_lex[1][i]]
			p = plt.plot(x_new, y_new, marker='o', markevery=[0])#, label='Bottleneck Edge {}'.format(i + 1))
			if timestep > 1:
				circle_agent = self.get_at(i, timestep)
				circle_task = self.get_bt(i, timestep)
				# print('agent radius {} and task radius {} for agent {}'.format(circle_agent, circle_task, i))

				a = sg.Point(self.init_agent_pos[0][i], self.init_agent_pos[1][i]).buffer(circle_agent)
				b = sg.Point(self.init_task_pos[0][i], self.init_task_pos[1][i]).buffer(circle_task)

				middle = a.intersection(b)

				ax = plt.gca()

				ax.add_patch(descartes.PolygonPatch(middle, fc=p[0].get_color(), ec='k', alpha=0.4))

		plt.legend()
		plt.grid()
		ax.set(xlim=(2 * self.lower_bound, 2 * self.upper_bound), ylim=(2 * self.lower_bound, 2 * self.upper_bound))
		ax.set_aspect('equal')
		plt.xlabel("X-distance [m]")
		plt.ylabel("Y-distance [m]")
		plt.title('Lexicographic BAP')

		save_dir = os.path.join('/home/dimitri/Documents/EPFL/Courses/Sycamore/Analysis/LexBAP/Test')
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
			for j in range(0,len(timestep_list)):
				current_x_history.append(self.x_agent_array[j][i])
				current_y_history.append(self.y_agent_array[j][i])

			x_new = [x_match_lex[0][i], x_match_lex[1][i]]
			y_new = [y_match_lex[0][i], y_match_lex[1][i]]
			p = plt.plot(x_new, y_new, marker='o', markevery=[0])#, label='Bottleneck Edge {}'.format(i + 1))
			plt.plot(current_x_history, current_y_history, color = p[0].get_color(), linestyle='dotted')

			if timestep > 1:
				circle_agent = self.get_at(i, timestep)
				circle_task = self.get_bt(i, timestep)
				# print('agent radius {} and task radius {} for agent {}'.format(circle_agent, circle_task, i))

				a = sg.Point(self.init_agent_pos[0][i], self.init_agent_pos[1][i]).buffer(circle_agent)
				b = sg.Point(self.init_task_pos[0][i], self.init_task_pos[1][i]).buffer(circle_task)
				middle = a.intersection(b)

				ax = plt.gca()
				ax.add_patch(descartes.PolygonPatch(middle, fc=p[0].get_color(), ec='k', alpha=0.4))

			current_x_history = []
			current_y_history = []

		plt.legend()
		plt.grid()
		ax.set(xlim=(2 * self.lower_bound, 2 * self.upper_bound), ylim=(2 * self.lower_bound, 2 * self.upper_bound))
		ax.set_aspect('equal')
		plt.xlabel("X-distance [m]")
		plt.ylabel("Y-distance [m]")
		plt.title('Lexicographic BAP')

		save_dir = os.path.join('/home/dimitri/Documents/EPFL/Courses/Sycamore/Analysis/LexBAP/History')
		my_plot = 'plot_0{}'.format(counter)
		plt.savefig(os.path.join(save_dir, my_plot))

	def plot_robustness_margins(self, counter):
		timestep = []

		for j in range(0, counter):
			timestep.append(j)

		current_assignement_margin = []

		fig, ax = plt.subplots()
		for i in range(0, self.agent_dim-1):
			for j in range(0,len(timestep)):
				current_assignement_margin.append(self.mu_k_array[j][i])
			plt.plot(timestep, current_assignement_margin, marker='o', markevery=[-1])#, label='Bottleneck Edge {}'.format(i+1))
			current_assignement_margin = []

		plt.legend()
		plt.grid()
		plt.xlabel("Timestep")
		plt.ylabel("Robustness margins")
		plt.title('Lexicographic BAP')

		save_dir = os.path.join('/home/dimitri/Documents/EPFL/Courses/Sycamore/Analysis/LexBAP/Robustness')
		my_plot = 'plot_010000{}'.format(counter)
		plt.savefig(os.path.join(save_dir, my_plot))
		# plt.show()


	def areSame(self, A, B, n):
		for i in range(self.agent_dim):
			for j in range(self.task_dim):
				if (A[i][j] != B[i][j]):
					return False
		return True

	def generate_cost(self):
		X_targets, X_agents = np.meshgrid(self.task_pos[0], self.agent_pos[0])
		Y_targets, Y_agents = np.meshgrid(self.task_pos[1], self.agent_pos[1])
		cost = np.sqrt((X_targets - X_agents) ** 2 + (Y_targets - Y_agents) ** 2)
		return cost

	def optimise(self):


		self.cost = self.generate_cost()

		self.optimal_ass, self.mu_k = self.lexico_assignement()

		for i in range(0,self.agent_dim):
			if self.cost[i][i] < 0.06:
				self.optimal_ass[i][i] = i+1
				self.mu_k[i] = self.mu_k_array[-1][i]


	def	lexico_assignement(self):

		E = np.empty(self.task_dim + 1, dtype=object)
		M = np.empty(self.task_dim, dtype=object)
		Eplus = np.empty(self.task_dim, dtype=object)

		i_bott = np.empty(self.task_dim, dtype=object)
		j_bott = np.empty(self.task_dim, dtype=object)
		i_crit_plus = np.empty(self.task_dim, dtype=object)
		j_crit_plus = np.empty(self.task_dim, dtype=object)

		b = np.empty(self.task_dim, dtype=object)
		b_plus = np.empty(self.task_dim, dtype=object)
		b_plus[-1] = np.inf

		M0 = np.eye(self.agent_dim)
		E[0] = np.ones((self.agent_dim, self.agent_dim))

		for gn in range(0, self.task_dim):

			M[gn], i_bott[gn], j_bott[gn], b[gn] = self.BAP_algorithm(E[gn], M0)

			Eplus[gn] = deepcopy(E[gn])
			Eplus[gn][int(i_bott[gn][0]), int(j_bott[gn][0])] = 0

			if np.sum(np.sum(Eplus[gn])) != 0:
				M0_plus_bar = deepcopy(M[gn])
				M0_plus_bar[int(i_bott[gn][0]), int(j_bott[gn][0])] = 0
				M0_plus = self.augmented_path(j_bott[gn][0], M0_plus_bar, Eplus[gn])

				smth, i_crit_plus[gn], j_crit_plus[gn], b_plus[gn] = self.BAP_algorithm(Eplus[gn],  M0_plus)
			else:
				j_crit_plus[gn] = 0
				i_crit_plus[gn] = 0
				b_plus[gn] = np.inf

			E[gn + 1] = deepcopy(E[gn])
			E[gn + 1][int(i_bott[gn][0]), :] = 0
			E[gn + 1][:, int(j_bott[gn][0])] = 0

			M0 = deepcopy(M[gn] * E[gn + 1])

		Bott = np.zeros((self.agent_dim, self.task_dim))
		mu = np.zeros((self.task_dim, 1))

		for gn in range(0, self.task_dim):
			Bott[int(i_bott[gn][0]), int(j_bott[gn][0])] = gn + 1
			mu[gn] = b_plus[gn] - b[gn]

		return Bott, mu


	def BAP_algorithm(self, E_gn, M0):
		J, I = np.meshgrid(np.linspace(0, self.task_dim - 1, self.task_dim),
						   np.linspace(0, self.agent_dim - 1, self.agent_dim))
		M = deepcopy(M0)
		matching = True
		Ebar = deepcopy(E_gn)
		ibar = None
		jbar = None
		bottleneck = None
		while matching is True:
			# find max edge == bottleneck
			# print('\n \ncost matrix is \n {} \n \n and Ebar is \n {}'.format(self.cost, Ebar))

			J_connected = J[Ebar == 1]
			I_connected = I[Ebar == 1]
			tau_connected = self.cost[Ebar == 1]

			midx = [i for i, val in enumerate(tau_connected) if val == max(tau_connected)]
			jbar = J_connected[midx]
			ibar = I_connected[midx]
			bottleneck = np.amax(tau_connected)
			# remove edge
			Ebar[int(ibar[0]), int(jbar[0])] = 0
			if M[int(ibar[0]), int(jbar[0])] == 1:  # if edge in matching
				Mbar = deepcopy(M)
				# Mbar = M
				Mbar[int(ibar[0]), int(jbar[0])] = 0
				Mv = self.augmented_path(int(jbar[0]), Mbar, Ebar)
				if self.areSame(Mv, Mbar, len(Mv)) is False:
					M = deepcopy(Mv)
				else:  # if Mbar == Mv (augmented path is the same as initial matrix)
					matching = False
			elif np.sum(np.sum(M)) == 0:
				raise Exception("No edge in matching found")
			else:
				pass
		return M, ibar, jbar, bottleneck


	def augmented_path(self, jbar, Mbar, Ebar):
		F = np.zeros((self.agent_dim, 1))
		m = -np.ones((self.agent_dim, 1))

		for i in range(0, self.agent_dim):
			for j in range(0, self.task_dim):
				if Mbar[i, j] == 1:
					m[i] = j
		v = deepcopy(m)
		done = False
		t = jbar
		while done is False:
			st = np.zeros((self.agent_dim, 1))
			for i in range(0, self.agent_dim):
				if F[i] == 0 and Ebar[i, int(t)] == 1:
					st[i] = i + 1
			self.cost_t = deepcopy(self.cost[:, int(t)])
			indx = np.where(st != 0)
			self.cost_st = []
			for i in range(0, len(indx[1])):
				index = indx[0][i]
				self.cost_st.append(self.cost_t[index])
			st_nonz = st[st != 0] - 1
			astar_partidx = [i for i, val in enumerate(self.cost_st) if val == min(self.cost_st)]
			astar = st_nonz[astar_partidx]
			if np.sum(st) == 0:
				if t == jbar:
					done = True
				else:
					k = np.where(m == t)
					astar = k[0]
					t = deepcopy(v[astar[0]])
					v[astar] = deepcopy(m[astar[0]])
			else:
				if m[int(astar[0])] == -1:
					v[int(astar[0])] = t
					done = True
				else:
					v[int(astar[0])] = t
					F[int(astar[0])] = 1
					t = m[int(astar[0])]

		M = np.zeros((self.agent_dim, self.task_dim))
		for i in range(0, self.agent_dim):
			if v[i] != -1:
				M[i, int(v[i])] = 1

		return M

	def graph_vis(self, counter,*, clean = bool):
		'''
		Make a graph visualisation displaying the cost between all agents and tasks
		'''
		fig, ax = plt.subplots()

		self.optimise()
		G = nx.Graph()
		MatchGraph = deepcopy(G)
		Bottleneck = deepcopy(G)


		# Generate Agents and Task nodes at defined position
		for i in range(0, self.agent_dim):
			G.add_node('A{}'.format(i + 1), pos=(int(self.agent_pos[0][i]), int(self.agent_pos[1][i])),
					   node_color="green")
		for j in range(0, self.task_dim):
			G.add_node('T{}'.format(j + 1), pos=(int(self.task_pos[0][j]), int(self.task_pos[1][j])))
		node_list = list(G.nodes)

		# Generate edges from cost matrix
		for i in range(0, self.agent_dim):
			for j in range(0, self.task_dim):
				G.add_edge("A{}".format(i + 1), "T{}".format(j + 1), weight=round(self.cost[i, j], 2))

		# Generate assigned edges from algorithm
		for j, task_number in enumerate(self.task_list):
			MatchGraph.add_edge("A{}".format(self.agent_list[j] + 1), "T{}".format(task_number + 1),
								weight=round(self.cost[self.agent_list[j], task_number], 2))

		# Generate 1st order bottleneck edge from algorithm
		Bottleneck.add_edge("A{}".format(self.agent_list[0] + 1), "T{}".format(self.task_list[0] + 1))

		edge_list = G.edges
		matched_edges_list = MatchGraph.edges
		bottleneck_edge = Bottleneck.edges

		# Choose what kind of layout to display graph (Real vs Tight)

		if clean is True:
			pos = nx.spring_layout(G)
		else:
			pos = nx.get_node_attributes(G, 'pos')

		# nodes
		options = {"edgecolors": "tab:gray", "node_size": 800, "alpha": 0.9}
		nx.draw_networkx_nodes(G, pos, nodelist=node_list[0:self.agent_dim], node_color="tab:blue", **options)
		nx.draw_networkx_nodes(G, pos, nodelist=node_list[self.agent_dim:self.agent_dim + self.task_dim],
							   node_color="tab:green", **options)

		# node labels
		nx.draw_networkx_labels(G, pos, font_size=15, font_family="arial")

		# # edge weight labels
		edge_labels = nx.get_edge_attributes(MatchGraph, "weight")
		nx.draw_networkx_edge_labels(G, pos, edge_labels)

		nx.draw_networkx_edges(G, pos, width=1.0, alpha=0.5, edge_color="k")

		nx.draw_networkx_edges(
			G,
			pos,
			edgelist=edge_list,
			width=8,
			alpha=0.2,
			edge_color="tab:blue",
		)
		nx.draw_networkx_edges(
			MatchGraph,
			pos,
			edgelist=matched_edges_list,
			width=8,
			alpha=0.8,
			edge_color="tab:green",

		)
		nx.draw_networkx_edges(
			Bottleneck,
			pos,
			edgelist=bottleneck_edge,
			width=8,
			alpha=1,
			edge_color="tab:red",
		)

		plt.tight_layout()
		plt.title('Lexicographic Bottleneck Assignment')
		save_dir = os.path.join('/home/dimitri/Documents/EPFL/Courses/Sycamore/Analysis/LexBAP/Graph')
		my_plot = 'plot_010000{}'.format(counter)
		plt.savefig(os.path.join(save_dir, my_plot))
		# plt.show()