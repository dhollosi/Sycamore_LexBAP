import numpy as np
from matplotlib import pyplot as plt
import random
from copy import deepcopy
import networkx as nx
import pickle
import os
import shapely.geometry as sg
import descartes


class LexicoBAP:

	def __init__(self, Problem):
		''' Main class to be used with the AssignmentProblem class to generate a Lexicographic Bottleneck Assignment
		Problem. The currently implemented solver is the Edge Removal algorithm (BAP_algorithm method)
		It gets called from the main ap_generator.py module'''

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

		self.agent_pos_ordered_lex = []
		self.task_pos_ordered_lex = []

		self.agent_pos_ordered_cb = None
		self.task_pos_ordered_cb = None

		self.agent_list = None
		self.task_list = None

		self.colour = None

		self.cost = self.generate_cost()
		self.optimal_ass = None
		self.mu_k = None
		self.min_mu_k = None
		self.a_k = None

	def get_ak(self):

		self.min_mu_k = min(self.mu_k)
		store_array = []
		for i in range(0, self.task_dim):
			current_val = self.cost[i][i] + self.mu_k[i] - 0.5 * (self.min_mu_k - 0.1)
			store_array.append(current_val)
		self.a_k = min(store_array)

	def check_at(self, timestep):

		self.get_ak()
		a_t = 0.1*timestep+0.5*(self.min_mu_k - 0.1)

		if a_t < self.a_k:
			return a_t
		else:
			a_k = self.a_k
			return a_k
	def get_bt(self, timestep):
		output = self.check_at(timestep)

		b_t = self.a_k - output + 0.5*(self.min_mu_k - 0.1)
		return b_t

	def plot_solution_robust(self, timestep):
		x_match_lex, y_match_lex = self.get_agent_to_task_match()
		fig, ax = plt.subplots()
		self.colour = self.colourmap(self.agent_dim)
		self.min_mu_k = min(self.mu_k)

		circle_agent = self.check_at(timestep)
		circle_task = self.get_bt(timestep)

		print('agent radius {} and task radius {}'.format(circle_agent,circle_task))


		for i in range(0, self.task_dim):
			x_new = [x_match_lex[0][i], x_match_lex[1][i]]
			y_new = [y_match_lex[0][i], y_match_lex[1][i]]
			p = plt.plot(x_new, y_new, marker='o', markevery=[0], label='Bottleneck Edge {}'.format(i + 1))

			a = sg.Point(self.agent_pos_ordered_lex[0][i], self.agent_pos_ordered_lex[1][i]).buffer(circle_agent)
			b = sg.Point(self.task_pos_ordered_lex[0][i], self.task_pos_ordered_lex[1][i]).buffer(circle_task)

			# left = a.difference(b)
			# right = b.difference(a)
			left = a
			right = b
			middle = a.intersection(b)
			print(middle, middle == 'POLYGON EMPTY')

			ax = plt.gca()
			ax.add_patch(descartes.PolygonPatch(left, fc=p[-1].get_color(), ec='k', alpha=0.2))
			ax.add_patch(descartes.PolygonPatch(right, fc=p[-1].get_color(), ec='k', alpha=0.2))
			# ax.add_patch(descartes.PolygonPatch(middle, fc='g', ec='k', alpha=0.4))


		plt.legend()
		plt.grid()
		ax.set(xlim=(2 * self.lower_bound, 2 * self.upper_bound), ylim=(2 * self.lower_bound, 2 * self.upper_bound))
		ax.set_aspect('equal')
		plt.xlabel("X-distance [m]")
		plt.ylabel("Y-distance [m]")
		plt.title('Lexicographic BAP')
		plt.show()

	def robust_setup(self, Problem_Data):
		self.optimise()

		index_min = [i for i, val in enumerate(self.mu_k) if val == min(self.mu_k)]
		if min(self.mu_k) <= self.rob_margin:
			print('Margin on bottlneck order {} is not big enough'.format(index_min))
			self.agent_pos = Problem_Data.agent_pos_lex
			self.task_pos = Problem_Data.task_pos_lex
			self.optimise()
			if self.cost[0][0] == 0.0:
				raise AssertionError('Could not compute a suitable margin, bad luck! Try rerunning the problem; '
									 'increasing the bounds;  making the '
									 'robustness margins lower or decreasing number of agents....')

		if min(self.mu_k) >= self.rob_margin:
			self.robustsetup = False # robustness margins are large enough


	def generate_cost(self):
		X_targets, X_agents = np.meshgrid(self.task_pos[0], self.agent_pos[0])
		Y_targets, Y_agents = np.meshgrid(self.task_pos[1], self.agent_pos[1])
		cost = np.sqrt((X_targets - X_agents) ** 2 + (Y_targets - Y_agents) ** 2)
		return cost

	def optimise(self):

		self.optimal_ass, self.mu_k = self.lexico_assignement()


	def lexico_assignement(self):
		'''
		Computes the Lexicographic Bottleneck Assignement Problem with Edge Removal algorithm
		TODO: Make this more "pythonic" as this was adapted from Matlab
		'''

		E = np.empty(self.task_dim+1, dtype = object)
		M = np.empty(self.task_dim, dtype = object)
		Eplus = np.empty(self.task_dim, dtype = object)

		i_bott = np.empty(self.task_dim, dtype = object)
		j_bott = np.empty(self.task_dim, dtype = object)
		i_crit_plus = np.empty(self.task_dim, dtype = object)
		j_crit_plus = np.empty(self.task_dim, dtype = object)

		b = np.empty(self.task_dim, dtype = object)
		b_plus = np.empty(self.task_dim, dtype = object)
		b_plus[-1] = np.inf

		M0 = np.eye(self.agent_dim)
		E[0] = np.ones((self.agent_dim,self.task_dim))

		for gn in range(0,self.task_dim):
			#find bottleneck
			# print('\nstarting first BAP at iteration {}\n'.format(gn+1))
			M[gn], i_bott[gn], j_bott[gn], b[gn] = self.BAP_algorithm(E[gn], self.cost, M0)
			# print('agent {}'.format(int(i_bott[gn]+1)),' assigned to task {}'.format(int(j_bott[gn]+1)))
			# print('and corresponding bottleneck is b = {}'.format(b[gn]))

			#find bounding edge above
			Eplus[gn] = deepcopy(E[gn])
			Eplus[gn][int(i_bott[gn][0]), int(j_bott[gn][0])] = 0

			if np.sum(np.sum(Eplus[gn])) != 0:
				M0_plus_bar = deepcopy(M[gn])
				M0_plus_bar[int(i_bott[gn][0]), int(j_bott[gn][0])] = 0
				M0_plus = self.augmented_path(j_bott[gn][0], M0_plus_bar, Eplus[gn],self.cost)

				smth, i_crit_plus[gn], j_crit_plus[gn], b_plus[gn] = self.BAP_algorithm(Eplus[gn], self.cost, M0_plus)
			else:
				j_crit_plus[gn] = 0
				i_crit_plus[gn] = 0
				b_plus[gn] = np.inf

			E[gn+1] = deepcopy(E[gn])
			E[gn+1][int(i_bott[gn][0]),:] = 0
			E[gn+1][:, int(j_bott[gn][0])] = 0

			M0 = deepcopy(M[gn] * E[gn+1]) # Try rerunning again if it blocks here...

		Bott = np.zeros((self.agent_dim,self.task_dim))
		mu = np.zeros((self.task_dim,1))

		for gn in range(0,self.task_dim):
			Bott[int(i_bott[gn][0]),int(j_bott[gn][0])] = gn+1
			mu[gn] = b_plus[gn] - b[gn]

		return Bott, mu

	def BAP_algorithm(self, E_gn, tau, M0):
		'''
		Edge removal algorithm for solving the LexicoBAP assignment
		TODO: Make this more "pythonic" as this was adapted from Matlab
		'''

		J, I = np.meshgrid(np.linspace(0, self.task_dim-1, self.task_dim), np.linspace(0, self.agent_dim-1, self.agent_dim))
		M = deepcopy(M0)
		matching = True
		Ebar = deepcopy(E_gn)
		ibar = None
		jbar = None
		bottleneck = None
		while matching is True:
			#find max edge == bottleneck
			# print('Ebar is {}'.format(Ebar)) #useful for debugging ...
			# print('Cost is {}'.format(self.cost))
			J_connected = J[Ebar == 1]
			I_connected = I[Ebar == 1]
			tau_connected = tau[Ebar == 1]

			midx = [i for i, val in enumerate(tau_connected) if val == max(tau_connected)]
			jbar = J_connected[midx]
			ibar = I_connected[midx]
			bottleneck = np.amax(tau_connected)

			#remove edge
			Ebar[int(ibar[0]),int(jbar[0])] = 0
			if M[int(ibar[0]),int(jbar[0])] == 1: #if edge in matching
				Mbar = deepcopy(M)
				Mbar[int(ibar[0]),int(jbar[0])] = 0
				Mv = self.augmented_path(int(jbar[0]), Mbar, Ebar, tau)

				if self.areSame(Mv, Mbar, len(Mv)) is False:
					M = deepcopy(Mv)
				else: #if Mbar == Mv (augmented path is the same as initial matrix)
					matching = False
			elif np.sum(np.sum(M)) == 0:
				raise Exception("No edge in matching found")
			else:
				pass
		return M, ibar, jbar, bottleneck

	def augmented_path(self, jbar, Mbar, Ebar, tau):
		'''
		Augmented Path method used in Edge Removal Algo.
		TODO: Make this more "pythonic" as this was adapted from Matlab
		'''

		F = np.zeros((self.agent_dim,1))
		m = -np.ones((self.agent_dim,1))

		for i in range(0,self.agent_dim):
			for j in range(0,self.task_dim):
				if Mbar[i,j] == 1:
					m[i] = j
		v = deepcopy(m)
		done = False
		t = jbar
		while done is False:
			st = np.zeros((self.agent_dim,1))
			for i in range(0, self.agent_dim):
				if F[i] == 0 and Ebar[i,int(t)] == 1:
					st[i] = i+1
			tau_t = deepcopy(tau[:,int(t)])
			indx = np.where(st!=0)
			tau_st = []
			for i in range(0,len(indx[1])):
				index = indx[0][i]
				tau_st.append(tau_t[index])
			st_nonz = st[st != 0]-1
			astar_partidx = [i for i, val in enumerate(tau_st) if val == min(tau_st)]
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
				M[i,int(v[i])] = 1

		return M

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

	def plot_solution(self):
		x_match_lex, y_match_lex = self.get_agent_to_task_match()
		fig, ax = plt.subplots()
		self.colour = self.colourmap(self.agent_dim)
		self.min_mu_k = min(self.mu_k)

		for i in range(0,self.task_dim):
			x_new = [x_match_lex[0][i], x_match_lex[1][i]]
			y_new = [y_match_lex[0][i], y_match_lex[1][i]]
			p =plt.plot(x_new, y_new, marker='o', markevery=[0], label = 'Bottleneck Edge {}'.format(i+1))
			cc = plt.Circle((self.agent_pos[0][i], self.agent_pos[1][i]), 0.09, alpha=0.8, color=p[0].get_color())
			ax.add_artist(cc)
		for j in range(0,self.agent_dim):
			plt.scatter(self.agent_pos[0], self.agent_pos[1], color = self.colour [j])
		plt.legend()
		plt.grid()
		ax.set(xlim=(2*self.lower_bound, 2*self.upper_bound), ylim=(2*self.lower_bound, 2*self.upper_bound))
		ax.set_aspect('equal')
		plt.xlabel("X-distance [m]")
		plt.ylabel("Y-distance [m]")
		plt.title('Lexicographic BAP')

		plt.show()

	def graph_vis(self,*, clean = bool):
		'''
		Make a graph visualisation displaying the cost between all agents and tasks
		'''
		self.optimise()
		G = nx.Graph()
		MatchGraph = deepcopy(G)
		Bottleneck = deepcopy(G)

		# print('Agents \n x = {}'.format(agent_idx + 1), ' \n Matched with tasks: \n y = {}'.format(task_idx + 1))

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
			MatchGraph.add_edge("A{}".format(self.agent_list[j]+1), "T{}".format(task_number + 1),
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
		plt.show()

	def store_data_lex(self, *, store_dir=str):

		if type(store_dir) is not str:
			raise TypeError('Directory must be in string format')

		reassigned_agent_data_lex = 'reassigned_agent_lex_format.pk'
		reassigned_task_data_lex = 'reassigned_tasks_lex_format.pk'

		agent_file = os.path.join(store_dir, reassigned_agent_data_lex)
		task_file = os.path.join(store_dir, reassigned_task_data_lex)

		with open(agent_file, 'wb') as fi:
			pickle.dump(self.agent_pos_ordered_lex, fi)

		with open(task_file, 'wb') as fi:
			pickle.dump(self.task_pos_ordered_lex, fi)

	def store_data_cb(self, *, store_dir=str):

		if type(store_dir) is not str:
			raise TypeError('Directory must be in string format')

		reassigned_task_data_cb = 'reassigned_tasks_cb_format.pk'
		reassigned_agents_data_cb = 'reassigned_agents_cb_format.pk'

		agent_file = os.path.join(store_dir, reassigned_agents_data_cb)
		task_file = os.path.join(store_dir, reassigned_task_data_cb)

		with open(agent_file, 'wb') as fi:
			pickle.dump(self.agent_pos_ordered_cb, fi)

		with open(task_file, 'wb') as fi:
			pickle.dump(self.task_pos_ordered_cb, fi)

	def areSame(self, A, B, n):
		for i in range(self.agent_dim):
			for j in range(self.task_dim):
				if (A[i][j] != B[i][j]):
					return False
		return True

	def get_sequence(self):
		''' Generate current sequence of agents assigned to tasks'''

		task_sequence = []
		agent_sequence = []
		for i in range(0, self.task_dim):
			agent_idx, task_idx = np.where(self.optimal_ass == i + 1)
			task_sequence.append(int(task_idx))
			agent_sequence.append((int(agent_idx)))

		return agent_sequence, task_sequence


	def order_assignment(self):
		'''
		used for setting matched agend ID to task ID with corresponding bottleneck order increasing,
		i.e 1st order bottelenck --> A1 - T1, 2nd --> A2 - T2 etc...
		This is necessary for the current implementation in ChoiRbot
		'''

		agent_sequence, task_sequence = self.get_sequence()

		self.agent_pos_ordered_lex = [self.agent_pos[0][agent_sequence], self.agent_pos[1][agent_sequence]]
		self.task_pos_ordered_lex = [self.task_pos[0][task_sequence], self.task_pos[1][task_sequence]]

		self.agent_pos_ordered_cb = self.agent_pos_cb[agent_sequence, :]
		self.task_pos_ordered_cb = self.task_pos_cb[task_sequence, :]

	def colourmap(self,size):
		'''
		To be used if manay agents are computed (plot visualisation)
		'''
		#method 1
		number_of_colors = size
		colour = ["#" + ''.join([random.choice('0123456789ABCDEF') for j in range(6)])
				 for i in range(number_of_colors)]

		#method 2
		colors_ = lambda n: list(map(lambda i: "#" + "%06x" % random.randint(0, 0xFFFFFF), range(n)))

		return colors_(size) #colors_(size)

