########### Linear Sum Assignment Problem #####################3
import numpy as np

from AssignmentProblem import AssignmentProblem
from LexicoBAP import LexicoBAP
import pickle

if __name__ == "__main__":

	''' ############ SET INITIAL PARAMETERS UNDER ################ '''
	# Number of Tasks, Agents and safety distance
	task_dim = 6
	agent_dim = 6
	d = 1
	rob_margin = 0.3

	RobustSetup = True
	lower_bound = -3
	upper_bound = 3

	use_data = True

	''' ############ SET INITIAL PARAMETERS ABOVE ################ '''

	agent_data = 'previous_agent.pk'
	task_data = 'previous_task.pk'

	Problem = AssignmentProblem(agent_dim, task_dim, rob_margin, lower_bound = lower_bound,
								upper_bound = upper_bound, isRobust = RobustSetup)

	with open(agent_data, 'rb') as fi:
		Problem.agent_pos_conv  = pickle.load(fi)
		# print(Problem.agent_pos_conv)
	with open(task_data, 'rb') as fi:
		Problem.task_pos_conv  = pickle.load(fi)
		# print(Problem.task_pos_conv)

	LexBAP = LexicoBAP(Problem)
	LexBAP.optimise()

	while LexBAP.robustsetup is not False:

		if use_data is True:
			LexBAP = LexicoBAP(Problem)

			LexBAP.run_coll_avoidance(Problem)
		else:

			Problem = AssignmentProblem(agent_dim, task_dim, rob_margin, lower_bound=lower_bound,
										upper_bound=upper_bound, isRobust=RobustSetup)

			LexBAP = LexicoBAP(Problem)

			LexBAP.run_coll_avoidance(Problem)


	print('\n \n Optimal Task Assignment is: \n', LexBAP.optimal_ass, '\n \n muk is: \n', LexBAP.mu_k)

	# LexBAP.graph_vis(clean = False)


	#store data
	with open(agent_data, 'wb') as fi:
		pickle.dump(LexBAP.agent_pos, fi)
	with open(task_data, 'wb') as fi:
		pickle.dump(LexBAP.task_pos, fi)

	# try to invert rows of tasks according to optimal_ass
	reassigned_task_data = 'reassigned_tasks.pk'

	Problem.make_choirbot_compatible()
	P = Problem.agent_pos_cb
	T = Problem.task_pos_cb
	task_sequence = []
	for i in range(0,agent_dim):
		row_idx, col_idx = np.where(LexBAP.optimal_ass == i+1)
		task_sequence.append(int(col_idx))

	print('task sequence {}'.format(task_sequence))

	T_new = T[task_sequence, :]
	print('\nRepositioning of tasks, set to \n{}'.format(T_new))

	with open(reassigned_task_data, 'wb') as fi:
		pickle.dump(T_new, fi)

	LexBAP.plot_solution()

