from AssignmentProblem import AssignmentProblem
from LexicoBAP import LexicoBAP
from LSAP import LSAP
import pickle
import os

reassigned_agent_data_lex = 'reassigned_agent_lex_format.pk'
reassigned_task_data_lex = 'reassigned_tasks_lex_format.pk'

reassigned_agent_data_lsap = 'reassigned_agent_lex_format_lsap.pk'
reassigned_task_data_lsap = 'reassigned_tasks_lex_format_lsap.pk'

'''###############################################################
############### SET INITIAL PARAMETERS UNDER #####################
 #################################################################'''
# Number of Tasks, Agents and safety distance
task_dim = 6
agent_dim = 6

# Set to RobustSetup True if robustness margin below is to set the be the minimum mu_k
rob_margin = 0
RobustSetup = True

lower_bound = -3
upper_bound = 3

'''###############################################################
############### SET INITIAL PARAMETERS ABOVE #####################
 #################################################################'''


safety_distance = 0.1  # TODO: Include in future safe sets analysis (currently hardcoded in LexBAP_to_ChoiRbot class)
use_data = True  # Set to False if above dimensions are being changed. Set to True if you wish to re-use previous problem data
store_data = True # Store data in same directory as this module, for fast evaluation
store_data_ros = True # store the data in the ros package share directory to be used for ChoiRbot. Removes previous problem

use_lexbap_lsap = True


if __name__ == "__main__":
	''' This is the main module to be used for generating a LSAP or LexBAP problem. '''

	# Generate a Problem instance containing initialising parameters (random tasks and agent locations)
	Problem = AssignmentProblem(agent_dim, task_dim, rob_margin, lower_bound=lower_bound,
								upper_bound=upper_bound, isRobust=RobustSetup)


	# Load previously stored problem (if it exists)
	if use_data is True:
		if use_lexbap_lsap is True:
			with open(reassigned_agent_data_lex, 'rb') as fi:
				Problem.agent_pos_lex = pickle.load(fi)
			with open(reassigned_task_data_lex, 'rb') as fi:
				Problem.task_pos_lex = pickle.load(fi)


	# Make format compatible with ChoiRbot, and order assignment to obtain agent and task ID
	Problem.make_choirbot_compatible()

	# Instance of LexicoBAP assignment with solver and/or LSAP with Hungarian algorithm
	LexBAP = LexicoBAP(Problem)
	LSAP = LSAP(Problem)

	# Solve...
	LSAP.hungarian_solver()
	LexBAP.optimise()

	# OPTIONAL: Find a problem giving the minimum robustness margin defined above (for LexBAP)
	while LexBAP.robustsetup is not False:

		# Use previously stored problem if desired
		if use_data is True:
			LexBAP = LexicoBAP(Problem)
			LexBAP.robust_setup(Problem)

		else: # Find a problem with a suitable robustness margin
			Problem = AssignmentProblem(agent_dim, task_dim, rob_margin, lower_bound=lower_bound,
										upper_bound=upper_bound, isRobust=RobustSetup)

			Problem.make_choirbot_compatible()

			LexBAP = LexicoBAP(Problem)
			LexBAP.robust_setup(Problem)

	Problem.make_choirbot_compatible()
	LexBAP.order_assignment()
	LSAP.order_assignment()
	print('\n \n Optimal Task Assignment is: \n', LexBAP.optimal_ass, '\n \n muk is: \n', LexBAP.mu_k)


	''' ############### Plot obtained solutions #########'''
	LexBAP.plot_solution()
	# LexBAP.graph_vis(clean = True)
	LSAP.plot_solution()
	# LSAP.graph_vis(clean = True)


	# Store data locally and/or for ROS launch file
	if store_data_ros is True:
		# ros2_share_dir = os.path.join('/home/dimitri/ChoirBot/install/sycamore/share/sycamore')
		path_cwd = os.getcwd()
		path_parent = os.path.abspath(os.path.join(path_cwd, os.pardir, os.pardir, os.pardir))
		ros2_share_dir = os.path.join(path_parent + '/install/sycamore/share/sycamore')
		print('save directory for ROS2 is: ', ros2_share_dir)


		if use_lexbap_lsap is True:
			#comment out next two lines if first time generating
			# os.remove(os.path.join(ros2_share_dir, reassigned_agent_data_lex))
			# os.remove(os.path.join(ros2_share_dir, reassigned_task_data_lex))
			LexBAP.store_data_lex(store_dir = ros2_share_dir)
			LexBAP.store_data_cb(store_dir = ros2_share_dir)

			# os.remove(os.path.join(ros2_share_dir, reassigned_agent_data_lsap))
			# os.remove(os.path.join(ros2_share_dir, reassigned_task_data_lsap))
			LSAP.store_data_lex(store_dir=ros2_share_dir)
			LSAP.store_data_cb(store_dir=ros2_share_dir)

	if store_data is True:
		__location__ = os.path.realpath(
			os.path.join(os.getcwd(), os.path.dirname(__file__)))
		if use_lexbap_lsap is True:
			LexBAP.store_data_lex(store_dir=__location__)
			LexBAP.store_data_cb(store_dir=__location__)

			LSAP.store_data_lex(store_dir=__location__)
			LSAP.store_data_cb(store_dir=__location__)


	# Make gif
	# import imageio
	#
	# images = []
	# image_dir = os.path.join('/home/dimitri/Documents/EPFL/Courses/Sycamore/Analysis/LexBAP/Test')
	# gif_name = 'LexBAP.gif'
	# # print(sorted(os.listdir(image_dir)))
	#
	# for file_name in sorted(os.listdir(image_dir)):
	# 	if file_name.endswith('.png'):
	# 		file_path = os.path.join(image_dir, file_name)
	# 		images.append(imageio.imread(file_path))
	# imageio.mimsave('/home/dimitri/Documents/EPFL/Courses/Sycamore/animation/lexbap_no_collision.gif', images, fps=10)
	# #
	# #





