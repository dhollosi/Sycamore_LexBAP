import rclpy
from choirbot.guidance.task.task_sycamore import TaskGuidance
from choirbot.guidance.task.executor_sycamore import PositionTaskExecutor
from choirbot.optimizer.task_optimizer_sycamore import TaskOptimizer

def main():
    rclpy.init()

    # initialize task guidance
    opt_settings = {'max_iterations': 20}
    executor = PositionTaskExecutor()
    optimizer = TaskOptimizer(resolution_strategy='lexbap', cost_function= 'euclidean',
                              settings=opt_settings)
    
    guidance = TaskGuidance(optimizer, executor, None, 'pubsub', 'odom')

    rclpy.spin(guidance)
    rclpy.shutdown()
