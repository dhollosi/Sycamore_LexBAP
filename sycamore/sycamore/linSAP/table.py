import rclpy
from rclpy.node import Node
from choirbot.guidance.task.table_sycamore import PositionTaskTable
import time


def main():
    rclpy.init()

    node = Node('table_parameters', allow_undeclared_parameters=True,
                automatically_declare_parameters_from_overrides=True)
    N = node.get_parameter('N').value
    use_lexBAP = node.get_parameter('use_lexBAP').value
    use_LSAP = node.get_parameter('use_LSAP').value

    table = PositionTaskTable(N, use_lexBAP, use_LSAP)
    table.gc.trigger()

    table.get_logger().info('Waiting for 10 seconds to let all nodes be ready {}'.format(table))
    time.sleep(10)

    rclpy.spin(table)
    rclpy.shutdown()
