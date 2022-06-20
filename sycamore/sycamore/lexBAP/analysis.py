import rclpy
from rclpy.node import Node
from rclpy.node import Node
from choirbot import Pose
from choirbot.utils.position_getter import pose_subscribe

from std_msgs.msg import String
from custom_msgs.msg import Custom

import matplotlib.pyplot as plt

global_x_cordinates = []
global_y_cordinates = []

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Custom,
            '/custom/custom_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data.speed_data)

        point_size = len(msg.path)
        print("Number of points : {}".format(point_size))

        x_cordinates = []
        y_cordinates = []

        for each_point in msg.path:
            x_cordinates.append(each_point.x)
            y_cordinates.append(each_point.y)

        global global_x_cordinates
        global_x_cordinates = x_cordinates

        global global_y_cordinates
        global_y_cordinates = y_cordinates

        plt.plot(global_x_cordinates, global_y_cordinates)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    plt.show()
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    print('gloobal coordinates x {}'.format(global_x_cordinates))