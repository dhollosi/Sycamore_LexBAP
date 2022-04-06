from .integrator import Integrator
from geometry_msgs.msg import Vector3
import numpy as np


class DoubleIntegrator(Integrator):

    def __init__(self, integration_freq: float, odom_freq: float=None):
        super().__init__(integration_freq, odom_freq)

        # create input subscription
        self.u = np.zeros(3)
        self.current_vel = np.zeros(3)
        self.subscription = self.create_subscription(Vector3, 'u', self.input_callback, 1)
        
        self.get_logger().info('Integrator {} started'.format(self.agent_id))
    
    def input_callback(self, msg):
        # save new input
        self.u = np.array([msg.x, msg.y, msg.z])

    def integrate(self):
        self.current_pos += self.samp_time * self.current_vel
        self.current_vel += self.samp_time * self.u
