import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class SystemStatePublisher(Node):
    def __init__(self):
        super().__init__('system_state_publisher')
        
        # declare a parameter for system state.
        # this will be updated through a service call from the user.
        self.state_param_ = self.declare_parameter('system_state', 0)
        
        self.update_system_state_ = self.create_service(
            
        )
        self.state_publisher_ = self.create_publisher(
            Int32,
            '~/system_state',
            10
        )
        
        self.state_timer_ = self.create_timer(1, )
    
    def state_timer_cb(self):
        pass
    