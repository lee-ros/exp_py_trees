import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt32

INITIAL_STATE = 0

class SystemStateMockerPublisher(Node):
    def __init__(self):
        super().__init__('system_state_mocker_publisher')
        
        # declare a parameter for the system state.
        # this will be updated through the ros2 cli by the user to mock a change.
        self.state_param_ = self.declare_parameter('system_state', INITIAL_STATE)
        
        self.state_publisher_ = self.create_publisher(
            UInt32,
            '~/system_state',
            10
        )
        
        self.state_timer_ = self.create_timer(1, self.state_timer_cb)
    
    def state_timer_cb(self):
        msg = UInt32()
        msg.data = self.get_parameter('system_state').get_parameter_value().integer_value
        self.state_publisher_.publish(msg)
        
def main():
    rclpy.init()
    
    state_publisher = SystemStateMockerPublisher()
    
    try:
        rclpy.spin(state_publisher)
    except KeyboardInterrupt:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    