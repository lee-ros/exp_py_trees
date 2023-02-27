import time
import py_trees
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt32

from system_behaviours_manager.tasks import Task1, Task2
from system_behaviours_manager.common import SystemFlags

TICK_PERIOD = 1

class SystemBehaviourNode(Node):
    def __init__(self):
        super().__init__('system_behaviour_tree')

        self.current_state: SystemFlags = 0
        
        self.system_bt = self._create_bt()
        self.system_bt.setup(timeout=15)
        
        self.state_subscriber_ = self.create_subscription(
            UInt32,
            '~/system_state',
            self._fetch_new_state_cb,
            10
        )
        
        self.tree_tick_timer = self.create_timer(TICK_PERIOD, self.system_bt.tick)

    def _fetch_new_state_cb(self, msg: UInt32):
        self.current_state = SystemFlags(msg.data)

    def _set_system_state_to_blackboard(self, bt: py_trees.trees.BehaviourTree) -> None:
        py_trees.blackboard.Blackboard.set(
            'system_state',
            self.current_state
        )

    def _create_root(self) -> py_trees.behaviour.Behaviour:
        root = py_trees.composites.Selector(
            'root',
            memory=False,
            children=[  
                py_trees.decorators.EternalGuard(
                    name='task1_guard',
                    condition=Task1.eternal_guard,
                    blackboard_keys={'system_state'},
                    child=Task1('task1')
                ),
                py_trees.decorators.EternalGuard(
                    name='task2_guard',
                    condition=Task2.eternal_guard,
                    blackboard_keys={'system_state'},
                    child=Task2('task2')
                ),
                py_trees.behaviours.Running('idle'),
            ]
        )
        
        return root

    def _create_bt(self) -> py_trees.trees.BehaviourTree:
        bt = py_trees.trees.BehaviourTree(self._create_root())
        bt.add_pre_tick_handler(self._set_system_state_to_blackboard)
        bt.visitors.append(
            py_trees.visitors.DisplaySnapshotVisitor(display_blackboard=True, display_activity_stream=True)
        )
        return bt
    
    def tick_tree(self) -> None:
        os.system('clear')
        self.system_bt.tick()
        

def main():
    rclpy.init()
    
    py_trees.logging.level = py_trees.logging.level.DEBUG
    py_trees.blackboard.Blackboard.enable_activity_stream(100)
    
    state_to_blackboard = SystemBehaviourNode()
    
    while True:
        try:
            rclpy.spin(state_to_blackboard)
        except KeyboardInterrupt:
            rclpy.shutdown()
        
    
if __name__ == '__main__':
    main()