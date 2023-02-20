import py_trees
import os
import time

from task1 import Task1
from task2 import Task2
from system_flags import SystemFlags


def create_root() -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Selector(
        'root',
        memory=True,
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
        ]
    )
    
    return root

def pre_tick_handler(bt: py_trees.trees.BehaviourTree) -> None:
    py_trees.blackboard.Blackboard.set(
        'system_state',
        pre_tick_handler.states[pre_tick_handler.counter]
    )
    os.system('clear')
    print(f'current system status is: {pre_tick_handler.states[pre_tick_handler.counter]}')
    pre_tick_handler.counter += 1
    if pre_tick_handler.counter == len(pre_tick_handler.states):
        pre_tick_handler.counter = 0

pre_tick_handler.states = [
    SystemFlags.HAS_FLAG1,
    SystemFlags.HAS_FLAG1 | SystemFlags.HAS_FLAG3,
    SystemFlags.HAS_FLAG1 | SystemFlags.HAS_FLAG3,
    SystemFlags.HAS_FLAG2,
    SystemFlags.HAS_FLAG1 | SystemFlags.HAS_FLAG3,
]
pre_tick_handler.counter = 0

def main():
    py_trees.logging.level = py_trees.logging.level.INFO
    py_trees.blackboard.Blackboard.enable_activity_stream(100)
        
    root = create_root()

    bt = py_trees.trees.BehaviourTree(root)
    bt.add_pre_tick_handler(pre_tick_handler)
    bt.visitors.append(
        py_trees.visitors.DisplaySnapshotVisitor(display_blackboard=True, display_activity_stream=True)
    )
    
    bt.setup(timeout=15)
    
    while True:
        try:
            bt.tick()
            py_trees.console.read_single_keypress()
        except KeyboardInterrupt:
            break
        
    
if __name__ == '__main__':
    main()