import rclpy
import py_trees
import py_trees_ros
from std_msgs.msg import UInt32

from system_behaviours_manager.tasks import Task1, Task2

def create_root() -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Parallel(
        name='root',
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
    )
    
    topics2BB = py_trees.composites.Sequence('topics2BB', memory=True)
    system_state2BB = py_trees_ros.subscribers.ToBlackboard(
        'system_state2BB',
        '/system_state',
        UInt32,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        initialise_variables={'system_state': 0},
        blackboard_variables={'system_state': 'data'}
    )
        
    tasks = py_trees.composites.Selector('tasks', memory=False)
    task1 = py_trees.decorators.EternalGuard(
                name='task1_guard',
                condition=Task1.eternal_guard,
                blackboard_keys={'system_state'},
                child=Task1('task1')
            )
    task2 = py_trees.decorators.EternalGuard(
                name='task2_guard',
                condition=Task2.eternal_guard,
                blackboard_keys={'system_state'},
                child=Task2('task2')
            )
    idle = py_trees.behaviours.Running('idle')
    
    root.add_children([
        topics2BB,
        tasks,
    ])
    topics2BB.add_children([
        system_state2BB,
    ])
    tasks.add_children([
        task1,
        task2,
        idle,
    ])
    
    return root

def main():
    py_trees.logging.level = py_trees.logging.level.INFO
    py_trees.blackboard.Blackboard.enable_activity_stream(100)
    
    rclpy.init()
    
    root = create_root()
    tree = py_trees_ros.trees.BehaviourTree(root)
    tree.add_visitor(
        py_trees.visitors.DisplaySnapshotVisitor(
            display_blackboard=True,
            display_activity_stream=True
        )
    )
    
    try:
        tree.setup(node_name='ros_behaviour_tree', timeout=15)
    except py_trees_ros.exceptions.TimedOutError:
        tree.shutdown()
        rclpy.try_shutdown()
    except KeyboardInterrupt:
        tree.shutdown()
        rclpy.try_shutdown()
        
    tree.tick_tock(period_ms=500)
    
    try:
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()