import py_trees

from system_flags import SystemFlags


class Task2(py_trees.behaviour.Behaviour):
    def eternal_guard(blackboard: py_trees.blackboard.Client) -> bool:
        if (
            blackboard.system_state
            ==
            SystemFlags.HAS_FLAG2
        ):
            return True
        return False
    
    def __init__(self, name: str) -> None:
        super(Task2, self).__init__(name)
        self.logger.debug('init task2')
        
    def setup(self, **kwargs: int) -> None:
        self.logger.debug('setup task2')
    
    def initialise(self) -> None:
        self.logger.debug('initialize task2')
    
    def update(self) -> py_trees.common.Status:
        self.logger.debug('update on task2')
        return py_trees.common.Status.RUNNING
