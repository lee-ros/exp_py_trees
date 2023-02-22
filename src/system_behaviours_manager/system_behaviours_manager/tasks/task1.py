import py_trees

from system_behaviours_manager.common.system_flags import SystemFlags


class Task1(py_trees.behaviour.Behaviour):
    def eternal_guard(blackboard: py_trees.blackboard.Client) -> bool:
        if (
            blackboard.system_state
            ==
            SystemFlags.HAS_FLAG1 | SystemFlags.HAS_FLAG3
        ):
            return True
        return False
        
    def __init__(self, name: str) -> None:
        super(Task1, self).__init__(name)
        
    def setup(self, **kwargs: int) -> None:
        self.logger.debug('setup task1')
    
    def initialise(self) -> None:
        self.logger.debug('initialize task1')
    
    def update(self) -> py_trees.common.Status:
        self.logger.debug('update on task1')
        return py_trees.common.Status.RUNNING