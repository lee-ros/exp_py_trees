import py_trees

from system_flags import SystemFlags

class Task1Condition(py_trees.behaviour.Behaviour):
    def __init__(self, name: str) -> None:
        super(Task1Condition, self).__init__(name)
        self.logger.debug('init task1 condition')
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            'system_state', access=py_trees.common.Access.READ
        )
        
    def setup(self, **kwargs: int) -> None:
        self.logger.debug('setup task1 condition')

    def initialise(self) -> None:
        self.logger.debug('initialize task1 condition')
    
    def update(self) -> py_trees.common.Status:
        self.logger.debug('update on task1 condition')
        if not self.blackboard.system_state == SystemFlags.HAS_FLAG1 | SystemFlags.HAS_FLAG3:
            return py_trees.common.Status.FAILURE
        
        return py_trees.common.Status.RUNNING
    
class Task1Action(py_trees.behaviour.Behaviour):
    def __init__(self, name: str) -> None:
        super(Task1Action, self).__init__(name)
        
    def setup(self, **kwargs: int) -> None:
        self.logger.debug('setup task1 action')
    
    def initialise(self) -> None:
        self.logger.debug('initialize task1 action')
    
    def update(self) -> py_trees.common.Status:
        self.logger.debug('update on task1 action')
        return py_trees.common.Status.RUNNING
    
class Task1:     
    def create_condition(name: str) -> Task1Condition:
        return Task1Condition(f'{name}_condition')
    
    def create_action(name: str) -> Task1Action:
        return Task1Action(f'{name}_action')