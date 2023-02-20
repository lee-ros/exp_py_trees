import py_trees

from system_flags import SystemFlags

class Task2Condition(py_trees.behaviour.Behaviour):
    def __init__(self, name: str) -> None:
        super(Task2Condition, self).__init__(name)
        self.logger.debug('setup task2 condition')
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            'system_state', access=py_trees.common.Access.READ
        )
        
    def setup(self, **kwargs: int) -> None:
        self.logger.debug('setup task2 action condition')
    
    def initialise(self) -> None:
        self.logger.debug('initialize task2 condition')
    
    def update(self) -> py_trees.common.Status:
        self.logger.debug('update on task2 condition')
        if not self.blackboard.system_state == SystemFlags.HAS_FLAG2:
            return py_trees.common.Status.FAILURE
        
        return py_trees.common.Status.RUNNING
    
class Task2Action(py_trees.behaviour.Behaviour):
    def __init__(self, name: str) -> None:
        super(Task2Action, self).__init__(name)
        self.logger.debug('init task2 condition')
        
    def setup(self, **kwargs: int) -> None:
        self.logger.debug('setup task2 action')
    
    def initialise(self) -> None:
        self.logger.debug('initialize task2 action')
    
    def update(self) -> py_trees.common.Status:
        self.logger.debug('update on task2 action')
        return py_trees.common.Status.RUNNING
    
class Task2:     
    def create_condition(name: str) -> Task2Condition:
        return Task2Condition(f'{name}_condition')
    
    def create_action(name: str) -> Task2Action:
        return Task2Action(f'{name}_action')