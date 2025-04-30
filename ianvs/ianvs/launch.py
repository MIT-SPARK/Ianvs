import launch_ros.actions
from launch.frontend import expose_action, Entity, Parser


@expose_action("custom_node")
class CustomNode(launch_ros.actions.Node):
    """Custom node action that handles virtual environments."""

    def __init__(self, *, executable, **kwargs):
        """Set up the node."""
        print(executable)
        super().__init__(executable=executable, **kwargs)

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        _, kwargs = super().parse(entity, parser)

        env = entity.get_attr("pyenv", optional=False)
        print(env)

        return cls, kwargs
