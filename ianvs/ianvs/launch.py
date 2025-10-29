import pathlib
from typing import Optional, Sequence, Text
from launch.launch_context import LaunchContext
from launch.frontend import expose_action, Entity, Parser
from launch_ros.actions import Node
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitution import Substitution
from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions


class PyenvPrefix(Substitution):
    """Custom substitution for prefixing exec with correct interperter."""

    def __init__(
        self,
        pyenv: SomeSubstitutionsType,
        prefix: Optional[SomeSubstitutionsType] = None,
    ):
        super().__init__()
        self.pyenv = normalize_to_list_of_substitutions(pyenv)
        self.prefix = prefix
        if self.prefix is not None:
            self.prefix = normalize_to_list_of_substitutions(self.prefix)

    @classmethod
    def parse(cls, data: Sequence[SomeSubstitutionsType]):
        raise ValueError("Cannot directly parse!")

    def describe(self) -> Text:
        env_repr = " + ".join([s.describe() for s in self.pyenv])
        if self.prefix is not None:
            prefix_repr = ", pkg=" + " + ".join([s.describe() for s in self.prefix])
        else:
            prefix_repr = ""

        return f"PyenvExec(pyenv={env_repr}{prefix_repr})"

    def perform(self, context: LaunchContext) -> Text:
        pyenv = perform_substitutions(context, self.pyenv)
        if pyenv:
            pyinterp = pathlib.Path(pyenv) / "bin" / "python"
            if not pyinterp.exists():
                raise ValueError(f"Interperter '{pyinterp}' not found for venv '{pyenv}'")
        else:
            pyinterp = ""

        if self.prefix is None:
            return str(pyinterp)

        prefix = perform_substitutions(context, self.prefix)
        return f"{prefix} {pyinterp}"


@expose_action("pyenv_node")
class CustomNode(Node):
    """Custom node action that handles virtual environments."""

    def __init__(self, *, pyenv, prefix=None, **kwargs):
        """Set up the node."""
        super().__init__(prefix=PyenvPrefix(pyenv, prefix), **kwargs)

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        _, kwargs = super().parse(entity, parser)
        env = parser.parse_substitution(entity.get_attr("pyenv", optional=False))
        kwargs["pyenv"] = env
        return cls, kwargs
