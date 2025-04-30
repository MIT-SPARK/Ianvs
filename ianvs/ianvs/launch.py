import pathlib
from typing import Optional, Sequence, Text
from launch.launch_context import LaunchContext
from launch.frontend import expose_action, Entity, Parser
from launch_ros.actions import Node
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitution import Substitution
from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions

import importlib.metadata as im


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
        pyinterp = pathlib.Path(pyenv) / "bin" / "python"
        if not pyinterp.exists():
            raise ValueError(f"Interperter '{pyinterp}' not found for venv '{pyenv}'")

        if self.prefix is None:
            return str(pyinterp)

        prefix = perform_substitutions(context, self.prefix)
        return f"{prefix} {pyinterp}"


class PythonExecutable(Substitution):
    """Custom substitution for prefixing exec with correct interperter."""

    def __init__(
        self,
        executable: SomeSubstitutionsType,
        package: Optional[SomeSubstitutionsType] = None,
    ):
        super().__init__()
        self.executable = normalize_to_list_of_substitutions(executable)
        if package is not None:
            self.package = normalize_to_list_of_substitutions(package)
        else:
            self.package = None

    @classmethod
    def parse(cls, data: Sequence[SomeSubstitutionsType]):
        raise ValueError("Cannot directly parse!")

    def describe(self) -> Text:
        exec_repr = " + ".join([s.describe() for s in self.executable])
        if self.package is not None:
            pkg_repr = ", pkg=" + " + ".join([s.describe() for s in self.package])
        else:
            pkg_repr = ""

        return f"PyenvExec(exec={exec_repr}, {pkg_repr})"

    def perform(self, context: LaunchContext) -> Text:
        executable = perform_substitutions(context, self.executable)
        if self.package is None:
            return executable

        package = perform_substitutions(context, self.package)
        import py_pubsub
        print(py_pubsub.__file__)
        pkg_info = im.distribution(package)
        print(pkg_info.entry_points)
        print(f"parsed exec='{executable}', pkg='{package}'")
        return executable


@expose_action("custom_node")
class CustomNode(Node):
    """Custom node action that handles virtual environments."""

    def __init__(self, *, executable, pyenv, package=None, prefix=None, **kwargs):
        """Set up the node."""
        super().__init__(
            executable=PythonExecutable(executable, package),
            prefix=PyenvPrefix(pyenv, prefix),
            package=package,
            **kwargs,
        )

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        _, kwargs = super().parse(entity, parser)
        env = parser.parse_substitution(entity.get_attr("pyenv", optional=False))
        kwargs["pyenv"] = env
        return cls, kwargs
