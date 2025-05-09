from dataclasses import dataclass, field
from typing import List

from .world import Connection

@dataclass
class FreeVariable:
    """
    Stub class for free variables.
    """
    ...

@dataclass
class FixedConnection(Connection):
    """
    A connection that has 0 degrees of freedom.
    """

@dataclass
class MoveableConnection(Connection):
    """
    Base class for moveable connections.
    """
    free_variables: List[FreeVariable] = field(default_factory=list)

@dataclass
class PrismaticConnection(MoveableConnection):
    ...

@dataclass
class RevoluteConnection(MoveableConnection):
    ...