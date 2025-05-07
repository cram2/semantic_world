from typing import List

from .world import Connection

class FreeVariable:
    """
    Stub class for free variables.
    """
    ...

class FixedConnection(Connection):
    """
    A connection that has 0 degrees of freedom.
    """

class MoveableConnection(Connection):
    """
    Base class for moveable connections.
    """
    free_variables: List[FreeVariable]


class PrismaticConnection(MoveableConnection):
    ...

class RevoluteConnection(MoveableConnection):
    ...