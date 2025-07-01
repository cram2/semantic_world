from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, TYPE_CHECKING

from .geometry import Shape
from .prefixed_name import PrefixedName
from .spatial_types.spatial_types import TransformationMatrix, Expression
from .spatial_types import spatial_types as cas
from .utils import IDGenerator

if TYPE_CHECKING:
    from .world import World

id_generator = IDGenerator()


@dataclass(unsafe_hash=True)
class WorldEntity:
    """
    A class representing an entity in the world.
    """

    _world: Optional[World] = field(default=None, repr=False, kw_only=True, hash=False)
    """
    The backreference to the world this entity belongs to.
    """

    _views: List[View] = field(default_factory=list, init=False, repr=False, hash=False)
    """
    The views this entity is part of.
    """


@dataclass
class Body(WorldEntity):
    """
    Represents a body in the world.
    A body is a semantic atom, meaning that it cannot be decomposed into meaningful smaller parts.
    """

    name: PrefixedName
    """
    The name of the link. Must be unique in the world.
    If not provided, a unique name will be generated.
    """

    visual: List[Shape] = field(default_factory=list, repr=False)
    """
    List of shapes that represent the visual appearance of the link.
    The poses of the shapes are relative to the link.
    """

    collision: List[Shape] = field(default_factory=list, repr=False)
    """
    List of shapes that represent the collision geometry of the link.
    The poses of the shapes are relative to the link.
    """

    index: Optional[int] = field(default=None, init=False)
    """
    The index of the entity in `_world.kinematic_structure`.
    """

    def __post_init__(self):
        if not self.name:
            self.name = PrefixedName(f"body_{id_generator(self)}")

        if self._world is not None:
            self._world.kinematic_structure.add_body(self)

    def __hash__(self):
        return hash(self.name)

    def __eq__(self, other):
        return self.name == other.name and self._world is other._world

    def has_collision(self) -> bool:
        return len(self.collision) > 0

    @property
    def child_bodies(self) -> List[Body]:
        """
        Returns the child bodies of this body.
        """
        return self._world.compute_child_bodies(self)

    @property
    def parent_body(self) -> Body:
        """
        Returns the parent body of this body.
        """
        return self._world.compute_parent_body(self)

    @property
    def parent_connection(self) -> Connection:
        """
        Returns the parent connection of this body.
        """
        return self._world.compute_parent_connection(self)

    @classmethod
    def from_body(cls, body: Body):
        """
        Creates a new link from an existing link.
        """
        new_link = cls(body.name, body.visual, body.collision)
        new_link._world = body._world
        return new_link

@dataclass
class View(WorldEntity):
    """
    Represents a view on a set of bodies in the world.

    This class can hold references to certain bodies that gain meaning in this context.
    """

@dataclass
class RootedView(View):
    """
    Represents a view that is rooted in a specific body.
    """
    root: Body = field(default_factory=Body)

@dataclass
class EnvironmentView(View):
    """
    Represents a view of the environment.
    """


@dataclass
class Connection(WorldEntity):
    """
    Represents a connection between two bodies in the world.
    """

    parent: Body
    """
    The parent body of the connection.
    """

    child: Body
    """
    The child body of the connection.
    """

    origin: TransformationMatrix = None
    """
    The origin of the connection.
    """

    def __post_init__(self):
        if self.origin is None:
            self.origin = TransformationMatrix()
        self.origin.reference_frame = self.parent.name
        self.origin.child_frame = self.child.name

    def __hash__(self):
        return hash((self.parent, self.child))

    def __eq__(self, other):
        return self.name == other.name

    @property
    def name(self):
        return PrefixedName(f'{self.parent.name.name}_T_{self.child.name.name}', prefix=self.child.name.prefix)

    # @lru_cache(maxsize=None)
    def origin_as_position_quaternion(self) -> Expression:
        position = self.origin.to_position()[:3]
        orientation = self.origin.to_quaternion()
        return cas.vstack([position, orientation]).T
