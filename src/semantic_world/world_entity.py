from __future__ import annotations

from collections import deque
from collections.abc import Iterable
from dataclasses import dataclass, field, fields
from functools import reduce
from typing import List, Optional, TYPE_CHECKING, Set, get_args, get_type_hints
import numpy as np
from numpy import ndarray

from .geometry import Shape, BoundingBox, BoundingBoxCollection
from dataclasses import dataclass, field
from typing import List, Optional, TYPE_CHECKING

from .geometry import Shape
from .prefixed_name import PrefixedName
from .spatial_types.spatial_types import TransformationMatrix, Expression, Point3
from .spatial_types import spatial_types as cas
from .spatial_types.spatial_types import TransformationMatrix, Expression
from .types import NpMatrix4x4
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
            self.index = self._world.kinematic_structure.add_node(self)

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
    def bounding_box_collection(self) -> BoundingBoxCollection:
        """
        Return the bounding box collection of the link with the given name.
        This method computes the bounding box of the link in world coordinates by transforming the local axis-aligned
        bounding boxes of the link's geometry to world coordinates.

        Note: These bounding boxes may not be disjoint, however the random events library always makes them disjoint. If
        this is the case, and we feed he non-disjoint bounding boxes into the gcs, it may trigger unexpected behavior.

        :return: A BoundingBoxCollection containing the bounding boxes of the link's geometry in world
        """
        world = self._world
        body_transform: ndarray = world.compute_forward_kinematics_np(world.root, self)
        world_bboxes = []

        for shape in self.collision:
            shape_transform: ndarray = shape.origin.to_np()

            world_transform: ndarray = body_transform @ shape_transform
            body_pos = world_transform[:3, 3]
            body_rotation_matrix = world_transform[:3, :3]

            local_bb: BoundingBox = shape.as_bounding_box()

            # Get all 8 corners of the BB in link-local space
            corners = np.array([corner.to_np()[:3] for corner in local_bb.get_points()])  # shape (8, 3)

            # Transform each corner to world space: R * corner + T
            transformed_corners = (corners @ body_rotation_matrix.T) + body_pos

            # Compute world-space bounding box from transformed corners
            min_corner = np.min(transformed_corners, axis=0)
            max_corner = np.max(transformed_corners, axis=0)

            world_bb = BoundingBox.from_min_max(Point3.from_xyz(*min_corner), Point3.from_xyz(*max_corner))
            world_bboxes.append(world_bb)

        return BoundingBoxCollection(world_bboxes)


    @property
    def global_pose(self) -> NpMatrix4x4:
        """
        Computes the pose of the body in the world frame.
        :return: 4x4 transformation matrix.
        """
        return self._world.compute_forward_kinematics_np(self._world.root, self)


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
        new_link.index = body.index
        return new_link

@dataclass
class View(WorldEntity):
    """
    Represents a view on a set of bodies in the world.

    This class can hold references to certain bodies that gain meaning in this context.
    """

    @property
    def aggregated_bodies(self) -> Set[Body]:
        """
        Recursively traverses the view and its attributes to find all bodies contained within it.

        :return: A set of bodies that are part of this view.
        """
        bodies: Set[Body] = set()
        visited: Set[int] = set()
        stack: deque = deque([self])

        # Use a stack to traverse the view and its attributes
        while stack:
            obj = stack.pop()
            oid = id(obj)
            if oid in visited:
                continue
            visited.add(oid)

            match obj:
                # Bodies are aggregated directly
                case Body():
                    bodies.add(obj)

                # Views are traversed recursively
                case View():
                    for f in fields(obj):
                        value = getattr(obj, f.name)
                        if isinstance(value, Body):
                            bodies.add(value)
                        elif isinstance(value, View):
                            stack.append(value)
                        elif isinstance(value, (list, set)):
                            stack.extend(value)

                # Iterables are traversed
                case Iterable() if not isinstance(obj, (str, bytes, bytearray)):
                    stack.extend(obj)
        return bodies

    def as_bounding_box_collection(self) -> BoundingBoxCollection:
        """
        Returns a bounding box collection that contains the bounding boxes of all bodies in this view.
        """
        bbs = reduce(
            lambda accumulator, bb_collection: accumulator.merge(bb_collection),
            (body.bounding_box_collection for body in self.aggregated_bodies if body.has_collision())
        )
        return bbs


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

    origin_expression: TransformationMatrix = None
    """
    A symbolic expression describing the origin of the connection.
    """

    def __post_init__(self):
        if self.origin_expression is None:
            self.origin_expression = TransformationMatrix()
        self.origin_expression.reference_frame = self.parent.name
        self.origin_expression.child_frame = self.child.name

    def __hash__(self):
        return hash((self.parent, self.child))

    def __eq__(self, other):
        return self.name == other.name

    @property
    def name(self):
        return PrefixedName(f'{self.parent.name.name}_T_{self.child.name.name}', prefix=self.child.name.prefix)

    @property
    def origin(self) -> NpMatrix4x4:
        """
        :return: The relative transform between the parent and child frame.
        """
        return self._world.compute_forward_kinematics_np(self.parent, self.child)

    # @lru_cache(maxsize=None)
    def origin_as_position_quaternion(self) -> Expression:
        position = self.origin_expression.to_position()[:3]
        orientation = self.origin_expression.to_quaternion()
        return cas.vstack([position, orientation]).T
