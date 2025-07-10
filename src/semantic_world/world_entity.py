from __future__ import annotations

from dataclasses import dataclass, field
from functools import lru_cache
from typing import List, Optional, TYPE_CHECKING, Tuple

import numpy as np
from trimesh.proximity import closest_point, nearby_faces
from trimesh.sample import sample_surface
from scipy.stats import geom

import numpy as np

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
            self.index = self._world.kinematic_structure.add_node(self)

    def __hash__(self):
        return hash(self.name)

    def __eq__(self, other):
        return self.name == other.name and self._world is other._world

    def has_collision(self) -> bool:
        return len(self.collision) > 0

    def compute_closest_points_multi(self, others: list[Body], sample_size=25) -> Tuple[
        np.ndarray, np.ndarray, np.ndarray]:
        """
        Computes the closest points to each given body respectively.

        :param others: The list of bodies to compute the closest points to.
        :param sample_size: The number of samples to take from the surface of the other bodies.
        :return: A tuple containing: The points on the self body, the points on the other bodies, and the distances. All points are in the of this body.
        """

        @lru_cache(maxsize=None)
        def evaluated_geometric_distribution(n: int) -> np.ndarray:
            """
            Evaluates the geometric distribution for a given number of samples.
            :param n: The number of samples to evaluate.
            :return: An array of probabilities for each sample.
            """
            return geom.pmf(np.arange(1, n + 1), 0.5)

        query_points = []
        for other in others:
            # Calculate the closest vertex on this body to the other body
            closest_vert_id = \
                self.collision[0].mesh.kdtree.query(
                    (self._world.compute_forward_kinematics_np(self, other) @ other.collision[0].origin.to_np())[:3, 3],
                    k=1)[1]
            closest_vert = self.collision[0].mesh.vertices[closest_vert_id]

            # Compute the closest faces on the other body to the closes vertex
            faces = nearby_faces(other.collision[0].mesh,
                                 [(self._world.compute_forward_kinematics_np(other, self) @ self.collision[
                                     0].origin.to_np())[:3, 3] + closest_vert])[0]
            face_weights = np.zeros(len(other.collision[0].mesh.faces))

            # Assign weights to the faces based on a geometric distribution
            face_weights[faces] = evaluated_geometric_distribution(len(faces))

            # Sample points on the surface of the other body
            q = sample_surface(other.collision[0].mesh, sample_size, face_weight=face_weights, seed=420)[0]
            # Make 4x4 transformation matrix from points
            points = np.tile(np.eye(4, dtype=np.float32), (len(q), 1, 1))
            points[:, :3, 3] = q

            # Transform from the mesh to the other mesh
            transform = np.linalg.inv(self.collision[0].origin.to_np()) @  self._world.compute_forward_kinematics_np(self, other) @ other.collision[0].origin.to_np()
            points = points @ transform

            points = points[:, :3, 3]  # Extract the points from the transformation matrix

            query_points.extend(points)

        # Actually compute the closest points
        points, dists = closest_point(self.collision[0].mesh, query_points)[:2]
        # Find the closest points for each body out of all the sampled points
        points = np.array(points).reshape(len(others), sample_size, 3)
        dists = np.array(dists).reshape(len(others), sample_size)
        dist_min = np.min(dists, axis=1)
        points_min_self = points[np.arange(len(others)), np.argmin(dists, axis=1), :]
        points_min_other = np.array(query_points).reshape(len(others), sample_size, 3)[np.arange(len(others)),
                           np.argmin(dists, axis=1), :]
        return points_min_self, points_min_other, dist_min

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
    def global_pose(self) -> np.ndarray:
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
    def origin(self) -> np.ndarray:
        """
        :return: The relative transform between the parent and child frame.
        """
        return self._world.compute_forward_kinematics_np(self.parent, self.child)

    # @lru_cache(maxsize=None)
    def origin_as_position_quaternion(self) -> Expression:
        position = self.origin_expression.to_position()[:3]
        orientation = self.origin_expression.to_quaternion()
        return cas.vstack([position, orientation]).T
