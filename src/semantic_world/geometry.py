from __future__ import annotations

from abc import ABC
from dataclasses import dataclass, field
from functools import cached_property

import trimesh

from .spatial_types import TransformationMatrix
from .utils import IDGenerator

id_generator = IDGenerator()

@dataclass
class Color:
    """
    Dataclass for storing rgba_color as an RGBA value.
    The values are stored as floats between 0 and 1.
    The default rgba_color is white.
    """
    R: float = 1
    """
    Red value of the color.
    """

    G: float = 1
    """
    Green value of the color.
    """

    B: float = 1
    """
    Blue value of the color.
    """

    A: float = 1
    """
    Opacity of the color.
    """

@dataclass
class Scale:
    """
    Dataclass for storing the scale of geometric objects.
    """

    x: float = 1.
    """
    The scale in the x direction.
    """

    y: float = 1.
    """
    The scale in the y direction.
    """

    z: float = 1.
    """
    The scale in the z direction.
    """

@dataclass
class Shape(ABC):
    """
    Base class for all shapes in the world.
    """
    origin: TransformationMatrix

    @property
    def mesh(self) -> trimesh.Trimesh:
        """
        The mesh object of the shape.
        This should be implemented by subclasses.
        """
        raise NotImplementedError("Subclasses must implement the mesh property.")

@dataclass
class Mesh(Shape):
    """
    A mesh shape.
    """

    filename: str = ""
    """
    Filename of the mesh.
    """

    scale: Scale = field(default_factory=Scale)
    """
    Scale of the mesh.
    """

    @cached_property
    def mesh(self) -> trimesh.Trimesh:
        """
        The mesh object.
        """
        return trimesh.load_mesh(self.filename)

    def __hash__(self):
        """
        Returns a unique hash for the shape based on its id.
        """
        return id(self)



@dataclass
class Primitive(Shape):
    """
    A primitive shape.
    """
    color: Color = field(default_factory=Color)


@dataclass
class Sphere(Primitive):
    """
    A sphere shape.
    """

    radius: float = 0.5
    """
    Radius of the sphere.
    """

    @property
    def mesh(self) -> trimesh.Trimesh:
        """
        Returns a trimesh object representing the sphere.
        """
        return trimesh.creation.icosphere(subdivisions=2, radius=self.radius)

    def __hash__(self):
        """
        Returns a unique hash for the shape based on its id.
        """
        return id(self)


@dataclass
class Cylinder(Primitive):
    """
    A cylinder shape.
    """
    width: float = 0.5
    height: float = 0.5

    @property
    def mesh(self) -> trimesh.Trimesh:
        """
        Returns a trimesh object representing the cylinder.
        """
        return trimesh.creation.cylinder(radius=self.width / 2, height=self.height, sections=16)

    def __hash__(self):
        """
        Returns a unique hash for the shape based on its id.
        """
        return id(self)



@dataclass
class Box(Primitive):
    """
    A box shape. Pivot point is at the center of the box.
    """
    scale: Scale = field(default_factory=Scale)

    @property
    def mesh(self) -> trimesh.Trimesh:
        """
        Returns a trimesh object representing the box.
        The box is centered at the origin and has the specified scale.
        """
        return trimesh.creation.box(extents=(self.scale.x, self.scale.y, self.scale.z))

    def __hash__(self):
        """
        Returns a unique hash for the shape based on its id.
        """
        return id(self)


