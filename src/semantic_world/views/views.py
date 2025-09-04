from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from entity_query_language import symbol
from probabilistic_model.probabilistic_circuit.rx.helper import uniform_measure_of_event
from typing_extensions import List

from ..world_description.geometry import BoundingBoxCollection
from ..datastructures.prefixed_name import PrefixedName
from ..spatial_types import Point3
from ..datastructures.variables import SpatialVariables
from ..world_description.world_entity import View, Body, Region


@dataclass
class HasDrawers:
    """
    A mixin class for views that have drawers.
    """

    drawers: List[Drawer] = field(default_factory=list, hash=False)


@dataclass
class HasDoors:
    """
    A mixin class for views that have doors.
    """

    doors: List[Door] = field(default_factory=list, hash=False)


@symbol
@dataclass(unsafe_hash=True)
class Handle(View):
    body: Body

    def __post_init__(self):
        if self.name is None:
            self.name = PrefixedName(str(self.body.name), self.__class__.__name__)


@symbol
@dataclass(unsafe_hash=True)
class Container(View):
    body: Body

    def __post_init__(self):
        if self.name is None:
            self.name = PrefixedName(str(self.body.name), self.__class__.__name__)


@dataclass(unsafe_hash=True)
class Door(View):  # Door has a Footprint
    """
    Door in a body that has a Handle and can open towards or away from the user.
    """

    handle: Handle
    body: Body

    def __post_init__(self):
        self.name = PrefixedName(str(self.body.name), self.__class__.__name__)


@dataclass(unsafe_hash=True)
class Fridge(View):
    """
    A view representing a fridge that has a door and a body.
    """

    body: Body
    door: Door

    def __post_init__(self):
        if self.name is None:
            self.name = PrefixedName(str(self.body.name), self.__class__.__name__)


@dataclass(unsafe_hash=True)
class Table(View):
    """
    A view that represents a table.
    """

    top: Body
    """
    The body that represents the table's top surface.
    """

    def points_on_table(self, amount: int = 100) -> List[Point3]:
        """
        Get points that are on the table.

        :amount: The number of points to return.
        :returns: A list of points that are on the table.
        """
        area_of_table = BoundingBoxCollection.from_shapes(self.top.collision)
        event = area_of_table.event
        p = uniform_measure_of_event(event)
        p = p.marginal(SpatialVariables.xy)
        samples = p.sample(amount)
        z_coordinate = np.full(
            (amount, 1), max([b.max_z for b in area_of_table]) + 0.01
        )
        samples = np.concatenate((samples, z_coordinate), axis=1)
        return [Point3(*s, reference_frame=self.top) for s in samples]

    def __post_init__(self):
        self.name = self.top.name


################################


@dataclass(unsafe_hash=True)
class Components(View): ...


@dataclass(unsafe_hash=True)
class Furniture(View): ...


@dataclass
class SupportingSurface(View):
    """
    A view that represents a supporting surface.
    """

    region: Region
    """
    The body that represents the supporting surface.
    """

    def __post_init__(self):
        if self.name is None:
            self.name = self.region.name


#################### subclasses von Components


@dataclass(unsafe_hash=True)
class EntryWay(Components):
    body: Body

    def __post_init__(self):
        if self.name is None:
            self.name = PrefixedName(str(self.body.name), self.__class__.__name__)


@dataclass(unsafe_hash=True)
class Door(EntryWay):
    handle: Handle

    def __post_init__(self):
        self.name = self.body.name


@dataclass(unsafe_hash=True)
class Fridge(View):
    body: Body
    door: Door

    def __post_init__(self):
        if self.name is None:
            self.name = PrefixedName(str(self.body.name), self.__class__.__name__)


@dataclass(unsafe_hash=True)
class DoubleDoor(EntryWay):
    left_door: Door
    right_door: Door

    def __post_init__(self):
        if self.name is None:
            self.name = PrefixedName(str(self.body.name), self.__class__.__name__)


@symbol
@dataclass(unsafe_hash=True)
class Drawer(Components):
    container: Container
    handle: Handle

    def __post_init__(self):
        if self.name is None:
            self.name = self.container.name


############################### subclasses to Furniture
@dataclass
class Cupboard(Furniture): ...


@dataclass
class Dresser(Furniture):
    container: Container
    drawers: List[Drawer] = field(default_factory=list, hash=False)
    doors: List[Door] = field(default_factory=list, hash=False)

    def __post_init__(self):
        if self.name is None:
            self.name = self.container.name


############################### subclasses to Cupboard
@dataclass(unsafe_hash=True)
class Cabinet(Cupboard):
    container: Container
    drawers: list[Drawer] = field(default_factory=list, hash=False)

    def __post_init__(self):
        self.name = self.container.name


@dataclass
class Wardrobe(Cupboard):
    doors: List[Door] = field(default_factory=list)


@dataclass
class Room(SupportingSurface):
    """
    A view that represents a closed area with a specific purpose
    """


@dataclass
class Wall(View):
    body: Body
    doors: List[Door] = field(default_factory=list)

    def __post_init__(self):
        if self.name is None:
            self.name = self.body.name
