from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Dict, Any, List

import numpy as np
from probabilistic_model.probabilistic_circuit.rx.helper import uniform_measure_of_event

from ..geometry import BoundingBoxCollection
from ..prefixed_name import PrefixedName
from ..spatial_types import Point3
from ..variables import SpatialVariables
from ..world import View, Body


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


@dataclass(unsafe_hash=True)
class Handle(View):
    body: Body

    def __post_init__(self):
        if self.name is None:
            self.name = PrefixedName(str(self.body.name), self.__class__.__name__)


@dataclass(unsafe_hash=True)
class Container(View):
    body: Body

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


@dataclass(unsafe_hash=True)
class Components(View): ...


@dataclass(unsafe_hash=True)
class Furniture(View): ...


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
class DoubleDoor(EntryWay):
    left_door : Door
    right_door : Door

    def __post_init__(self):
        if self.name is None:
            self.name = PrefixedName(str(self.body.name), self.__class__.__name__)


@dataclass(unsafe_hash=True)
class Drawer(Components):
    container: Container
    handle: Handle

    affordances: Optional[Dict[str, Any]] = None
    manipulation_properties: Optional[Dict[str, Any]] = None
    state_information: Optional[Dict[str, Any]] = None
    grasp_poses: Optional[Any] = None
    effects: Optional[Dict[str, Any]] = None
    description: Optional[str] = None
    joint_type: Optional[str] = None
    joint_sim_name: Optional[str] = None

    def __post_init__(self):
        if self.name is None:
            self.name = self.container.name


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


@dataclass(unsafe_hash=True)
class Cabinet(Cupboard):
    body: Body
    doors: List[Door] = field(default_factory=list, hash=False)
    drawers: List[Drawer] = field(default_factory=list, hash=False)

    affordances: Optional[Dict[str, Any]] = None
    manipulation_properties: Optional[Dict[str, Any]] = None
    state_information: Optional[Dict[str, Any]] = None
    grasp_poses: Optional[Any] = None
    effects: Optional[Dict[str, Any]] = None
    description: Optional[str] = None
    joint_type: Optional[str] = None
    joint_sim_name: Optional[str] = None

    def __post_init__(self):
        if self.name is None:
            self.name = PrefixedName(str(self.body.name), self.__class__.__name__)


@dataclass
class Wardrobe(Cupboard):
    doors: List[Door] = field(default_factory=list)


@dataclass(unsafe_hash=True)
class Fridge(View):
    body: Body
    left_door: Door
    right_door: Door
    drawers: List[Drawer] = field(default_factory=list, hash=False)

    affordances: Optional[Dict[str, Any]] = None
    manipulation_properties: Optional[Dict[str, Any]] = None
    state_information: Optional[Dict[str, Any]] = None
    grasp_poses: Optional[Any] = None
    effects: Optional[Dict[str, Any]] = None
    description: Optional[str] = None
    joint_type: Optional[str] = None
    joint_sim_name: Optional[str] = None

    def __post_init__(self):
        if self.name is None:
            self.name = PrefixedName(str(self.body.name), self.__class__.__name__)


@dataclass(unsafe_hash=True)
class Microwave(View):
    body: Body
    door: Door
    handle: Handle

    affordances: Optional[Dict[str, Any]] = None
    manipulation_properties: Optional[Dict[str, Any]] = None
    state_information: Optional[Dict[str, Any]] = None
    grasp_poses: Optional[Any] = None
    effects: Optional[Dict[str, Any]] = None
    description: Optional[str] = None
    joint_type: Optional[str] = None
    joint_sim_name: Optional[str] = None

    def __post_init__(self):
        if self.name is None:
            self.name = PrefixedName(str(self.body.name), self.__class__.__name__)


@dataclass(unsafe_hash=True)
class Oven(View):
    body: Body
    door: Door
    handle: Handle

    affordances: Optional[Dict[str, Any]] = None
    manipulation_properties: Optional[Dict[str, Any]] = None
    state_information: Optional[Dict[str, Any]] = None
    grasp_poses: Optional[Any] = None
    effects: Optional[Dict[str, Any]] = None
    description: Optional[str] = None
    joint_type: Optional[str] = None
    joint_sim_name: Optional[str] = None

    def __post_init__(self):
        if self.name is None:
            self.name = PrefixedName(str(self.body.name), self.__class__.__name__)


@dataclass(unsafe_hash=True)
class Knob(View):
    body: Body

    def __post_init__(self):
        if self.name is None:
            self.name = PrefixedName(str(self.body.name), self.__class__.__name__)


@dataclass(unsafe_hash=True)
class Stove(View):
    body: Body
    knobs: List[Knob] = field(default_factory=list, hash=False)

    affordances: Optional[Dict[str, Any]] = None
    manipulation_properties: Optional[Dict[str, Any]] = None
    state_information: Optional[Dict[str, Any]] = None
    grasp_poses: Optional[Any] = None
    effects: Optional[Dict[str, Any]] = None
    description: Optional[str] = None
    joint_type: Optional[str] = None
    joint_sim_name: Optional[str] = None

    def __post_init__(self):
        if self.name is None:
            self.name = PrefixedName(str(self.body.name), self.__class__.__name__)


@dataclass(unsafe_hash=True)
class Faucet(View):
    body: Body

    def __post_init__(self):
        if self.name is None:
            self.name = PrefixedName(str(self.body.name), self.__class__.__name__)


@dataclass(unsafe_hash=True)
class Sink(View):
    body: Body
    faucet: Faucet
    handles: List[Handle] = field(default_factory=list, hash=False)

    affordances: Optional[Dict[str, Any]] = None
    manipulation_properties: Optional[Dict[str, Any]] = None
    state_information: Optional[Dict[str, Any]] = None
    grasp_poses: Optional[Any] = None
    effects: Optional[Dict[str, Any]] = None
    description: Optional[str] = None
    joint_type: Optional[str] = None
    joint_sim_name: Optional[str] = None

    def __post_init__(self):
        if self.name is None:
            self.name = PrefixedName(str(self.body.name), self.__class__.__name__)


@dataclass(unsafe_hash=True)
class Dishwasher(View):
    body: Body
    door: Optional[Door] = None
    handle: Optional[Handle] = None

    affordances: Optional[Dict[str, Any]] = None
    manipulation_properties: Optional[Dict[str, Any]] = None
    state_information: Optional[Dict[str, Any]] = None
    grasp_poses: Optional[Any] = None
    effects: Optional[Dict[str, Any]] = None
    description: Optional[str] = None
    joint_type: Optional[str] = None
    joint_sim_name: Optional[str] = None

    def __post_init__(self):
        if self.name is None:
            self.name = PrefixedName(str(self.body.name), self.__class__.__name__)


@dataclass(unsafe_hash=True)
class Counter(View):
    body: Body
    containers: List[Container] = field(default_factory=list, hash=False)

    affordances: Optional[Dict[str, Any]] = None
    manipulation_properties: Optional[Dict[str, Any]] = None
    state_information: Optional[Dict[str, Any]] = None
    grasp_poses: Optional[Any] = None
    effects: Optional[Dict[str, Any]] = None
    description: Optional[str] = None
    joint_type: Optional[str] = None
    joint_sim_name: Optional[str] = None

    def __post_init__(self):
        if self.name is None:
            self.name = PrefixedName(str(self.body.name), self.__class__.__name__)


@dataclass(unsafe_hash=True)
class CoffeeMachine(View):
    body: Body
    handles: List[Handle] = field(default_factory=list, hash=False)

    affordances: Optional[Dict[str, Any]] = None
    manipulation_properties: Optional[Dict[str, Any]] = None
    state_information: Optional[Dict[str, Any]] = None
    grasp_poses: Optional[Any] = None
    effects: Optional[Dict[str, Any]] = None
    description: Optional[str] = None
    joint_type: Optional[str] = None
    joint_sim_name: Optional[str] = None

    def __post_init__(self):
        if self.name is None:
            self.name = PrefixedName(str(self.body.name), self.__class__.__name__)


@dataclass(unsafe_hash=True)
class Hood(View):
    body: Body

    def __post_init__(self):
        if self.name is None:
            self.name = PrefixedName(str(self.body.name), self.__class__.__name__)


@dataclass
class Wall(View):
    body: Body
    doors: List[Door] = field(default_factory=list)

    def __post_init__(self):
        if self.name is None:
            self.name = self.body.name
