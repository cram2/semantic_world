from dataclasses import dataclass, field

import numpy as np
from probabilistic_model.probabilistic_circuit.rx.helper import uniform_measure_of_event
from typing_extensions import List

from semantic_world.geometry import BoundingBox, BoundingBoxCollection
from semantic_world.spatial_types import Point3
from semantic_world.variables import SpatialVariables
from semantic_world.world import View, Body


@dataclass(unsafe_hash=True)
class Root(View):
    """
    Represents root-level Components
    """
    body: Body

    def __post_init__(self):
        self.name = self.body.name

@dataclass(unsafe_hash=True)
class Wall(View):
    """
    Represents wall Components or surrounding partitions.
    """
    body: Body

    def __post_init__(self):
        self.name = self.body.name

@dataclass(unsafe_hash=True)
class Window(View):
    """
    Represents window Components in the world.
    """
    body: Body

    def __post_init__(self):
        self.name = self.body.name


@dataclass(unsafe_hash=True)
class Container(View):
    body: Body

    def __post_init__(self):
        self.name = self.body.name

# =======================
# === GROUP DEFINITIONS
# =======================
@dataclass(unsafe_hash=True)
class Components(View):                                                   #################################
    """
    Represents structural or functional parts of furniture or appliances.
    """
    ...


@dataclass(unsafe_hash=True)
class Furniture(View):
    """
    Base class for all types of furniture.
    """
    ...

@dataclass(unsafe_hash=True)
class Appliance(View):
    """
    Represents a collection of home appliances.
    """
    ...


@dataclass(unsafe_hash=True)
class Handle(Components):
    """
    A body that is usually attached to a door or drawer
    """
    body: Body

    def __post_init__(self):
        self.name = self.body.name


@dataclass(unsafe_hash=True)
class Container(Components):
    """
    A body that can contain other bodies
    """
    body: Body

    def __post_init__(self):
        self.name = self.body.name


@dataclass(unsafe_hash=True)
class Door(Components):
    """
    A hinged or sliding barrier for opening/closing access to a compartment or appliance.
    """
    body: Body
    handle: Handle

    def __post_init__(self):
        self.name = self.body.name


@dataclass(unsafe_hash=True)
class Drawer(Components):
    """
    A sliding container housed in furniture, with a handle and container body.
    """
    container: Container
    handle: Handle

    def __post_init__(self):
        self.name = self.container.name


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
        z_coordinate = np.full((amount, 1), max([b.max_z for b in area_of_table]) + 0.01)
        samples = np.concatenate((samples, z_coordinate), axis=1)
        return [Point3.from_xyz(*s, reference_frame=self.top) for s in samples]

    def __post_init__(self):
        self.name = self.top.name

#################### subclasses von Components
@dataclass(unsafe_hash=True)
class Surface(Components):
    """
    Represents a flat surface (e.g., tabletop).
    """
    body: Body

    def __post_init__(self):
        self.name = self.body.name

@dataclass(unsafe_hash=True)
class Leg(Components):
    """
    A vertical support Components of furniture.
    """
    body: Body

    def __post_init__(self):
        self.name = self.body.name

@dataclass(unsafe_hash=True)
class ArmRest(Components):
    """
    A comfortable chair with armrests and backrest.
    """
    body: Body

    def __post_init__(self):
        self.name = self.body.name

@dataclass(unsafe_hash=True)
class Base(Components):
    """
    The sitting area of a chair or sofa.
    """
    body: Body

    def __post_init__(self):
        self.name = self.body.name

class Cushion(Components):
    """
    Cushions are primarily used for comfort, support, and decoration, often found on sofas, chairs, or beds.
    """
    body: Body

    def __post_init__(self):
        self.name = self.body.name

@dataclass(unsafe_hash=True)
class Sides(Components):                                                   #################################
    """
    Represents the Sides of a structure
    """
    body: Body

    def __post_init__(self):
        self.name = self.body.name

@dataclass(unsafe_hash=True)
class Countertop(Components):
    """
    A flat working surface in a kitchen or workspace.
    """
    body: Body

    def __post_init__(self):
        self.name = self.body.name


############################### subclasses to Furniture
@dataclass
class Cupboard(Furniture):
    ...

@dataclass(unsafe_hash=True)
class Hotplate(Components):
    """
    A heating element typically placed on top of a cooktop.
    """
    body: Body

    def __post_init__(self):
        self.name = self.body.name

############################### subclasses to Cupboard

@dataclass(unsafe_hash=True)
class Sink(Components):
    """
    A sink Components typically used in kitchen or bathroom setups.
    """
    body: Body

    def __post_init__(self):
        self.name = self.body.name

# ============================
# === FURNITURE SUBCLASSES
# ============================
@dataclass(unsafe_hash=True)
class Table(Furniture):
    """
    A generic table furniture item.
    """
    body: Body

    def __post_init__(self):
        self.name = self.body.name


@dataclass(unsafe_hash=True)
class Armchair(Furniture):
    """
    A generic armchair.
    """
    base: Base
    armrest: List[ArmRest] = field(default_factory=list, hash=False)
    legs: List[Leg] = field(default_factory=list, hash=False)

    def __post_init__(self):
        self.name = self.base.name

@dataclass(unsafe_hash=True)
class Sofa(Furniture):
    """
    A generic sofa.
    """
    base: Base
    armrest: List[ArmRest] = field(default_factory=list, hash=False)
    legs: List[Leg] = field(default_factory=list, hash=False)
    cushion: List[Cushion] = field(default_factory=list, hash=False)

    def __post_init__(self):
        self.name = self.base.name

# =============================
# === CUPBOARD SUBCLASSES
# =============================
@dataclass(unsafe_hash=True)
class Cabinet(Cupboard):
    """
    A cupboard with a container and optional drawers.
    """
    container: Container
    drawers: list[Drawer] = field(default_factory=list, hash=False)

    def __post_init__(self):
        self.name = self.container.name


@dataclass(unsafe_hash=True)
class Wardrobe(Cupboard):
    """
    A cupboard consisting of one or more doors.
    """
    body: Body
    doors: List[Door] = field(default_factory=list)

    def __post_init__(self):
        self.name = self.body.name



# =============================
# === APPLIANCE SUBCLASSES
# =============================
@dataclass(unsafe_hash=True)
class Fridge(Appliance):
    """
    A fridge appliance with a body and door.
    """
    body: Body
    door: Door

    def __post_init__(self):
        self.name = self.body.name

@dataclass(unsafe_hash=True)
class Oven(Appliance):
    """
    An oven appliance with a body
    """
    body: Body

    def __post_init__(self):
        self.name = self.body.name

@dataclass(unsafe_hash=True)
class Cooktop(Appliance):
    """
    A cooking surface appliance, usually with one or more hotplate.
    """
    body: Body
    hotplate: list[Hotplate] = field(default_factory=list, hash=False)

    def __post_init__(self):
        self.name = self.body.name