from dataclasses import dataclass, field

from typing_extensions import List

from semantic_world.world import View, Body

#from __future__ import annotations


################################ Groups
@dataclass(unsafe_hash=True)
class Components(View):
    ...


@dataclass(unsafe_hash=True)
class Furniture(View):
    ...

@dataclass
class Cupboard(View):
    """
    A closed storage furniture piece with doors.
    """
    ...

@dataclass(unsafe_hash=True)
class Appliances(View):
    """
    Represents a collection of home appliances.
    """
    ...

##################### Detailed stuff
@dataclass(unsafe_hash=True)
class Surface(View):
    body: Body

@dataclass(unsafe_hash=True)
class Leg(View):
    body: Body

@dataclass(unsafe_hash=True)
class ArmRest(View):
    body: Body

@dataclass(unsafe_hash=True)
class Base(View):           # Sitting area
    body: Body

@dataclass(unsafe_hash=True)
class Cushion(View):
    body: Body

@dataclass(unsafe_hash=True)
class Sides(View):
    body: Body

@dataclass(unsafe_hash=True)
class Countertop(View):
    body: Body
#############################

@dataclass(unsafe_hash=True)
class Roots(View):
    body: Body

@dataclass(unsafe_hash=True)
class Walls(View):
    body: Body

@dataclass(unsafe_hash=True)
class Windows(View):
    body: Body

@dataclass(unsafe_hash=True)
class Container(View):
    """
    A body that can contain other bodies
    """
    body: Body

#################### subclasses von Components
@dataclass(unsafe_hash=True)
class Handle(View):
    """
    A body that is usually attached to a door or drawer
    """
    body: Body

@dataclass(unsafe_hash=True)
class Door(Components):
    body: Body
    handle: Handle


@dataclass(unsafe_hash=True)
class Drawer(Components):
    container: Container
    handle: Handle

@dataclass(unsafe_hash=True)
class Hotplates(Components):
    body: Body

@dataclass(unsafe_hash=True)
class Cooktop(Components):
    body: Body

@dataclass(unsafe_hash=True)
class DetailedCooktop(Components):
    cooktop: Cooktop
    hotplates: list[Hotplates] = field(default_factory=list, hash=False)

@dataclass(unsafe_hash=True)
class Sink(Components):
    body: Body

############################### subclasses to Furniture
@dataclass(unsafe_hash=True)
class Table(Furniture):
    body: Body

@dataclass(unsafe_hash=True)
class DetailedTable(View):
    surface: Surface
    legs: tuple[Leg, ...]   ##legs: List[Leg] = field(default_factory=list)

@dataclass(unsafe_hash=True)
class Armchair(Furniture):
    body: Body

@dataclass(unsafe_hash=True)
class DetailedArmchair(Furniture):
    base: Base
    armrest: tuple[ArmRest, ...]
    legs: tuple[Leg, ...]

@dataclass(unsafe_hash=True)
class Sofa(Furniture):
    body: Body

@dataclass(unsafe_hash=True)
class DetailedSofa(Furniture):
    base: Base
    armrest: tuple[ArmRest, ...]
    legs: tuple[Leg, ...]
    cushion: tuple[Cushion, ...]



############################### subclasses to Cupboard
@dataclass(unsafe_hash=True)
class Cabinet(Cupboard):
    container: Container
    drawers: list[Drawer] = field(default_factory=list, hash=False)


@dataclass
class Wardrobe(Cupboard):
    doors: List[Door] = field(default_factory=list)


############################################## subclasses to Appliances
@dataclass(unsafe_hash=True)
class Fridge(Appliances):
    body: Body
    door: Door

@dataclass(unsafe_hash=True)
class Oven(Appliances):
    body: Body