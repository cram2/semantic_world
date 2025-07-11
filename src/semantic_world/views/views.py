from dataclasses import dataclass, field

from typing_extensions import List

from semantic_world.world import View, Body

#from __future__ import annotations





@dataclass(unsafe_hash=True)
class Roots(View):
    """
    Represents root-level components or aka Floor
    """
    body: Body

@dataclass(unsafe_hash=True)
class Walls(View):
    """
    Represents wall components or surrounding partitions.
    """
    body: Body

@dataclass(unsafe_hash=True)
class Windows(View):
    """
    Represents window components in the world.
    """
    body: Body

# =======================
# === GROUP DEFINITIONS
# =======================
@dataclass(unsafe_hash=True)
class Components(View):
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

# ===========================
# === COMPONENT SUBCLASSES
# ===========================
@dataclass(unsafe_hash=True)
class Surface(Components):
    """
    Represents a flat surface (e.g., tabletop).
    """
    body: Body

@dataclass(unsafe_hash=True)
class Leg(Components):
    """
    A vertical support component of furniture.
    """
    body: Body

@dataclass(unsafe_hash=True)
class ArmRest(Components):
    """
    A comfortable chair with armrests and backrest.
    """
    body: Body

@dataclass(unsafe_hash=True)
class Base(Components):
    """
    The sitting area of a chair or sofa.
    """
    body: Body

@dataclass(unsafe_hash=True)
class Cushion(Components):
    """
    Cushions are primarily used for comfort, support, and decoration, often found on sofas, chairs, or beds.
    """
    body: Body

@dataclass(unsafe_hash=True)
class Sides(Components):
    """
    Represents the Sides of a structure
    """
    body: Body

@dataclass(unsafe_hash=True)
class Countertop(Components):
    """
    A flat working surface in a kitchen or workspace.
    """
    body: Body

@dataclass(unsafe_hash=True)
class Handle(Components):
    """
    A body that is usually attached to a door or drawer
    """
    body: Body

@dataclass(unsafe_hash=True)
class Container(Components):
    """
    A body that can contain other bodies
    """
    body: Body

@dataclass(unsafe_hash=True)
class Door(Components):
    """
    A hinged or sliding barrier for opening/closing access to a compartment or appliance.
    """
    body: Body
    handle: Handle


@dataclass(unsafe_hash=True)
class Drawer(Components):
    """
    A sliding container housed in furniture, with a handle and container body.
    """
    container: Container
    handle: Handle

@dataclass(unsafe_hash=True)
class Hotplates(Components):
    """
    A heating element typically placed on top of a cooktop.
    """
    body: Body

@dataclass(unsafe_hash=True)
class Sink(Components):
    """
    A sink component typically used in kitchen or bathroom setups.
    """
    body: Body

# ============================
# === FURNITURE SUBCLASSES
# ============================
@dataclass(unsafe_hash=True)
class Table(Furniture):
    """
    A generic table furniture item.
    """
    body: Body

@dataclass(unsafe_hash=True)
class DetailedTable(View):
    """
    A detailed table view with a surface and legs.
    """
    surface: Surface
    legs: tuple[Leg, ...]   ##legs: List[Leg] = field(default_factory=list)

@dataclass(unsafe_hash=True)
class Armchair(Furniture):
    """
    A generic armchair.
    """
    body: Body

@dataclass(unsafe_hash=True)
class DetailedArmchair(Furniture):
    """
    A comfortable chair with armrests and backrest.
    """
    base: Base
    armrest: List[ArmRest] = field(default_factory=list, hash=False)
    legs: List[Leg] = field(default_factory=list, hash=False)

@dataclass(unsafe_hash=True)
class Sofa(Furniture):
    """
    A generic sofa.
    """
    body: Body

@dataclass(unsafe_hash=True)
class DetailedSofa(Furniture):
    """
    A detailed sofa view with base, armrests, legs, and cushions.
    """
    base: Base
    armrest: List[ArmRest] = field(default_factory=list, hash=False)
    legs: List[Leg] = field(default_factory=list, hash=False)
    cushion: List[Cushion] = field(default_factory=list, hash=False)

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


@dataclass
class Wardrobe(Cupboard):
    """
    A cupboard consisting of one or more doors.
    """
    doors: List[Door] = field(default_factory=list)



# =============================
# === APPLIANCE SUBCLASSES
# =============================
@dataclass(unsafe_hash=True)
class Fridge(Appliances):
    """
    A fridge appliance with a body and door.
    """
    body: Body
    door: Door

@dataclass(unsafe_hash=True)
class Oven(Appliances):
    """
    An oven appliance with a body
    """
    body: Body

@dataclass(unsafe_hash=True)
class Cooktop(Appliances):
    """
    A cooking surface appliance, usually with one or more hotplates.
    """
    body: Body

@dataclass(unsafe_hash=True)
class DetailedCooktop(Appliances):
    """
    A cooktop view that includes its connected hotplates.
    """
    cooktop: Cooktop
    hotplates: list[Hotplates] = field(default_factory=list, hash=False)