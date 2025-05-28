from dataclasses import dataclass, field

from typing_extensions import List

from semantic_world.world import View, Body


@dataclass(unsafe_hash=True)
class Handle(View):
    body: Body


@dataclass(unsafe_hash=True)
class Container(View):
    body: Body


@dataclass
class Door(View):  # Door has a Footprint
    """
    Door in a body that has a Handle and can open towards or away from the user.
    """
    handle: Handle
    body: Body


################################


@dataclass(unsafe_hash=True)
class Components(View):
    ...


@dataclass(unsafe_hash=True)
class Furniture(View):
    ...


#################### subclasses von Components


@dataclass(unsafe_hash=True)
class Door(Components):
    body: Body
    handle: Handle


@dataclass(unsafe_hash=True)
class Drawer(Components):
    container: Container
    handle: Handle


############################### subclasses to Furniture
@dataclass
class Cupboard(Furniture):
    ...


############################### subclasses to Cupboard
@dataclass(unsafe_hash=True)
class Cabinet(Cupboard):
    container: Container
    drawers: list[Drawer] = field(default_factory=list, hash=False)


@dataclass
class Wardrobe(Cupboard):
    doors: List[Door] = field(default_factory=list)
