from dataclasses import dataclass, field

from semantic_world.world import View, Body


@dataclass(unsafe_hash=True)
class Handle(View):
    body: Body


@dataclass(unsafe_hash=True)
class Container(View):
    body: Body


@dataclass(unsafe_hash=True)
class Drawer(View):
    container: Container
    handle: Handle


@dataclass
class Cabinet(View):
    container: Container
    drawers: list[Drawer] = field(default_factory=list)

    def __hash__(self):
        return hash((self.__class__.__name__, self.container))