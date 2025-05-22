from dataclasses import dataclass

from ..world import Body
from ..world_entity import View


@dataclass
class Container(View):
    body: Body

@dataclass
class Handle(View):
    body: Body

@dataclass
class Draw(View):
    handle: Handle
    container: Container