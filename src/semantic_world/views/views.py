from dataclasses import dataclass

from ..world import View, Body

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