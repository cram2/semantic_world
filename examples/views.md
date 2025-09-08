---
jupyter:
  jupytext:
    text_representation:
      extension: .md
      format_name: markdown
      format_version: '1.3'
      jupytext_version: 1.17.3
  kernelspec:
    display_name: Python 3
    language: python
    name: python3
---

(semantic_annotations)=
# Semantic Annotations (Views)

Views are semantic annotations to world entities.
For instance, they can be used to say that a certain body should be interpreted as a handle or that a combination of
bodies should be interpreted as a drawer.
Ontologies inspire views. The semantic world overcomes the technical limitations of ontologies by representing
semantic annotations as python classes and use the typing system of python together with EQL for reasoning.
This tutorial shows you how to apply views to the world and how to create your own views.

Used Concepts:
- Factories
- [](creating-custom-bodies)
- [](world-structure-manipulation)
- [Entity Query Language](https://abdelrhmanbassiouny.github.io/entity_query_language/intro.html)

First, let's create a world containing a drawer.

```python
from dataclasses import dataclass
from typing import List

from entity_query_language import entity, an, let

from semantic_world.datastructures.prefixed_name import PrefixedName
from semantic_world.spatial_types.spatial_types import TransformationMatrix
from semantic_world.views.factories import (
    DrawerFactory,
    ContainerFactory,
    HandleFactory,
    Direction,
)
from semantic_world.views.views import Drawer, Handle, Container
from semantic_world.world import World
from semantic_world.world_description.connections import Connection6DoF
from semantic_world.world_description.geometry import Sphere, Scale
from semantic_world.world_description.world_entity import View, Body


world = DrawerFactory(
    name=PrefixedName("drawer"),
    container_factory=ContainerFactory(name=PrefixedName("container")),
    handle_factory=HandleFactory(name=PrefixedName("handle")),
).create()

print(*world.views, sep="\n")
```

The annotations now proof useful when an agent needs to infer information from the world.
For instance, an agent might want to open a drawer. Opening a drawer is done by grasping the drawer handle.
Due to the semantic structure, it is easily possible to access this information for the agent by formulating a query like this

```python
handles = an(entity(let("handle", Handle, world.views)))
print(list(handles.evaluate()))
```

The interlinking of the semantic annotations is done via classes. This is very useful, to filter for more context
in the world. For instance, consider a world that has a second handle that is attached to nothing and hence
shouldn't be used by the agent to open a drawer.


```python

useless_handle = HandleFactory(name=PrefixedName("useless handle")).create()
print(useless_handle.views)
with world.modify_world():
    world.merge_world_at_pose(
        useless_handle, TransformationMatrix.from_xyz_rpy(x=1.0, y=1.0)
    )

```

If we now evaluate the handle query, we see that multiple options exist.

```python
print(*list(an(entity(let("handle", Handle, world.views))).evaluate()), sep="\n")
```

We can refine the handle the agent wants by saying it must belong to a drawer

```python
drawer = let("drawer", Drawer, world.views)
handle = let("handle", Handle, world.views)
print(*list(an(entity(handle, drawer.handle == handle)).evaluate()), sep="\n")
```

Now we will shift the focus to creating new views.
Views define what they are (is-a) through inheritance.
Make sure you follow the (Liskov substitution principle)[https://en.wikipedia.org/wiki/Liskov_substitution_principle] when creating new views that inherit from existing views.
Views define what they relate to via their attribute types.
For instance, we can make an Apple view that classifies a body as an apple.

```python
@dataclass
class Apple(View):
    """A simple custom view declaring that a Body is an Apple."""

    body: Body

    def __post_init__(self):
        # Give the view a default name if none was specified
        if self.name is None:
            self.name = PrefixedName(str(self.body.name), self.__class__.__name__)


world = World()
with world.modify_world():

    root = Body(name=PrefixedName("root"))

    # Create a body with spherical geometry
    apple_body = Body(name=PrefixedName("apple_body"))
    sphere = Sphere(
        radius=0.15, origin=TransformationMatrix(reference_frame=apple_body)
    )
    apple_body.collision = [sphere]
    apple_body.visual = [sphere]

    root_to_apple = Connection6DoF(parent=root, child=apple_body, _world=world)
    world.add_connection(root_to_apple)

    # Declare the body as an Apple view and add it to the world
    apple_view = Apple(body=apple_body, name=PrefixedName("apple"))
    world.add_view(apple_view)

print(world.views)
```

Views can become arbitrary complex. For instance, we can make a box of fruits.

```python
@dataclass
class FruitBox(View):
    box: Container
    fruits: List[Apple]

    def __post_init__(self):
        if self.name is None:
            self.name = PrefixedName(str(self.box.name), self.__class__.__name__)


apple_body_2 = Body(name=PrefixedName("apple_body_2"))
sphere = Sphere(radius=0.15, origin=TransformationMatrix(reference_frame=apple_body_2))
apple_body_2.collision = [sphere]
apple_body_2.visual = [sphere]


with world.modify_world():
    bowl_world = ContainerFactory(
        name=PrefixedName("box"), direction=Direction.Z, scale=Scale(1.0, 1.0, 0.3)
    ).create()
    world.merge_world_at_pose(
        bowl_world,
        TransformationMatrix(),
    )

    root_to_apple2 = Connection6DoF(
        parent=root,
        child=apple_body_2,
        _world=world,
    )
    world.add_connection(root_to_apple2)
    world.state[root_to_apple2.x.name].position = 0.3

world.add_view(Apple(body=apple_body_2, name=PrefixedName("apple2")))
fruit_box = FruitBox(
    box=world.get_views_by_type(Container)[0], fruits=world.get_views_by_type(Apple)
)
world.add_view(fruit_box)
print(f"Fruit box with {len(fruit_box.fruits)} fruits")
```

One great quality of this is that every other agent who imports your definitions of views in the world is now able
to understand what you mean and how you define it. Since everything is python, the processes won't have any compatibility
issues.
