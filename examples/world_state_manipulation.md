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

(world-state-manipulation)=
# World State Manipulation

In this tutorial we will manipulate the state (free variables) of the world.

Concepts Used:
- [](visualizing-worlds)
- Factories (TODO)
- [Entity Query Language](https://abdelrhmanbassiouny.github.io/entity_query_language/intro.html)
- [](world-structure-manipulation)

First, we create a dresser containing a single drawer using the respective factories.

```python
import threading
import time

import numpy as np
import rclpy
from entity_query_language import the, entity, let

from semantic_world.adapters.viz_marker import VizMarkerPublisher
from semantic_world.datastructures.prefixed_name import PrefixedName
from semantic_world.spatial_types.spatial_types import TransformationMatrix
from semantic_world.views.factories import (
    DresserFactory,
    ContainerFactory,
    HandleFactory,
    DrawerFactory,
    Direction,
)
from semantic_world.views.views import Drawer
from semantic_world.world_description.degree_of_freedom import DegreeOfFreedom
from semantic_world.world_description.geometry import Scale

rclpy.init()

drawer_factory = DrawerFactory(
    name=PrefixedName("drawer"),
    container_factory=ContainerFactory(
        name=PrefixedName("drawer_container"),
        direction=Direction.Z,
        scale=Scale(0.3, 0.3, 0.2),
    ),
    handle_factory=HandleFactory(name=PrefixedName("drawer_handle")),
)
drawer_transform = TransformationMatrix()

container_factory = ContainerFactory(
    name=PrefixedName("dresser_container"), scale=Scale(0.31, 0.31, 0.21)
)

dresser_factory = DresserFactory(
    name=PrefixedName("dresser"),
    drawer_transforms=[drawer_transform],
    drawers_factories=[drawer_factory],
    container_factory=container_factory,
)

world = dresser_factory.create()
```

Next, we creat the visualizer.

```python
node = rclpy.create_node("semantic_world")
thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
thread.start()
viz = VizMarkerPublisher(world=world, node=node)

```

Let's get a reference to the drawer we built above.

```python
drawer = the(
    entity(
        let("drawer", type_=Drawer, domain=world.views),
        True,
    )
).evaluate()
```

We can update the drawer's state by altering the free variables position of its prismatic connection to the dresser.

```python
time.sleep(1)
drawer.container.body.parent_connection.position = 0.1
time.sleep(1)
```

Note that this only works in this simple way for connections that only have one degree of freedom. For multiple degrees of freedom you either have to set the entire transformation or use the world state directly.
To show this we first create a new root for the world and make a free connection from the new root to the dresser.

```python
from semantic_world.world_description.connections import Connection6DoF
from semantic_world.world_description.world_entity import Body

with world.modify_world():
    old_root = world.root
    new_root = Body(name=PrefixedName("virtual root"))
    world.add_body(new_root)
    root_T_dresser = Connection6DoF(new_root, old_root, _world=world)
    world.add_connection(root_T_dresser)
```

Now we can start moving the dresser everywhere and even rotate it.

```python
from semantic_world.world_description.world_entity import Connection

free_connection = the(entity(connection := let("connection", type_=Connection, domain=world.connections), connection.parent == world.root)).evaluate()
time.sleep(1)
with world.modify_world():
    free_connection.origin_expression = TransformationMatrix.from_xyz_rpy(1., 1., 0, 0., 0., 0.5 * np.pi)
time.sleep(1)

```

The final way of manipulating the world state is the registry for all degrees of freedom, the {py:class}`semantic_world.world_description.world_state.WorldState`.
This class acts as a dict like structure that maps degree of freedoms to their state.
The state is an array of 4 values: the position, velocity, acceleration and jerk.
Since it is an aggregation of all degree of freedoms existing in the world, it can be messy to access.
We can close the drawer again as follows:

```python
dof = the(entity(name := let("degree_of_freedom", type_=PrefixedName, domain=world.state.keys()), name.name == "drawer_container_connection")).evaluate()
with world.modify_world():
    world.state[dof] = [0., 0, 0, 0.]
time.sleep(1.)
```

```python
node.destroy_node()
rclpy.shutdown()
```
