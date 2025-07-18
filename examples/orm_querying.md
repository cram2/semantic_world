---
jupytext:
  text_representation:
    extension: .md
    format_name: myst
    format_version: 0.13
    jupytext_version: 1.17.2
kernelspec:
  display_name: Python 3
  language: python
  name: python3
---

# ORM Example

The semantic world comes with an ORM attached to it that is derived from the python datastructures.
The ORM can be used to serialize entire worlds into an SQL database and retrieve them later. The semantic annotations (views) are stored alongside the kinematic information.
The queried worlds are full objects that can be reconstructed into the original objects without any problems.
Let's go into an example where we create a world, store it, retrieve and reconstruct it.
First, lets load a world from a URDF file.

```{code-cell} ipython2
import os

import rclpy
from sqlalchemy import create_engine, select
from sqlalchemy.orm import Session

from ormatic.dao import to_dao
from semantic_world.adapters.urdf import URDFParser
from semantic_world.orm.ormatic_interface import *
from semantic_world.views import Table

# setup ros 2
rclpy.init()

# set up an in memory database
engine = create_engine('sqlite:///:memory:')
session = Session(engine)
Base.metadata.create_all(bind=session.bind)

# load the table world from urdf
urdf_dir = os.path.join(os.path.expanduser("~"), "semantic_world", "resources", "urdf")
table = os.path.join(urdf_dir, "table.urdf")
world = URDFParser(table).parse()
```

Next, we create a semantic annotation that describes the table.

```{code-cell} ipython2
table_view = Table([b for b in world.bodies if "top" in str(b.name)][0])
world.add_view(table_view)
print(table_view)
```

Now, let's store the world to a database. For that, we need to convert it to its data access object which than can be stored in the database.

```{code-cell} ipython2
dao = to_dao(world)
session.add(dao)
session.commit()
```

We can now query the database about the world and reconstruct it to the original instance. As you can see the semantic annotations are also available and fully working.

```{code-cell} ipython2
queried_world = session.scalars(select(WorldMappingDAO)).one()
reconstructed_world = queried_world.from_dao()
table = [view for view in reconstructed_world.views if isinstance(view, Table)][0]
print(table)
print(table.points_on_table(2))
rclpy.shutdown()
```

## Maintaining the ORM 🧰

You can maintain the ORM by maintaining the [generate_orm.py](https://github.com/cram2/semantic_world/blob/main/scripts/generate_orm.py).
In there you have to list all the classes you want to generate mappings for and perhaps some type decorators for advanced use cases.
Whenever you write a new dataclass that should appear or has semantic meaningful content make sure it appears in the set of classes.
Pay attention to the logger during generation and see if it understands your datastructures correctly.

+++

## The sharp bits 🔪
The world class manages the dependencies of the bodies in the world. Whenever you retrieve a body or connection, it comes as a data access object that is disconnected from the world itself.
The relationships to the world exist and can be joined. However, when you reconstruct something else but the world, the reconstructed object does not have a world available. You can always reconstruct the entire world by querying for the objects world instead.
