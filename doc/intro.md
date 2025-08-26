# Semantic World


# Semantic World


Welcome to the semantic world package!
The semantic world is a python package that unifies the access and manipulation of scene graphs.
It features a:

- Unified API to scene graphs
- Forward simulation for worlds and agents
- Semantic interpretation of structures in the world
- Automatic serialization of worlds to databases
- Connecting worlds to different simulator backends through Multiverse {cite:p}`Multiverse`

This package originates from different developments of the [AICOR Institute for Artificial Intelligence](https://ai.uni-bremen.de/). 
Four different projects developed a very similar component for different parts of cognitive modules.
This project aims to unify it under one solution that is flexible enough for all the different applications.

## Quick Start

New to semantic world? Start with our [Getting Started Guide](getting_started.md) for a hands-on tutorial covering:
- Creating your first world
- Working with bodies and connections  
- Building a simple robot arm
- Computing forward kinematics
- Using semantic views

For more advanced topics, see our comprehensive user guides:
- [World Construction Guide](world_construction.md) - Building complex kinematic structures
- [Body Management Guide](body_management.md) - Working with bodies and their properties
- [Connection Types Guide](connection_types.md) - All connection types and their uses
- [Practical Examples](examples.md) - Real-world robotics applications

## Core Concepts

## Core Concepts

## World

The central datastructure for interaction with a scene is the {py:class}`semantic_world.world_entity.World`.
The world is a mediator for bodies and their connections.
It handles the validation of the world's kinematic structure and the communication between the objects.

Physical Objects can be spawned by constructing a {py:class}`semantic_world.world_entity.Body` and a kinematic chain of 
those elements is added by specifying a {py:class}`semantic_world.world_entity.Connection` between bodies.

All those things have to be added to the world for full functionality.
More information on the kinematic world model can be found [here](kinematic_world.md).

### Quick Example

```python
from semantic_world.world import World
from semantic_world.world_entity import Body  
from semantic_world.connections import RevoluteConnection
from semantic_world.degree_of_freedom import DegreeOfFreedom
import numpy as np

# Create world and bodies
world = World()
base = Body(name="base")
arm = Body(name="arm") 

world.add_body(base)
world.add_body(arm)

# Create revolute joint
joint_dof = DegreeOfFreedom(name="shoulder", lower_limit=-np.pi, upper_limit=np.pi)
joint = RevoluteConnection(parent=base, child=arm, dof=joint_dof, axis=[0, 0, 1])
world.add_connection(joint)

# Control the joint
joint.position = np.pi/4  # 45 degrees
transform = world.compute_forward_kinematics(base, arm)
print(f"Arm position: {transform[:3, 3]}")
```


## WorldReasoner

The world reasoner {py:class}`semantic_world.reasoner.WorldReasoner` is a class that uses [Ripple Down Rules](https://github.com/AbdelrhmanBassiouny/ripple_down_rules/tree/main)
to classify concepts and attributes of the world. This is done using a rule based classifier that benefits from incremental
rule addition through querying the system and answering the prompts that pop up using python code.

The benefit of that is the rules of the reasoner are based on the world datastructures and are updates as the datastructures
are updated. Thus, the rules become a part of the semantic world repository and are update, migrated, and versioned with it.

More information about the world reasoner can be found [here](world_reasoner.md).

## Views

A View ({py:class}`semantic_world.world_entity.View`) is a different representation for a part or a collection of parts in the world that has a semantic meaning and
functional purpose in specific contexts.

For example, a Drawer can be seen as a view on a handle and a container that is connected via a fixed connection
and where the container has some prismatic connection.

Views can be inferred by specifying rules that make up a view. More information on how the views are inferred and used
can be found [here](views.md).

### Views Example

```python
from semantic_world.views.views import Table

# Create table bodies
table_top = Body(name="table_top")
table_legs = [Body(name=f"leg_{i}") for i in range(4)]

# Add to world and connect...
# (connections omitted for brevity)

# Create semantic view
table_view = Table(bodies=[table_top] + table_legs, name="dining_table")
world.add_view(table_view)

# Query views
for view in world.views:
    if isinstance(view, Table):
        print(f"Found table: {view.name} with {len(view.bodies)} bodies")
```

## Database

The entire world can be saved to any database
that has an [sqlalchemy](https://docs.sqlalchemy.org/en/20/index.html) connector.
The definitions and relationships for the database are automatically derived from the datastructures
derived in the python package via the [ormatic](https://github.com/tomsch420/ormatic) package.
The types for sqlalchemy are defined in {py:mod}`semantic_world.orm.model`.
The interface to sqlalchemy is auto-generated to {py:mod}`semantic_world.orm.ormatic_interface`.
The script to recreate the interface is found in [here](https://github.com/cram2/semantic_world/blob/main/scripts/generate_orm.py).

Learn more about the ORM in [this tutorial](orm-guide).

### Database Example

```python
from sqlalchemy import create_engine
from sqlalchemy.orm import Session
from semantic_world.orm.ormatic_interface import Base
from ormatic.dao import to_dao, from_dao

# Save world to database
engine = create_engine('sqlite:///my_robots.db')
session = Session(engine)
Base.metadata.create_all(bind=session.bind)

# Convert world to database object
world_dao = to_dao(world)
session.add(world_dao)
session.commit()

# Load world from database
loaded_world_dao = session.query(type(world_dao)).first()
loaded_world = from_dao(loaded_world_dao)
```

## Next Steps

Ready to start building? Check out these resources:

1. **[Getting Started Guide](getting_started.md)** - Your first semantic world in 10 minutes
2. **[Practical Examples](examples.md)** - Real robot manipulators, mobile robots, and multi-robot systems  
3. **[World Construction Guide](world_construction.md)** - Advanced kinematic structures and world merging
4. **[API Documentation](autoapi/index.html)** - Complete reference for all classes and functions

For specific topics:
- **Bodies**: [Body Management Guide](body_management.md) covers geometry, properties, and relationships
- **Connections**: [Connection Types Guide](connection_types.md) details all joint types and their usage
- **Kinematics**: [Kinematic World Model](kinematic_world.md) explains the underlying mathematical framework
- **Semantics**: [Views Guide](views.md) shows how to add semantic meaning to kinematic structures
- **Reasoning**: [World Reasoner](world_reasoner.md) covers rule-based semantic classification
- **Persistence**: [ORM Guide](orm-guide) explains database integration and serialization


