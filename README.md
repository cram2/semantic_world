# Welcome to the Semantic World Package

The semantic world is a Python package for querying and manipulating robot simulation data.  
It originates from PyCRAM's abstract world and unifies the functionality needed by multiple packages.

## Features

- **Unified API** for scene graphs and kinematic structures
- **Forward simulation** for worlds and robotic agents  
- **Semantic interpretation** of structures through views
- **Automatic serialization** to databases via ORM
- **Simulator integration** through Multiverse backends
- **Symbolic kinematics** with CasADi for differentiable computations

## Quick Start

```python
from semantic_world.world import World
from semantic_world.world_entity import Body
from semantic_world.connections import RevoluteConnection
from semantic_world.degree_of_freedom import DegreeOfFreedom
import numpy as np

# Create a simple robot
world = World()
base = Body(name="base")
arm = Body(name="arm")

world.add_body(base)
world.add_body(arm)

# Add revolute joint
joint = RevoluteConnection(
    parent=base, 
    child=arm,
    dof=DegreeOfFreedom(name="shoulder", lower_limit=-np.pi, upper_limit=np.pi),
    axis=[0, 0, 1]
)
world.add_connection(joint)

# Control and query
joint.position = np.pi/4  # 45 degrees
transform = world.compute_forward_kinematics(base, arm)
print(f"Arm tip position: {transform[:3, 3]}")
```

# User Installation


You can install the package directly from PyPI:

```bash
pip install -U semantic_world
```

# Contributing

If you are interested in contributing, you can check out the source code from GitHub:

```bash
git clone https://github.com/cram2/semantic_world.git
```

# Tests
The tests can be run using `pytest` after installing the package.

```bash
pytest test/
```

# Documentation

You can read the official documentation [here](https://cram2.github.io/semantic_world/intro.html)!

## Quick Links

- 🚀 **[Getting Started](https://cram2.github.io/semantic_world/getting_started.html)** - Your first semantic world in 10 minutes
- 📚 **[User Guides](https://cram2.github.io/semantic_world/world_construction.html)** - Comprehensive guides for world construction, bodies, and connections  
- 🤖 **[Examples](https://cram2.github.io/semantic_world/examples.html)** - Real robot manipulators, mobile robots, and multi-robot scenarios
- 📖 **[API Reference](https://cram2.github.io/semantic_world/api_reference.html)** - Complete class and function documentation
- 🔧 **[Troubleshooting](https://cram2.github.io/semantic_world/troubleshooting.html)** - Common issues and solutions

### Core Concepts

- **[World Construction Guide](https://cram2.github.io/semantic_world/world_construction.html)** - Building complex kinematic structures, world merging, and state management
- **[Body Management Guide](https://cram2.github.io/semantic_world/body_management.html)** - Working with bodies, geometry, properties, and relationships  
- **[Connection Types Guide](https://cram2.github.io/semantic_world/connection_types.html)** - All connection types: Fixed, Revolute, Prismatic, 6DOF, and advanced features
- **[Kinematic World Model](https://cram2.github.io/semantic_world/kinematic_world.html)** - Understanding the underlying symbolic kinematics
- **[Views and Semantics](https://cram2.github.io/semantic_world/views.html)** - Adding semantic meaning to kinematic structures
- **[World Reasoner](https://cram2.github.io/semantic_world/world_reasoner.html)** - Rule-based semantic classification
- **[ORM Guide](https://cram2.github.io/semantic_world/examples/orm_querying.html)** - Database integration and world persistence