# Getting Started with Semantic World

Welcome to the Semantic World library! This guide will help you get started with creating, manipulating, and working with semantic worlds, bodies, and connections.

## Installation

You can install the semantic world package directly from PyPI:

```bash
pip install semantic_world
```

Or for development, clone the repository and install in editable mode:

```bash
git clone https://github.com/cram2/semantic_world.git
cd semantic_world
pip install -e .
```

## Basic Concepts

The Semantic World library is built around three main concepts:

- **World**: The main container that manages all bodies and their connections
- **Body**: Physical entities or objects in the world (cannot be decomposed into smaller meaningful parts)  
- **Connection**: Relationships between bodies that define how they are connected kinematically

## Your First World

Let's start by creating a simple world with two connected bodies:

```python
from semantic_world.world import World
from semantic_world.world_entity import Body
from semantic_world.connections import FixedConnection

# Create a new world
world = World()

# Create two bodies
body1 = Body(name="base")
body2 = Body(name="arm")

# Add bodies to the world
world.add_body(body1)
world.add_body(body2)

# Create a fixed connection between them
connection = FixedConnection(parent=body1, child=body2)

# Add the connection to the world  
world.add_connection(connection)

# Validate the world structure
world.validate()

print(f"World contains {len(world.bodies)} bodies and {len(world.connections)} connections")
```

## Working with Bodies

Bodies represent physical objects in your world. They can have visual and collision geometry:

```python
from semantic_world.geometry import Box, Sphere, Scale
from semantic_world.world_entity import Body
from semantic_world.spatial_types.spatial_types import TransformationMatrix

# Create a body with visual geometry
visual_box = Box(size=[1.0, 1.0, 1.0])
collision_sphere = Sphere(radius=0.5)

body = Body(
    name="my_body",
    visual=[visual_box],
    collision=[collision_sphere]
)

# Bodies can be positioned relative to their parent
# The geometry origins are relative to the body frame
```

## Working with Connections

Connections define how bodies relate to each other kinematically. There are several types:

### Fixed Connections

Fixed connections create rigid relationships between bodies:

```python
from semantic_world.connections import FixedConnection
import numpy as np

# Create a fixed connection with a specific transform
transform = np.eye(4)
transform[0, 3] = 1.0  # 1 meter offset in x-direction

fixed_conn = FixedConnection(
    parent=body1, 
    child=body2,
    origin_expression=TransformationMatrix(transform)
)
```

### Active Connections (Joints)

Active connections have controllable degrees of freedom:

```python
from semantic_world.connections import RevoluteConnection, PrismaticConnection
from semantic_world.degree_of_freedom import DegreeOfFreedom

# Create a revolute joint (rotation)
dof_rotation = DegreeOfFreedom(
    name="shoulder_joint",
    lower_limit=-np.pi,
    upper_limit=np.pi
)

revolute_joint = RevoluteConnection(
    parent=body1,
    child=body2, 
    dof=dof_rotation,
    axis=[0, 0, 1]  # Rotate around z-axis
)

# Create a prismatic joint (linear motion)
dof_linear = DegreeOfFreedom(
    name="slider_joint",
    lower_limit=0.0,
    upper_limit=2.0
)

prismatic_joint = PrismaticConnection(
    parent=body1,
    child=body2,
    dof=dof_linear,
    axis=[1, 0, 0]  # Slide along x-axis
)
```

## Building a Simple Robot Arm

Let's put it all together and build a simple 2-link robot arm:

```python
from semantic_world.world import World
from semantic_world.world_entity import Body
from semantic_world.connections import FixedConnection, RevoluteConnection
from semantic_world.degree_of_freedom import DegreeOfFreedom
from semantic_world.geometry import Box, Scale
import numpy as np

# Create the world
world = World()

# Create bodies for the robot
base = Body(name="base", visual=[Box(scale=Scale(x=0.2, y=0.2, z=0.1))])
link1 = Body(name="link1", visual=[Box(scale=Scale(x=0.05, y=0.05, z=0.3))])  
link2 = Body(name="link2", visual=[Box(scale=Scale(x=0.05, y=0.05, z=0.25))])
end_effector = Body(name="end_effector", visual=[Box(scale=Scale(x=0.1, y=0.05, z=0.02))])

# Add bodies to world
for body in [base, link1, link2, end_effector]:
    world.add_body(body)

# Create joints with degrees of freedom
joint1_dof = DegreeOfFreedom(name="joint1", lower_limit=-np.pi, upper_limit=np.pi)
joint2_dof = DegreeOfFreedom(name="joint2", lower_limit=-np.pi/2, upper_limit=np.pi/2)

# Create connections
base_to_link1 = RevoluteConnection(
    parent=base, 
    child=link1, 
    dof=joint1_dof,
    axis=[0, 0, 1]
)

link1_to_link2 = RevoluteConnection(
    parent=link1, 
    child=link2, 
    dof=joint2_dof,
    axis=[0, 0, 1]
)

# Fixed connection for end effector
link2_to_ee = FixedConnection(parent=link2, child=end_effector)

# Add connections to world
world.add_connection(base_to_link1)
world.add_connection(link1_to_link2) 
world.add_connection(link2_to_ee)

# Validate the complete structure
world.validate()

print(f"Robot arm created with {len(world.bodies)} bodies and {len(world.connections)} connections")
print(f"Degrees of freedom: {len(world.degrees_of_freedom)}")

# Control the joints
world.set_positions_1DOF_connection({
    base_to_link1: np.pi/4,  # 45 degrees
    link1_to_link2: -np.pi/6  # -30 degrees  
})
```

## Computing Forward Kinematics

Once you have a kinematic structure, you can compute transformations between bodies:

```python
# Compute the transform from base to end effector
transform = world.compute_forward_kinematics(base, end_effector)
print(f"End effector position: {transform[:3, 3]}")

# Get the global pose of any body
ee_global_pose = end_effector.global_pose
print(f"End effector global pose:\n{ee_global_pose}")
```

## Working with Views

Views provide semantic interpretations of world structures:

```python
from semantic_world.views.views import Drawer, Table

# Views can be manually added or automatically inferred
table_view = Table(bodies=[base], name="work_table")
world.add_view(table_view)

# Check what views exist in the world
for view in world.views:
    print(f"View: {view.name}, Type: {type(view).__name__}")
```

## Saving and Loading Worlds

The semantic world can be serialized to a database:

```python
from sqlalchemy import create_engine
from sqlalchemy.orm import Session
from semantic_world.orm.ormatic_interface import Base
from ormatic.dao import to_dao, from_dao

# Create database connection
engine = create_engine('sqlite:///my_world.db')
session = Session(engine)
Base.metadata.create_all(bind=session.bind)

# Save world to database
world_dao = to_dao(world)
session.add(world_dao)
session.commit()

# Load world from database
loaded_world_dao = session.query(type(world_dao)).first()
loaded_world = from_dao(loaded_world_dao)
```

## Next Steps

Now that you understand the basics, explore these topics:

- [World Construction Guide](world_construction.md) - Detailed guide on building complex worlds
- [Body Management Guide](body_management.md) - Working with bodies and their properties  
- [Connection Types Guide](connection_types.md) - All connection types and their uses
- [Kinematic World Model](kinematic_world.md) - Understanding the underlying kinematic model
- [Views and Semantic Interpretation](views.md) - Working with semantic views
- [ORM and Database Integration](orm-guide) - Persisting worlds to databases

## Common Patterns

### Chain of Bodies
```python
# Create a kinematic chain
bodies = [Body(name=f"link_{i}") for i in range(5)]
for i, body in enumerate(bodies):
    world.add_body(body)
    if i > 0:
        connection = FixedConnection(parent=bodies[i-1], child=body)
        world.add_connection(connection)
```

### Parallel Structures
```python  
# Create parallel kinematic structures
base = Body(name="base")
left_arm = Body(name="left_arm") 
right_arm = Body(name="right_arm")

world.add_body(base)
world.add_body(left_arm)
world.add_body(right_arm)

# Both arms connect to the same base
world.add_connection(FixedConnection(parent=base, child=left_arm))
world.add_connection(FixedConnection(parent=base, child=right_arm))
```

### Dynamic Modification
```python
# Worlds can be modified at runtime
new_body = Body(name="attachment")
world.add_body(new_body)

# Connect to existing structure
connection = FixedConnection(parent=end_effector, child=new_body)
world.add_connection(connection)

# Remove elements when needed
world.remove_connection(connection)
world.remove_body(new_body)
```

This covers the fundamentals of working with semantic worlds. Each concept has more advanced features covered in the dedicated guides.