# API Reference

This reference provides comprehensive documentation for all classes and functions in the Semantic World library.

## Core Classes

### World

The main container class that manages all bodies, connections, and views in a semantic world.

```python
from semantic_world.world import World

world = World()
```

**Key Methods:**
- `add_body(body: Body)` - Add a body to the world
- `add_connection(connection: Connection)` - Add a connection between bodies  
- `add_view(view: View)` - Add a semantic view
- `validate()` - Validate world structure
- `compute_forward_kinematics(root: Body, tip: Body)` - Compute transform between bodies
- `set_positions_1DOF_connection(positions: Dict[Connection, float])` - Set joint positions
- `merge_with_pose(other_world: World, pose: np.ndarray)` - Merge another world

**Properties:**
- `bodies: List[Body]` - All bodies in the world
- `connections: List[Connection]` - All connections in the world
- `views: List[View]` - All semantic views
- `degrees_of_freedom: List[DegreeOfFreedom]` - All DOFs
- `state: WorldState` - Current state of all DOFs

### Body

Represents a physical entity that cannot be decomposed into smaller meaningful parts.

```python
from semantic_world.world_entity import Body
from semantic_world.geometry import Box

body = Body(
    name="my_body",
    visual=[Box(size=[1.0, 1.0, 1.0])],
    collision=[Box(size=[1.0, 1.0, 1.0])]
)
```

**Properties:**
- `name: PrefixedName` - Unique identifier
- `visual: List[Shape]` - Visual geometry shapes
- `collision: List[Shape]` - Collision geometry shapes
- `index: Optional[int]` - Index in world kinematic structure
- `global_pose: np.ndarray` - 4x4 global transformation matrix

### Connection

Base class for all connections between bodies. Defines parent->child relationships.

```python
from semantic_world.world_entity import Connection

# Use specific connection types like RevoluteConnection, not the base class directly
```

**Properties:**
- `parent: Body` - Parent body
- `child: Body` - Child body  
- `origin_expression: TransformationMatrix` - Symbolic transform expression
- `origin: TransformationMatrix` - Computed transform from parent to child

## Connection Types

### FixedConnection

Rigid connection with no degrees of freedom.

```python
from semantic_world.connections import FixedConnection

connection = FixedConnection(parent=body1, child=body2)
```

### RevoluteConnection

Single-axis rotational joint.

```python
from semantic_world.connections import RevoluteConnection  
from semantic_world.degree_of_freedom import DegreeOfFreedom

joint = RevoluteConnection(
    parent=body1,
    child=body2,
    dof=DegreeOfFreedom(name="joint1", lower_limit=-3.14, upper_limit=3.14),
    axis=[0, 0, 1]  # Rotation axis
)

# Control the joint
joint.position = 1.57  # 90 degrees
```

**Properties:**
- `dof: DegreeOfFreedom` - Controls rotation amount
- `axis: Vector3` - Rotation axis in parent frame
- `position: float` - Current joint angle (radians)
- `velocity: float` - Current joint velocity
- `acceleration: float` - Current joint acceleration

### PrismaticConnection

Single-axis translational joint.

```python
from semantic_world.connections import PrismaticConnection

slider = PrismaticConnection(
    parent=body1,
    child=body2, 
    dof=DegreeOfFreedom(name="slider1", lower_limit=0.0, upper_limit=1.0),
    axis=[1, 0, 0]  # Translation axis
)

# Control the slider
slider.position = 0.5  # 50cm extension
```

**Properties:**
- `dof: DegreeOfFreedom` - Controls translation amount
- `axis: Vector3` - Translation axis in parent frame
- `position: float` - Current position along axis
- `velocity: float` - Current velocity
- `acceleration: float` - Current acceleration

### Connection6DoF

Six degree-of-freedom connection for full spatial positioning.

```python
from semantic_world.connections import Connection6DoF

floating_joint = Connection6DoF(parent=world.root, child=floating_body)

# Set 6-DOF pose
pose = np.eye(4)
pose[:3, 3] = [1, 2, 3]  # Position
floating_joint.origin = pose
```

## Degree of Freedom

Represents a controllable parameter with limits.

```python
from semantic_world.degree_of_freedom import DegreeOfFreedom

dof = DegreeOfFreedom(
    name="joint1",
    lower_limit=-3.14159,
    upper_limit=3.14159,
    velocity_limit=2.0,
    acceleration_limit=5.0
)
```

**Properties:**
- `name: PrefixedName` - Unique identifier
- `lower_limit: float` - Minimum position value
- `upper_limit: float` - Maximum position value  
- `velocity_limit: float` - Maximum velocity magnitude
- `acceleration_limit: float` - Maximum acceleration magnitude
- `position: float` - Current position
- `velocity: float` - Current velocity
- `acceleration: float` - Current acceleration

## Geometry Classes

### Box

Rectangular box geometry.

```python
from semantic_world.geometry import Box

box = Box(size=[length, width, height])
```

### Sphere

Spherical geometry.

```python
from semantic_world.geometry import Sphere

sphere = Sphere(radius=0.5)
```

### Cylinder

Cylindrical geometry.

```python
from semantic_world.geometry import Cylinder

cylinder = Cylinder(radius=0.2, height=1.0)
```

### Mesh

Mesh geometry loaded from file.

```python
from semantic_world.geometry import Mesh

mesh = Mesh(filename="path/to/mesh.stl", scale=[1.0, 1.0, 1.0])
```

## Views

### View

Base class for semantic interpretations of world structures.

```python  
from semantic_world.world_entity import View

# Use specific view types like Table, Drawer, etc.
```

### Table

Semantic view representing a table structure.

```python
from semantic_world.views.views import Table

table = Table(bodies=[table_top, leg1, leg2, leg3, leg4], name="dining_table")
world.add_view(table)
```

### Drawer

Semantic view representing a drawer with handle and sliding motion.

```python
from semantic_world.views.views import Drawer

drawer = Drawer(
    bodies=[drawer_frame, drawer_box, handle],
    name="kitchen_drawer",
    handle=handle,
    container=drawer_box
)
world.add_view(drawer)
```

## Spatial Types

### TransformationMatrix

4x4 homogeneous transformation matrix for spatial relationships.

```python
from semantic_world.spatial_types.spatial_types import TransformationMatrix
import numpy as np

# Create from numpy array
transform = TransformationMatrix(np.eye(4))

# Create with translation
transform = TransformationMatrix.from_xyz_rpy(x=1.0, y=2.0, z=3.0)

# Access as numpy array
matrix = transform.matrix
```

### Vector3

3D vector representation.

```python
from semantic_world.spatial_types.spatial_types import Vector3

vec = Vector3.from_iterable([1.0, 2.0, 3.0])
```

## Utilities

### PrefixedName

Hierarchical naming system with optional prefixes.

```python
from semantic_world.prefixed_name import PrefixedName

# Simple name
name1 = PrefixedName("link1")

# Name with prefix (creates hierarchy)
name2 = PrefixedName("arm_link", prefix="robot1")  # "robot1/arm_link"
```

### WorldState  

Manages the state (position, velocity, acceleration) of all degrees of freedom.

```python
from semantic_world.world_state import WorldState

state = WorldState()

# Access DOF states
state[dof.name].position = 1.0
state[dof.name].velocity = 0.5
```

## Error Handling

### Common Exceptions

```python
from semantic_world.exceptions import (
    DuplicateViewError,
    AddingAnExistingViewError,
    ViewNotFoundError
)

try:
    world.add_view(duplicate_view)
except DuplicateViewError:
    print("View already exists")

try:
    world.validate()
except ValidationError as e:
    print(f"World validation failed: {e}")
```

## Complete Example

Here's a complete example using the main API components:

```python
from semantic_world.world import World
from semantic_world.world_entity import Body
from semantic_world.connections import RevoluteConnection, FixedConnection
from semantic_world.degree_of_freedom import DegreeOfFreedom
from semantic_world.geometry import Box, Cylinder
from semantic_world.views.views import Table
import numpy as np

# Create world
world = World()

# Create bodies with geometry
base = Body(
    name="robot_base",
    visual=[Cylinder(radius=0.2, height=0.1)],
    collision=[Cylinder(radius=0.2, height=0.1)]
)

arm = Body(
    name="robot_arm", 
    visual=[Box(size=[0.05, 0.05, 0.3])],
    collision=[Box(size=[0.05, 0.05, 0.3])]
)

tool = Body(name="tool", visual=[Box(size=[0.1, 0.02, 0.02])])

# Add bodies to world
world.add_body(base)
world.add_body(arm)
world.add_body(tool)

# Create degrees of freedom  
shoulder_dof = DegreeOfFreedom(
    name="shoulder_joint",
    lower_limit=-np.pi,
    upper_limit=np.pi,
    velocity_limit=2.0
)

# Create connections
shoulder_joint = RevoluteConnection(
    parent=base,
    child=arm,
    dof=shoulder_dof,
    axis=[0, 0, 1]
)

tool_mount = FixedConnection(parent=arm, child=tool)

# Add connections to world
world.add_connection(shoulder_joint)
world.add_connection(tool_mount)

# Create semantic view
robot_view = Table(bodies=[base], name="robot_platform") 
world.add_view(robot_view)

# Validate world
world.validate()

# Control robot
shoulder_joint.position = np.pi/4  # 45 degrees

# Compute forward kinematics
tool_pose = world.compute_forward_kinematics(base, tool)
print(f"Tool position: {tool_pose[:3, 3]}")

# Access world state
print(f"Shoulder angle: {world.state[shoulder_dof.name].position} rad")
print(f"World has {len(world.bodies)} bodies, {len(world.connections)} connections")

# Save to database (optional)
from sqlalchemy import create_engine
from sqlalchemy.orm import Session
from semantic_world.orm.ormatic_interface import Base
from ormatic.dao import to_dao

engine = create_engine('sqlite:///robot.db')
session = Session(engine)  
Base.metadata.create_all(bind=session.bind)

world_dao = to_dao(world)
session.add(world_dao)
session.commit()

print("Robot saved to database!")
```

## Best Practices

1. **Always validate** worlds after construction: `world.validate()`
2. **Use descriptive names** with prefixes for organization
3. **Batch state updates** for performance: `world.set_positions_1DOF_connection({...})`
4. **Check DOF limits** before setting positions
5. **Use views** to add semantic meaning to kinematic structures
6. **Handle exceptions** properly during world construction and modification
7. **Reset caches** after dynamic modifications: `world.reset_cache()`
8. **Use context managers** for temporary state changes: `with world.reset_state_context():`

## Performance Considerations

- Forward kinematics are cached - avoid unnecessary cache clearing
- Batch multiple joint updates together
- Use appropriate geometry complexity for collision vs visual shapes
- Consider world copying costs for simulation scenarios
- Database serialization is expensive - use judiciously

For more details and advanced usage, see the comprehensive user guides:
- [Getting Started Guide](getting_started.md)
- [World Construction Guide](world_construction.md) 
- [Body Management Guide](body_management.md)
- [Connection Types Guide](connection_types.md)
- [Practical Examples](examples.md)