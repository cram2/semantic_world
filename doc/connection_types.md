# Connection Types Guide

Connections define the kinematic relationships between bodies in semantic worlds. This comprehensive guide covers all connection types, their properties, and usage patterns.

## Connection Fundamentals

All connections inherit from the base `Connection` class and define how two bodies relate to each other:

```python
from semantic_world.world_entity import Connection, Body
from semantic_world.spatial_types.spatial_types import TransformationMatrix
import numpy as np

# Basic connection concept
parent_body = Body(name="parent")
child_body = Body(name="child")

# Every connection has parent -> child relationship
# and defines the transform from parent to child frame
```

## Connection Hierarchy

The semantic world supports several connection types:

```
Connection (base class)
├── FixedConnection (rigid, 0 DOF)
├── ActiveConnection (controllable DOF) 
│   ├── RevoluteConnection (1 rotational DOF)
│   ├── PrismaticConnection (1 translational DOF)
│   └── Connection6DoF (up to 6 DOF)
└── PassiveConnection (uncontrolled DOF)
    └── Custom implementations possible
```

## Fixed Connections

Fixed connections create rigid relationships with no degrees of freedom:

```python
from semantic_world.connections import FixedConnection
from semantic_world.world import World
from semantic_world.spatial_types.math import rotation_matrix_from_rpy

world = World()
base = Body(name="base")
attachment = Body(name="attachment")

world.add_body(base)
world.add_body(attachment)

# Create fixed connection with default identity transform
fixed_conn = FixedConnection(parent=base, child=attachment)
world.add_connection(fixed_conn)

# Fixed connection with specific transform
transform = np.eye(4)
transform[:3, 3] = [0.1, 0.2, 0.3]  # Translation
transform[:3, :3] = rotation_matrix_from_rpy(0, 0, np.pi/4)  # 45° rotation

fixed_conn_with_pose = FixedConnection(
    parent=base, 
    child=attachment,
    origin_expression=TransformationMatrix(transform)
)

print(f"Fixed connection has {len(fixed_conn.degrees_of_freedom)} DOFs")
```

### Use Cases for Fixed Connections
- Attaching sensors to robot bodies
- Connecting rigid parts of a structure
- Mounting tools or end effectors
- Creating composite bodies from parts

```python
def attach_camera_to_robot(world, robot_head, camera_offset=[0.05, 0, 0.02]):
    """Attach a camera to a robot head with fixed offset"""
    camera = Body(name="camera")
    world.add_body(camera)
    
    # Create transform for camera mounting
    camera_transform = np.eye(4)
    camera_transform[:3, 3] = camera_offset
    
    camera_connection = FixedConnection(
        parent=robot_head,
        child=camera,
        origin_expression=TransformationMatrix(camera_transform)
    )
    world.add_connection(camera_connection)
    
    return camera

# Example usage
robot_head = Body(name="robot_head")
world.add_body(robot_head)
camera = attach_camera_to_robot(world, robot_head)
```

## Revolute Connections

Revolute connections provide single-axis rotation:

```python
from semantic_world.connections import RevoluteConnection
from semantic_world.degree_of_freedom import DegreeOfFreedom

# Create degree of freedom for the joint
shoulder_dof = DegreeOfFreedom(
    name="shoulder_joint",
    lower_limit=-np.pi,      # -180 degrees
    upper_limit=np.pi,       # +180 degrees
    velocity_limit=2.0,      # rad/s
    acceleration_limit=5.0   # rad/s²
)

upper_arm = Body(name="upper_arm")
forearm = Body(name="forearm")

world.add_body(upper_arm)
world.add_body(forearm)

# Create revolute joint
elbow_joint = RevoluteConnection(
    parent=upper_arm,
    child=forearm,
    dof=shoulder_dof,
    axis=[1, 0, 0]  # Rotate around X-axis
)
world.add_connection(elbow_joint)

# Control the joint
elbow_joint.position = np.pi/4  # Set to 45 degrees
print(f"Elbow angle: {elbow_joint.position} rad ({np.degrees(elbow_joint.position):.1f}°)")

# Access DOF properties
print(f"Joint limits: [{elbow_joint.dof.lower_limit}, {elbow_joint.dof.upper_limit}]")
print(f"Current velocity: {elbow_joint.velocity}")
```

### Revolute Connection Configuration

```python
def create_revolute_joint(parent, child, name, axis, limits=None, offset=None):
    """Factory function for revolute connections"""
    
    # Set default limits if not provided
    if limits is None:
        limits = (-np.pi, np.pi)
    
    # Create DOF
    dof = DegreeOfFreedom(
        name=f"{name}_dof",
        lower_limit=limits[0],
        upper_limit=limits[1]
    )
    
    # Create connection
    joint = RevoluteConnection(
        parent=parent,
        child=child,
        dof=dof,
        axis=axis
    )
    
    # Add offset if specified
    if offset is not None:
        transform = np.eye(4)
        transform[:3, 3] = offset
        joint.origin_expression = TransformationMatrix(transform)
    
    return joint

# Create various revolute joints
shoulder_roll = create_revolute_joint(
    parent=torso, child=upper_arm,
    name="shoulder_roll", axis=[1, 0, 0],
    limits=(-np.pi/2, np.pi/2),
    offset=[0, 0.2, 0.4]  # Shoulder width and height
)

shoulder_pitch = create_revolute_joint(
    parent=upper_arm, child=forearm,
    name="shoulder_pitch", axis=[0, 1, 0],
    limits=(-np.pi, np.pi/6)
)

shoulder_yaw = create_revolute_joint(
    parent=forearm, child=hand,
    name="wrist_yaw", axis=[0, 0, 1],
    limits=(-np.pi, np.pi)
)
```

## Prismatic Connections

Prismatic connections provide single-axis translation:

```python
from semantic_world.connections import PrismaticConnection

# Create linear actuator
base_platform = Body(name="base_platform")
moving_platform = Body(name="moving_platform")

world.add_body(base_platform)
world.add_body(moving_platform)

# Linear DOF
linear_dof = DegreeOfFreedom(
    name="vertical_actuator",
    lower_limit=0.0,         # Minimum height
    upper_limit=1.5,         # Maximum height (1.5m)
    velocity_limit=0.5,      # 0.5 m/s
    acceleration_limit=2.0   # 2.0 m/s²
)

# Create prismatic joint
vertical_actuator = PrismaticConnection(
    parent=base_platform,
    child=moving_platform,
    dof=linear_dof,
    axis=[0, 0, 1]  # Move along Z-axis (vertical)
)
world.add_connection(vertical_actuator)

# Control the actuator
vertical_actuator.position = 0.8  # Extend to 80cm
print(f"Platform height: {vertical_actuator.position}m")

# Set velocity for motion
vertical_actuator.velocity = 0.2  # Moving up at 20cm/s
```

### Prismatic Joint Examples

```python
def create_linear_slide(world, parent_name, child_name, axis, travel_distance):
    """Create a linear sliding mechanism"""
    
    parent = Body(name=parent_name)
    child = Body(name=child_name)
    
    world.add_body(parent)
    world.add_body(child)
    
    # Create DOF with travel limits
    slide_dof = DegreeOfFreedom(
        name=f"{child_name}_slide",
        lower_limit=0.0,
        upper_limit=travel_distance
    )
    
    slide_joint = PrismaticConnection(
        parent=parent,
        child=child,
        dof=slide_dof,
        axis=axis
    )
    world.add_connection(slide_joint)
    
    return slide_joint

# Create XY table (2 orthogonal slides)
x_slide = create_linear_slide(world, "x_base", "x_carriage", [1, 0, 0], 0.5)
y_slide = create_linear_slide(world, "x_carriage", "xy_platform", [0, 1, 0], 0.3)

# Control XY position
x_slide.position = 0.2  # 20cm in X
y_slide.position = 0.15  # 15cm in Y

print(f"XY Platform position: ({x_slide.position}, {y_slide.position})")
```

## Connection6DoF (6-Degree-of-Freedom)

Connection6DoF provides full spatial freedom with up to 6 controllable degrees of freedom:

```python
from semantic_world.connections import Connection6DoF
from semantic_world.spatial_types.math import rotation_matrix_from_rpy

# Create floating connection (all 6 DOF)
floating_body = Body(name="floating_object")
world_frame = world.root

world.add_body(floating_body)

# Create 6-DOF connection for full positioning
floating_connection = Connection6DoF(
    parent=world_frame,
    child=floating_body
)
world.add_connection(floating_connection)

# Set 6-DOF pose using transformation matrix
pose = np.eye(4)
pose[:3, 3] = [1.0, 2.0, 3.0]  # Position
pose[:3, :3] = rotation_matrix_from_rpy(0.1, 0.2, 0.3)  # Orientation

floating_connection.origin = pose

print(f"6-DOF connection has {len(floating_connection.degrees_of_freedom)} DOFs")
```

### Constrained 6-DOF Connections

```python
def create_planar_joint(parent, child, plane_normal=[0, 0, 1]):
    """Create a connection constrained to move in a plane"""
    
    # Create DOFs for planar motion (3 DOF: x, y, rotation around normal)
    x_dof = DegreeOfFreedom(name="planar_x", lower_limit=-5.0, upper_limit=5.0)
    y_dof = DegreeOfFreedom(name="planar_y", lower_limit=-5.0, upper_limit=5.0)  
    rot_dof = DegreeOfFreedom(name="planar_rot", lower_limit=-np.pi, upper_limit=np.pi)
    
    planar_connection = Connection6DoF(
        parent=parent,
        child=child,
        translation_dofs=[x_dof, y_dof, None],  # X, Y free, Z constrained
        rotation_dofs=[None, None, rot_dof]     # Roll, Pitch constrained, Yaw free
    )
    
    return planar_connection

# Create mobile robot base (moves in ground plane)
robot_base = Body(name="mobile_base")
world.add_body(robot_base)

planar_joint = create_planar_joint(world.root, robot_base)
world.add_connection(planar_joint)
```

## Passive Connections

Passive connections have degrees of freedom but are not directly controllable. These would typically be implemented as custom classes extending PassiveConnection:

```python
from semantic_world.connections import PassiveConnection

# For passive connections, you would typically create custom classes
# Here's an example of how you might handle passive motion:

# Free-spinning wheel (passive rotation)
wheel = Body(name="wheel")
axle = Body(name="axle")

world.add_body(axle)
world.add_body(wheel)

# For now, use a fixed connection for non-actuated joints
# In practice, you'd implement a custom passive connection class
wheel_bearing = FixedConnection(parent=axle, child=wheel)
world.add_connection(wheel_bearing)

# Spring-loaded mechanism example
spring_base = Body(name="spring_base")
spring_mass = Body(name="spring_mass")

world.add_body(spring_base)
world.add_body(spring_mass)

# This would typically be a custom passive connection with physics
spring_connection = FixedConnection(parent=spring_base, child=spring_mass)
world.add_connection(spring_connection)

# Note: True passive connections with DOFs require custom implementation
# based on the PassiveConnection base class and physics simulation
```

### Custom Passive Connection Example

```python
@dataclass
class PassiveRevoluteConnection(PassiveConnection, Has1DOFState):
    """Custom implementation of a passive revolute connection"""
    
    axis: cas.Vector3 = field(kw_only=True)
    dof: DegreeOfFreedom = field(default=None)
    
    def __post_init__(self):
        super().__post_init__()
        self.axis = self.axis
        self._post_init_world_part()
        
        # Set up transformation based on DOF
        motor_expression = self.dof.symbols.position
        parent_R_child = cas.RotationMatrix.from_axis_angle(self.axis, motor_expression)
        self.origin_expression = self.origin_expression @ cas.TransformationMatrix(parent_R_child)
    
    @property
    def passive_dofs(self) -> List[DegreeOfFreedom]:
        return [self.dof]
```

## Shared Degrees of Freedom

Multiple connections can share the same degree of freedom for coupled motion:

```python
# Create shared DOF for synchronized motion
grip_dof = DegreeOfFreedom(
    name="gripper_width",
    lower_limit=0.0,    # Closed
    upper_limit=0.08,   # Open (8cm)
    velocity_limit=0.1
)

# Create parallel gripper fingers
gripper_base = Body(name="gripper_base")
left_finger = Body(name="left_finger")
right_finger = Body(name="right_finger")

for body in [gripper_base, left_finger, right_finger]:
    world.add_body(body)

# Both fingers share the same DOF but move in opposite directions
left_joint = PrismaticConnection(
    parent=gripper_base,
    child=left_finger,
    dof=grip_dof,         # Shared DOF
    axis=[-1, 0, 0]       # Move left (negative X)
)

right_joint = PrismaticConnection(
    parent=gripper_base, 
    child=right_finger,
    dof=grip_dof,         # Same shared DOF
    axis=[1, 0, 0]        # Move right (positive X)
)

world.add_connection(left_joint)
world.add_connection(right_joint)

# Controlling the shared DOF moves both fingers
grip_dof.position = 0.04  # Open gripper to 4cm
print(f"Left finger position: {left_joint.position}")   # -0.04
print(f"Right finger position: {right_joint.position}") # +0.04
```

## Advanced Connection Features

### Connection State Access

```python
# All active connections provide state access
def print_connection_state(connection):
    """Print current state of a connection"""
    print(f"Connection: {connection.name}")
    
    if hasattr(connection, 'position'):
        print(f"  Position: {connection.position}")
    if hasattr(connection, 'velocity'):
        print(f"  Velocity: {connection.velocity}")  
    if hasattr(connection, 'acceleration'):
        print(f"  Acceleration: {connection.acceleration}")
    
    if hasattr(connection, 'dof'):
        dof = connection.dof
        print(f"  Limits: [{dof.lower_limit}, {dof.upper_limit}]")
        print(f"  Vel Limit: {dof.velocity_limit}")
        print(f"  Acc Limit: {dof.acceleration_limit}")

# Print state for all controllable connections
for connection in world.connections:
    if hasattr(connection, 'position'):
        print_connection_state(connection)
```

### Connection Validation

```python
def validate_connection(connection):
    """Validate connection configuration"""
    issues = []
    
    # Check basic structure
    if connection.parent is None:
        issues.append("No parent body")
    if connection.child is None:
        issues.append("No child body")
    if connection.parent == connection.child:
        issues.append("Parent and child are the same body")
    
    # Check DOF limits for active connections
    if hasattr(connection, 'dof') and connection.dof:
        dof = connection.dof
        if dof.lower_limit >= dof.upper_limit:
            issues.append("Invalid DOF limits: lower >= upper")
        
        # Check current position within limits
        if hasattr(connection, 'position'):
            pos = connection.position
            if pos < dof.lower_limit or pos > dof.upper_limit:
                issues.append(f"Position {pos} outside limits [{dof.lower_limit}, {dof.upper_limit}]")
    
    # Check axis for revolute/prismatic
    if hasattr(connection, 'axis'):
        axis = np.array(connection.axis)
        if np.allclose(np.linalg.norm(axis), 0):
            issues.append("Axis vector has zero length")
    
    return issues

# Validate all connections
for connection in world.connections:
    issues = validate_connection(connection)
    if issues:
        print(f"Connection {connection.name} issues:")
        for issue in issues:
            print(f"  - {issue}")
```

### Connection Coordinate Frames

```python
# Understanding connection coordinate frames
def analyze_connection_frames(world, connection):
    """Analyze coordinate frames involved in a connection"""
    
    print(f"Connection: {connection.name}")
    print(f"Parent: {connection.parent.name}")
    print(f"Child: {connection.child.name}")
    
    # Get transform from parent to child
    parent_to_child = connection.origin
    print(f"Parent->Child transform:\n{parent_to_child}")
    
    # Get global poses
    parent_global = connection.parent.global_pose
    child_global = connection.child.global_pose
    
    print(f"Parent global pose:\n{parent_global}")
    print(f"Child global pose:\n{child_global}")
    
    # Verify consistency
    expected_child = parent_global @ parent_to_child
    if np.allclose(expected_child, child_global):
        print("✓ Frame transforms are consistent")
    else:
        print("✗ Frame transform mismatch!")

# Analyze a specific connection
analyze_connection_frames(world, elbow_joint)
```

## Connection Patterns and Best Practices

### Serial Chains

```python
def create_serial_chain(world, base_body, link_names, joint_types, joint_axes):
    """Create a serial kinematic chain"""
    
    bodies = [base_body]
    connections = []
    
    # Create bodies
    for name in link_names:
        body = Body(name=name)
        world.add_body(body)
        bodies.append(body)
    
    # Create connections
    for i in range(len(link_names)):
        parent = bodies[i]
        child = bodies[i + 1]
        joint_type = joint_types[i]
        axis = joint_axes[i]
        
        # Create DOF
        dof = DegreeOfFreedom(
            name=f"{child.name}_joint",
            lower_limit=-np.pi,
            upper_limit=np.pi
        )
        
        # Create appropriate connection type
        if joint_type == "revolute":
            connection = RevoluteConnection(parent=parent, child=child, dof=dof, axis=axis)
        elif joint_type == "prismatic":
            dof.lower_limit = 0.0
            dof.upper_limit = 1.0
            connection = PrismaticConnection(parent=parent, child=child, dof=dof, axis=axis)
        else:
            connection = FixedConnection(parent=parent, child=child)
        
        world.add_connection(connection)
        connections.append(connection)
    
    return bodies, connections

# Create 6-DOF robot arm
base = Body(name="robot_base")
world.add_body(base)

arm_bodies, arm_joints = create_serial_chain(
    world=world,
    base_body=base,
    link_names=["link1", "link2", "link3", "link4", "link5", "link6"],
    joint_types=["revolute"] * 6,
    joint_axes=[[0,0,1], [0,1,0], [0,1,0], [1,0,0], [0,1,0], [1,0,0]]
)
```

### Parallel Mechanisms

```python
def create_parallel_mechanism(world, base, platform, leg_count=3):
    """Create a parallel kinematic mechanism (e.g., Stewart platform)"""
    
    connections = []
    
    for i in range(leg_count):
        # Create actuator body for each leg
        actuator = Body(name=f"actuator_{i}")
        world.add_body(actuator)
        
        # Create prismatic joint for actuation
        actuator_dof = DegreeOfFreedom(
            name=f"actuator_{i}_length",
            lower_limit=0.5,
            upper_limit=1.5
        )
        
        actuator_joint = PrismaticConnection(
            parent=base,
            child=actuator,
            dof=actuator_dof,
            axis=[0, 0, 1]  # Extend vertically
        )
        world.add_connection(actuator_joint)
        
        # Connect actuator to platform with spherical joint
        # (approximated with 6-DOF connection)
        spherical_joint = Connection6DoF(parent=actuator, child=platform)
        world.add_connection(spherical_joint)
        
        connections.extend([actuator_joint, spherical_joint])
    
    return connections

# Create Stewart platform
platform_base = Body(name="platform_base")
platform_top = Body(name="platform_top")

world.add_body(platform_base)
world.add_body(platform_top)

stewart_joints = create_parallel_mechanism(world, platform_base, platform_top, leg_count=6)
```

## Connection Performance Optimization

### Batch State Updates

```python
# Efficient batch updates
def set_joint_positions(world, position_dict):
    """Set multiple joint positions efficiently"""
    
    # Group by connection type for batch processing
    revolute_updates = {}
    prismatic_updates = {}
    
    for connection, position in position_dict.items():
        if isinstance(connection, RevoluteConnection):
            revolute_updates[connection] = position
        elif isinstance(connection, PrismaticConnection):
            prismatic_updates[connection] = position
    
    # Apply updates in batches
    if revolute_updates:
        world.set_positions_1DOF_connection(revolute_updates)
    if prismatic_updates:
        world.set_positions_1DOF_connection(prismatic_updates)

# Usage
joint_positions = {
    shoulder_joint: np.pi/4,
    elbow_joint: -np.pi/3,
    wrist_joint: np.pi/6,
    linear_actuator: 0.5
}

set_joint_positions(world, joint_positions)
```

### Connection Caching

```python
# Cache expensive computations
def get_connection_jacobian(world, connection, target_body):
    """Get Jacobian for a connection (with caching)"""
    cache_key = f"jacobian_{connection.name}_{target_body.name}"
    
    if not hasattr(world, '_jacobian_cache'):
        world._jacobian_cache = {}
    
    if cache_key not in world._jacobian_cache:
        # Compute Jacobian (expensive operation)
        jacobian = compute_jacobian(world, connection, target_body)
        world._jacobian_cache[cache_key] = jacobian
    
    return world._jacobian_cache[cache_key]

def invalidate_connection_cache(world):
    """Invalidate cached connection computations"""
    if hasattr(world, '_jacobian_cache'):
        world._jacobian_cache.clear()
```

This comprehensive guide covers all aspects of connections in semantic worlds. The key principles for effective connection usage are:

1. **Choose appropriate connection types** for your kinematic requirements
2. **Use shared DOFs** for coupled motion systems
3. **Validate connections** thoroughly during development
4. **Optimize state updates** with batch operations
5. **Understand coordinate frames** for proper kinematic modeling
6. **Cache expensive computations** when dealing with complex mechanisms

Next, explore the [Getting Started Guide](getting_started.md) for basic usage or the [World Construction Guide](world_construction.md) for building complex systems.