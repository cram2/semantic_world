---
jupytext:
  text_representation:
    extension: .md
    format_name: markdown
    format_version: '1.3'
    jupytext_version: 1.15.2
kernelspec:
  display_name: Python 3
  language: python
  name: python3
---

# Connection Types Examples

This notebook demonstrates all connection types available in the Semantic World library with practical examples.

```python
import numpy as np
from semantic_world.world import World
from semantic_world.world_entity import Body
from semantic_world.connections import (
    FixedConnection, RevoluteConnection, PrismaticConnection, Connection6DoF
)
from semantic_world.degree_of_freedom import DegreeOfFreedom
from semantic_world.geometry import Box, Cylinder, Sphere
```

## Connection Type Hierarchy

Let me start by creating a function to visualize the connection hierarchy:

```python
def print_connection_hierarchy():
    """Generate connection type hierarchy from actual classes"""
    
    from semantic_world.world_entity import Connection
    from semantic_world.connections import (
        FixedConnection, RevoluteConnection, PrismaticConnection, Connection6DoF
    )
    
    # Import all connection classes and build hierarchy
    connection_classes = [
        Connection, FixedConnection, RevoluteConnection, PrismaticConnection, Connection6DoF
    ]
    
    print("Connection Type Hierarchy:")
    print("Connection (base class)")
    
    # Find direct subclasses
    for cls in connection_classes[1:]:  # Skip base Connection class
        if issubclass(cls, Connection):
            print(f"├── {cls.__name__}")
            if hasattr(cls, '__doc__') and cls.__doc__:
                doc_lines = cls.__doc__.strip().split('\n')
                if doc_lines:
                    print(f"│   └── {doc_lines[0].strip()}")
    
    print("\nConnection Features:")
    print("• FixedConnection: Rigid attachment, 0 DOF")
    print("• RevoluteConnection: Single rotational DOF")  
    print("• PrismaticConnection: Single translational DOF")
    print("• Connection6DoF: Up to 6 DOF for free-floating bodies")

print_connection_hierarchy()
```

## Setup: Creating Test Bodies

```python
# Create world and test bodies for connection examples
world = World()

with world.modify_world():
    # Create a set of test bodies for different connection examples
    base_platform = Body(
        name="base_platform",
        visual=[Box(size=[0.8, 0.8, 0.1])],
        collision=[Box(size=[0.8, 0.8, 0.1])]
    )
    
    rotating_arm = Body(
        name="rotating_arm", 
        visual=[Box(size=[0.05, 0.05, 0.4])],
        collision=[Box(size=[0.05, 0.05, 0.4])]
    )
    
    sliding_block = Body(
        name="sliding_block",
        visual=[Box(size=[0.1, 0.1, 0.1])],
        collision=[Box(size=[0.1, 0.1, 0.1])]
    )
    
    floating_object = Body(
        name="floating_object",
        visual=[Sphere(radius=0.1)],
        collision=[Sphere(radius=0.1)]
    )
    
    fixed_attachment = Body(
        name="fixed_attachment",
        visual=[Cylinder(radius=0.03, height=0.2)],
        collision=[Cylinder(radius=0.03, height=0.2)]
    )
    
    world.add_body(base_platform)
    world.add_body(rotating_arm)
    world.add_body(sliding_block)
    world.add_body(floating_object)
    world.add_body(fixed_attachment)

print(f"Created {len(world.bodies)} test bodies")
```

## Fixed Connections

Fixed connections create rigid attachments between bodies with no degrees of freedom:

```python
with world.modify_world():
    # Create a fixed connection between base and attachment
    fixed_conn = FixedConnection(
        parent=base_platform,
        child=fixed_attachment
    )
    world.add_connection(fixed_conn)
    
    # Set the fixed transform (optional, defaults to identity)
    fixed_transform = np.eye(4)
    fixed_transform[:3, 3] = [0.2, 0.2, 0.1]  # Offset position
    fixed_conn.origin = fixed_transform
    
    print(f"Created fixed connection: {fixed_conn.parent.name} -> {fixed_conn.child.name}")
    print(f"Connection has {0} degrees of freedom")
```

## Revolute Connections

Revolute connections provide single-axis rotation:

```python
with world.modify_world():
    # Create degree of freedom for revolute joint
    revolute_dof = DegreeOfFreedom(
        name="arm_rotation",
        lower_limit=-np.pi,
        upper_limit=np.pi,
        velocity_limit=2.0,
        acceleration_limit=5.0
    )
    
    # Create revolute connection
    revolute_conn = RevoluteConnection(
        parent=base_platform,
        child=rotating_arm,
        dof=revolute_dof,
        axis=[0, 0, 1]  # Rotation around Z-axis
    )
    world.add_connection(revolute_conn)
    
    print(f"Created revolute connection: {revolute_conn.parent.name} -> {revolute_conn.child.name}")
    print(f"Rotation axis: {revolute_conn.axis}")
    print(f"Joint limits: {np.degrees(revolute_dof.lower_limit):.1f}° to {np.degrees(revolute_dof.upper_limit):.1f}°")
```

### Controlling Revolute Joints

```python
# Control the revolute joint
revolute_conn.position = np.pi/4  # Set to 45 degrees
print(f"Revolute joint set to: {np.degrees(revolute_conn.position):.1f} degrees")

# Access joint properties
print(f"Current position: {revolute_conn.position:.3f} rad")
print(f"Current velocity: {revolute_conn.velocity:.3f} rad/s")
print(f"Within limits: {revolute_dof.lower_limit <= revolute_conn.position <= revolute_dof.upper_limit}")
```

## Prismatic Connections  

Prismatic connections provide single-axis translation:

```python
with world.modify_world():
    # Create degree of freedom for prismatic joint
    prismatic_dof = DegreeOfFreedom(
        name="block_slide",
        lower_limit=0.0,
        upper_limit=0.5,  # 50cm travel
        velocity_limit=0.2,
        acceleration_limit=1.0
    )
    
    # Create prismatic connection  
    prismatic_conn = PrismaticConnection(
        parent=rotating_arm,  # Child of the revolute joint
        child=sliding_block,
        dof=prismatic_dof,
        axis=[0, 0, 1]  # Translation along Z-axis (arm length direction)
    )
    world.add_connection(prismatic_conn)
    
    print(f"Created prismatic connection: {prismatic_conn.parent.name} -> {prismatic_conn.child.name}")
    print(f"Translation axis: {prismatic_conn.axis}")
    print(f"Travel range: {prismatic_dof.lower_limit:.2f}m to {prismatic_dof.upper_limit:.2f}m")
```

### Controlling Prismatic Joints

```python
# Control the prismatic joint
prismatic_conn.position = 0.25  # Extend to 25cm
print(f"Prismatic joint extended to: {prismatic_conn.position*100:.1f}cm")

# Test limit enforcement
try:
    prismatic_conn.position = 0.8  # Attempt to exceed limit
except ValueError as e:
    print(f"Limit enforcement: {e}")

# Set to valid position
prismatic_conn.position = 0.4  # 40cm extension
print(f"Valid extension: {prismatic_conn.position*100:.1f}cm")
```

## 6-DOF Connections

6-DOF connections allow full spatial positioning (translation + rotation):

```python
with world.modify_world():
    # Create 6-DOF connection for free-floating object
    # Note: 6-DOF connections don't use individual DegreeOfFreedom objects
    # Instead, they use the full transformation matrix
    sixdof_conn = Connection6DoF(
        parent=base_platform,  # Often connected to world root
        child=floating_object
    )
    world.add_connection(sixdof_conn)
    
    print(f"Created 6-DOF connection: {sixdof_conn.parent.name} -> {sixdof_conn.child.name}")
    print("This connection provides full 6-DOF positioning capability")
```

### Controlling 6-DOF Joints

```python
# Set 6-DOF pose using transformation matrix
pose_6dof = np.eye(4)

# Set position (translation)
pose_6dof[0, 3] = 0.3   # X: 30cm
pose_6dof[1, 3] = -0.2  # Y: -20cm  
pose_6dof[2, 3] = 0.4   # Z: 40cm

# Set orientation (rotation matrix)
# 45-degree rotation around Z-axis
angle = np.pi/4
pose_6dof[0, 0] = np.cos(angle)
pose_6dof[0, 1] = -np.sin(angle)
pose_6dof[1, 0] = np.sin(angle) 
pose_6dof[1, 1] = np.cos(angle)

# Apply the pose
sixdof_conn.origin = pose_6dof

print("6-DOF object positioned at:")
print(f"Position: [{pose_6dof[0,3]:.2f}, {pose_6dof[1,3]:.2f}, {pose_6dof[2,3]:.2f}]")
print(f"Rotation: {np.degrees(angle):.1f}° around Z-axis")
```

## Advanced Connection Features

### Shared DOF Connections

Multiple connections can share the same degree of freedom:

```python
with world.modify_world():
    # Create additional bodies for shared DOF example
    gear1 = Body(name="gear_1", visual=[Cylinder(radius=0.1, height=0.02)])
    gear2 = Body(name="gear_2", visual=[Cylinder(radius=0.15, height=0.02)])
    world.add_body(gear1)
    world.add_body(gear2)
    
    # Shared DOF - both gears rotate together with fixed ratio
    shared_dof = DegreeOfFreedom(
        name="gear_system",
        lower_limit=-10*np.pi,  # Multiple rotations
        upper_limit=10*np.pi,
        velocity_limit=5.0
    )
    
    # Both gears use the same DOF but with different axes/ratios
    gear1_conn = RevoluteConnection(
        parent=base_platform,
        child=gear1,
        dof=shared_dof,
        axis=[0, 0, 1]
    )
    
    # Gear 2 rotates in opposite direction (simulating meshed gears)
    gear2_conn = RevoluteConnection(
        parent=base_platform,
        child=gear2,
        dof=shared_dof,  # Same DOF!
        axis=[0, 0, -1]  # Opposite rotation direction
    )
    
    world.add_connection(gear1_conn)
    world.add_connection(gear2_conn)
    
    print("Created gear system with shared DOF")
    print(f"Both gears controlled by DOF: {shared_dof.name}")
```

### Testing Shared DOF

```python
# Control shared DOF - both gears should move
shared_dof_position = np.pi/2  # 90 degrees

# Update the shared DOF through world state
world.state[shared_dof.name].position = shared_dof_position

print(f"Gear system rotated to: {np.degrees(shared_dof_position):.1f}°")
print(f"Gear 1 position: {gear1_conn.position:.3f} rad")
print(f"Gear 2 position: {gear2_conn.position:.3f} rad (opposite direction)")
```

## Batch Control of Multiple Connections

```python
# Control multiple joints simultaneously using batch update
joint_positions = {
    revolute_conn: np.pi/6,    # 30 degrees
    prismatic_conn: 0.3,       # 30cm extension
    gear1_conn: np.pi/3,       # 60 degrees (affects both gears)
}

world.set_positions_1DOF_connection(joint_positions)

print("Batch update applied to multiple connections:")
for connection, position in joint_positions.items():
    if hasattr(connection, 'dof'):
        print(f"  {connection.dof.name}: {position:.3f}")
```

## Forward Kinematics with Connections

```python
# Compute forward kinematics through the kinematic chain
# Base -> Rotating Arm -> Sliding Block
try:
    # Compute pose of sliding block relative to base
    block_pose = world.compute_forward_kinematics(base_platform, sliding_block)
    
    print("\nForward kinematics result:")
    print(f"Sliding block position relative to base: {block_pose[:3, 3]}")
    print(f"Distance from base: {np.linalg.norm(block_pose[:3, 3]):.3f}m")
    
    # Compute pose of floating object (6-DOF)
    float_pose = world.compute_forward_kinematics(base_platform, floating_object)
    print(f"Floating object position: {float_pose[:3, 3]}")
    
except Exception as e:
    print(f"Forward kinematics error: {e}")
```

## Connection Validation and Diagnostics

```python
# Validate all connections
print("\nConnection validation:")
for i, connection in enumerate(world.connections):
    try:
        # Check connection properties
        parent_name = connection.parent.name if connection.parent else "None"
        child_name = connection.child.name if connection.child else "None"
        
        print(f"Connection {i+1}: {parent_name} -> {child_name}")
        print(f"  Type: {type(connection).__name__}")
        
        if hasattr(connection, 'dof') and connection.dof:
            print(f"  DOF: {connection.dof.name}")
            print(f"  Position: {connection.position:.3f}")
            print(f"  Limits: [{connection.dof.lower_limit:.2f}, {connection.dof.upper_limit:.2f}]")
        elif hasattr(connection, 'origin'):
            print(f"  Origin: 4x4 transformation matrix")
        else:
            print(f"  No DOF or origin specified")
            
    except Exception as e:
        print(f"  Error validating connection: {e}")

# Overall world validation
try:
    world.validate()
    print("\n✓ All connections and world structure validated successfully")
except Exception as e:
    print(f"\n✗ World validation failed: {e}")
```

## Connection Performance Considerations

```python
# Analyze connection complexity
def analyze_connections(world):
    connection_stats = {
        'total': len(world.connections),
        'fixed': 0,
        'revolute': 0,
        'prismatic': 0,
        'sixdof': 0,
        'total_dofs': len(world.degrees_of_freedom),
        'shared_dofs': 0
    }
    
    dof_usage = {}
    
    for conn in world.connections:
        conn_type = type(conn).__name__.lower()
        if 'fixed' in conn_type:
            connection_stats['fixed'] += 1
        elif 'revolute' in conn_type:
            connection_stats['revolute'] += 1
        elif 'prismatic' in conn_type:
            connection_stats['prismatic'] += 1
        elif '6dof' in conn_type:
            connection_stats['sixdof'] += 1
            
        # Check for shared DOFs
        if hasattr(conn, 'dof') and conn.dof:
            dof_name = conn.dof.name
            dof_usage[dof_name] = dof_usage.get(dof_name, 0) + 1
    
    # Count shared DOFs
    connection_stats['shared_dofs'] = sum(1 for count in dof_usage.values() if count > 1)
    
    return connection_stats, dof_usage

stats, dof_usage = analyze_connections(world)

print("\n=== Connection Analysis ===")
print(f"Total connections: {stats['total']}")
print(f"  Fixed: {stats['fixed']}")
print(f"  Revolute: {stats['revolute']}")  
print(f"  Prismatic: {stats['prismatic']}")
print(f"  6-DOF: {stats['sixdof']}")
print(f"Total DOFs: {stats['total_dofs']}")
print(f"Shared DOFs: {stats['shared_dofs']}")

print("\nDOF usage:")
for dof_name, usage_count in dof_usage.items():
    shared_indicator = " (SHARED)" if usage_count > 1 else ""
    print(f"  {dof_name}: {usage_count} connection(s){shared_indicator}")
```

## Summary

```python
print("\n=== Connection Types Summary ===")
print("✓ Fixed Connections: Rigid attachments, no DOF")
print("✓ Revolute Connections: Single rotational DOF with limits")
print("✓ Prismatic Connections: Single translational DOF with limits")  
print("✓ 6-DOF Connections: Full spatial positioning")
print("✓ Shared DOF: Multiple connections controlled by same parameter")
print("✓ Batch Control: Efficient simultaneous joint updates")
print("✓ Forward Kinematics: Pose computation through kinematic chains")
print("✓ Validation: Comprehensive connection and world structure checking")

print(f"\nDemo world created with:")
print(f"- {len(world.bodies)} bodies")
print(f"- {len(world.connections)} connections") 
print(f"- {len(world.degrees_of_freedom)} degrees of freedom")
```