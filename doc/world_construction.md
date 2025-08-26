# World Construction Guide

This guide covers advanced topics for constructing and managing semantic worlds, including complex kinematic structures, world merging, and state management.

## World Class Overview

The `World` class is the central data structure that manages the complete scene graph. It maintains:

- **Kinematic Structure**: A directed acyclic graph (DAG) of bodies and connections
- **Views**: Semantic interpretations of world structures  
- **Degrees of Freedom**: All controllable parameters in the world
- **State**: Current positions, velocities, and accelerations of all DOFs

```python
from semantic_world.world import World
from semantic_world.world_state import WorldState

# Create world with custom initial state
initial_state = WorldState()
world = World(state=initial_state)

# Access world properties
print(f"Bodies: {len(world.bodies)}")
print(f"Connections: {len(world.connections)}")
print(f"DOFs: {len(world.degrees_of_freedom)}")
print(f"Views: {len(world.views)}")
```

## Building Complex Kinematic Structures

### Tree Structures

Most robotic systems have tree-like kinematic structures:

```python
from semantic_world.world import World
from semantic_world.world_entity import Body
from semantic_world.connections import RevoluteConnection, FixedConnection
from semantic_world.degree_of_freedom import DegreeOfFreedom
import numpy as np

def build_robot_tree():
    world = World()
    
    # Create the robot base
    base = Body(name="robot_base")
    world.add_body(base)
    
    # Create torso
    torso = Body(name="torso")
    world.add_body(torso)
    
    base_torso_joint = RevoluteConnection(
        parent=base,
        child=torso,
        dof=DegreeOfFreedom(name="base_torso", lower_limit=-np.pi, upper_limit=np.pi),
        axis=[0, 0, 1]
    )
    world.add_connection(base_torso_joint)
    
    # Create arms branching from torso
    left_arm = create_arm_chain(world, torso, "left", [-0.3, 0, 0.5])
    right_arm = create_arm_chain(world, torso, "right", [0.3, 0, 0.5])
    
    # Create head
    head = Body(name="head")
    world.add_body(head)
    
    head_joint = RevoluteConnection(
        parent=torso,
        child=head,
        dof=DegreeOfFreedom(name="neck", lower_limit=-np.pi/2, upper_limit=np.pi/2),
        axis=[0, 0, 1]
    )
    world.add_connection(head_joint)
    
    return world

def create_arm_chain(world, parent_body, side, offset):
    """Create a 3-DOF arm chain"""
    shoulder = Body(name=f"{side}_shoulder")
    upper_arm = Body(name=f"{side}_upper_arm") 
    forearm = Body(name=f"{side}_forearm")
    hand = Body(name=f"{side}_hand")
    
    for body in [shoulder, upper_arm, forearm, hand]:
        world.add_body(body)
    
    # Shoulder connection with offset
    shoulder_joint = RevoluteConnection(
        parent=parent_body,
        child=shoulder,
        dof=DegreeOfFreedom(name=f"{side}_shoulder", lower_limit=-np.pi, upper_limit=np.pi),
        axis=[0, 1, 0]
    )
    # Set offset transform
    transform = np.eye(4)
    transform[:3, 3] = offset
    shoulder_joint.origin_expression.matrix = transform
    world.add_connection(shoulder_joint)
    
    # Upper arm joint
    upper_arm_joint = RevoluteConnection(
        parent=shoulder,
        child=upper_arm,
        dof=DegreeOfFreedom(name=f"{side}_upper_arm", lower_limit=-np.pi/2, upper_limit=np.pi/2),
        axis=[1, 0, 0]
    )
    world.add_connection(upper_arm_joint)
    
    # Forearm joint
    forearm_joint = RevoluteConnection(
        parent=upper_arm,
        child=forearm, 
        dof=DegreeOfFreedom(name=f"{side}_elbow", lower_limit=0, upper_limit=np.pi),
        axis=[1, 0, 0]
    )
    world.add_connection(forearm_joint)
    
    # Hand - fixed connection
    hand_connection = FixedConnection(parent=forearm, child=hand)
    world.add_connection(hand_connection)
    
    return [shoulder, upper_arm, forearm, hand]

# Build the complete robot
robot_world = build_robot_tree()
robot_world.validate()
```

### Parallel Kinematic Structures

Some systems have parallel kinematic chains:

```python
def build_parallel_gripper(world, parent_body):
    """Build a parallel gripper with synchronized fingers"""
    
    # Create gripper base
    gripper_base = Body(name="gripper_base")
    world.add_body(gripper_base)
    
    base_connection = FixedConnection(parent=parent_body, child=gripper_base)
    world.add_connection(base_connection)
    
    # Create shared DOF for synchronized movement
    grip_dof = DegreeOfFreedom(
        name="grip_width", 
        lower_limit=0.0, 
        upper_limit=0.1
    )
    
    # Create left finger
    left_finger = Body(name="left_finger")
    world.add_body(left_finger)
    
    left_joint = PrismaticConnection(
        parent=gripper_base,
        child=left_finger,
        dof=grip_dof,  # Shared DOF
        axis=[-1, 0, 0]  # Move in -x direction
    )
    world.add_connection(left_joint)
    
    # Create right finger  
    right_finger = Body(name="right_finger")
    world.add_body(right_finger)
    
    right_joint = PrismaticConnection(
        parent=gripper_base,
        child=right_finger,
        dof=grip_dof,  # Same shared DOF
        axis=[1, 0, 0]  # Move in +x direction  
    )
    world.add_connection(right_joint)
    
    return gripper_base, [left_finger, right_finger]

# Add gripper to existing robot
from semantic_world.connections import PrismaticConnection

left_arm_bodies = robot_world.get_bodies_by_name_pattern("left_hand")[0] 
gripper_base, fingers = build_parallel_gripper(robot_world, left_arm_bodies)
```

## World Merging and Composition

Worlds can be merged to combine separate kinematic structures:

```python
def create_table_world():
    """Create a world with a table"""
    world = World()
    
    table_top = Body(name="table_top")
    table_leg1 = Body(name="table_leg1")
    table_leg2 = Body(name="table_leg2") 
    table_leg3 = Body(name="table_leg3")
    table_leg4 = Body(name="table_leg4")
    
    for body in [table_top, table_leg1, table_leg2, table_leg3, table_leg4]:
        world.add_body(body)
        
    # Connect legs to table top
    for i, leg in enumerate([table_leg1, table_leg2, table_leg3, table_leg4]):
        connection = FixedConnection(parent=table_top, child=leg)
        world.add_connection(connection)
    
    return world

def merge_worlds_example():
    """Demonstrate world merging"""
    
    # Create separate worlds
    robot_world = build_robot_tree()
    table_world = create_table_world()
    
    # Merge table world into robot world
    merged_world = robot_world.copy()  # Create copy to preserve original
    
    # Method 1: Merge with fixed pose
    table_pose = np.eye(4)
    table_pose[0, 3] = 1.0  # 1 meter in front of robot
    merged_world.merge_with_pose(table_world, table_pose)
    
    # Method 2: Merge with connection
    # This creates a connection between worlds
    robot_base = merged_world.get_body_by_name("robot_base")
    table_connection = FixedConnection(
        parent=robot_base,
        child=table_world.root
    )
    merged_world.add_connection(table_connection)
    
    return merged_world

# Create merged environment
environment = merge_worlds_example()
environment.validate()
```

## State Management

### Setting and Getting Joint Positions

```python
# Set individual joint positions
robot_world.set_positions_1DOF_connection({
    base_torso_joint: np.pi/4,
    left_shoulder_joint: -np.pi/6,
    right_shoulder_joint: np.pi/6
})

# Get current joint positions
current_positions = {}
for connection in robot_world.connections:
    if hasattr(connection, 'position'):
        current_positions[connection.name] = connection.position

print("Current joint positions:", current_positions)

# Access DOF states directly
for dof in robot_world.degrees_of_freedom:
    print(f"{dof.name}: pos={dof.position}, vel={dof.velocity}")
```

### State Context Managers

Use context managers for temporary state changes:

```python
# Save current state and restore it automatically
with robot_world.reset_state_context():
    # Make temporary changes
    robot_world.set_positions_1DOF_connection({
        base_torso_joint: np.pi/2
    })
    
    # Compute something with changed state
    temp_transform = robot_world.compute_forward_kinematics(
        robot_world.root, head
    )
    
    print(f"Temporary head position: {temp_transform[:3, 3]}")
    
# State is automatically restored here
final_transform = robot_world.compute_forward_kinematics(
    robot_world.root, head  
)
print(f"Restored head position: {final_transform[:3, 3]}")
```

## Dynamic World Modification

Worlds can be modified at runtime:

```python
def add_tool_to_robot(world, attachment_body, tool_name):
    """Dynamically add a tool to the robot"""
    
    # Create tool body
    tool = Body(name=tool_name)
    world.add_body(tool)
    
    # Create connection
    tool_connection = FixedConnection(
        parent=attachment_body,
        child=tool
    )
    world.add_connection(tool_connection)
    
    # Invalidate caches after modification
    world.reset_cache()
    world.validate()
    
    return tool

def remove_tool_from_robot(world, tool_body):
    """Remove a tool from the robot"""
    
    # Find and remove connections involving this body
    connections_to_remove = []
    for connection in world.connections:
        if connection.parent == tool_body or connection.child == tool_body:
            connections_to_remove.append(connection)
    
    for connection in connections_to_remove:
        world.remove_connection(connection)
    
    # Remove the body itself
    world.remove_body(tool_body)
    
    # Clean up and validate
    world.reset_cache()
    world.validate()

# Example usage
left_hand = robot_world.get_body_by_name("left_hand")
screwdriver = add_tool_to_robot(robot_world, left_hand, "screwdriver")

# Use the tool...

# Remove when done
remove_tool_from_robot(robot_world, screwdriver)
```

## World Validation and Debugging

The world provides validation and debugging utilities:

```python
# Validate world structure
try:
    robot_world.validate()
    print("World structure is valid")
except Exception as e:
    print(f"Validation failed: {e}")

# Get kinematic chains
chain = robot_world.compute_chain_of_bodies(
    root=robot_world.root,
    tip=left_hand
)
print("Kinematic chain:", [body.name for body in chain])

# Check for cycles (should not exist in valid world)
try:
    cycle = robot_world.find_cycle()
    if cycle:
        print("Warning: Cycle detected!", cycle)
except:
    print("No cycles found (good)")

# Visualize world structure  
robot_world.plot_kinematic_graph(
    filename="robot_structure.png",
    show_labels=True
)
```

## Performance Considerations

### Caching and Updates

The world uses caching for expensive computations:

```python
# Forward kinematics are cached
transform1 = robot_world.compute_forward_kinematics(robot_world.root, left_hand)  # Computed
transform2 = robot_world.compute_forward_kinematics(robot_world.root, left_hand)  # Cached

# Cache is invalidated when state changes
robot_world.set_positions_1DOF_connection({base_torso_joint: np.pi/3})
transform3 = robot_world.compute_forward_kinematics(robot_world.root, left_hand)  # Recomputed

# Manual cache management
robot_world.reset_cache()  # Clear all caches
```

### Batch Operations

For efficiency, batch multiple operations:

```python
# Instead of multiple individual updates:
# robot_world.set_positions_1DOF_connection({joint1: pos1})
# robot_world.set_positions_1DOF_connection({joint2: pos2})

# Do batch update:
robot_world.set_positions_1DOF_connection({
    joint1: pos1,
    joint2: pos2,
    joint3: pos3
})
```

## Advanced World Features

### Custom Root Bodies

By default, worlds have a root body, but you can customize this:

```python
# Create world with custom root
custom_root = Body(name="world_origin")  
world = World()
world.add_body(custom_root)
world.set_root(custom_root)
```

### World Copying

Create copies of worlds for simulation or planning:

```python
# Deep copy preserves all structure and state
world_copy = robot_world.copy()

# Shallow copy shares some data structures (use carefully)  
world_shallow = robot_world.copy(deep=False)
```

### Coordinate Frame Transformations

Transform objects between different coordinate frames:

```python
# Transform a point from one body frame to another
point_in_hand_frame = [0.1, 0, 0]  # 10cm forward from hand
point_in_base_frame = robot_world.transform(
    spatial_object=point_in_hand_frame,
    target_frame=robot_world.root
)

# Transform poses
pose_in_hand = np.eye(4)
pose_in_base = robot_world.transform(pose_in_hand, robot_world.root)
```

This guide covers the essential patterns for building and managing complex semantic worlds. The key principles are:

1. Build incrementally and validate frequently
2. Use appropriate connection types for your kinematic needs
3. Leverage world merging for complex environments  
4. Manage state carefully with context managers
5. Use caching and batch operations for performance
6. Validate and debug thoroughly

Next, explore the [Body Management Guide](body_management.md) and [Connection Types Guide](connection_types.md) for more specific details.