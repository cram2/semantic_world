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

# Robot Manipulator Construction

This notebook demonstrates how to construct a complete 6-DOF robot manipulator using the Semantic World library.

```python
import numpy as np
from semantic_world.world import World
from semantic_world.world_entity import Body
from semantic_world.connections import RevoluteConnection, FixedConnection
from semantic_world.degree_of_freedom import DegreeOfFreedom
from semantic_world.geometry import Box, Cylinder, Sphere
```

## Creating the Robot Base

First, we create a world and add the robot base:

```python
# Create world using context manager for modifications
world = World()

with world.modify_world():
    # Create robot base
    base = Body(
        name="robot_base",
        visual=[Cylinder(radius=0.2, height=0.1)],
        collision=[Cylinder(radius=0.2, height=0.1)]
    )
    world.add_body(base)
```

## Building the Robot Arm

Now let's add the robot arm segments using proper kinematic chain construction:

```python
# Robot arm parameters (simplified 6-DOF)
link_lengths = [0.3, 0.4, 0.4, 0.3, 0.2, 0.1]
joint_limits = [
    (-np.pi, np.pi),      # Base rotation
    (-np.pi/2, np.pi/2),  # Shoulder
    (-np.pi/2, np.pi/2),  # Elbow
    (-np.pi, np.pi),      # Wrist 1
    (-np.pi/2, np.pi/2),  # Wrist 2
    (-np.pi, np.pi)       # Wrist 3
]

# Create arm segments and joints
with world.modify_world():
    previous_body = base
    
    for i, (length, (lower, upper)) in enumerate(zip(link_lengths, joint_limits)):
        # Create link body
        link = Body(
            name=f"link_{i+1}",
            visual=[Box(size=[0.05, 0.05, length])],
            collision=[Box(size=[0.05, 0.05, length])]
        )
        world.add_body(link)
        
        # Create degree of freedom
        dof = DegreeOfFreedom(
            name=f"joint_{i+1}",
            lower_limit=lower,
            upper_limit=upper,
            velocity_limit=2.0,
            acceleration_limit=5.0
        )
        
        # Create revolute joint
        if i < 3:  # First 3 joints around Z-axis
            axis = [0, 0, 1]
        else:  # Last 3 joints around X-axis  
            axis = [1, 0, 0]
            
        joint = RevoluteConnection(
            parent=previous_body,
            child=link,
            dof=dof,
            axis=axis
        )
        world.add_connection(joint)
        
        previous_body = link

print(f"Created robot with {len(world.bodies)} bodies and {len(world.connections)} connections")
```

## Adding an End-Effector

Let's add a simple gripper end-effector:

```python
with world.modify_world():
    # Create gripper base
    gripper_base = Body(
        name="gripper_base",
        visual=[Box(size=[0.1, 0.05, 0.02])],
        collision=[Box(size=[0.1, 0.05, 0.02])]
    )
    world.add_body(gripper_base)
    
    # Fixed connection to last link
    gripper_mount = FixedConnection(
        parent=previous_body,
        child=gripper_base
    )
    world.add_connection(gripper_mount)
    
    # Create gripper fingers
    finger1 = Body(
        name="finger_1",
        visual=[Box(size=[0.02, 0.01, 0.05])],
        collision=[Box(size=[0.02, 0.01, 0.05])]
    )
    
    finger2 = Body(
        name="finger_2", 
        visual=[Box(size=[0.02, 0.01, 0.05])],
        collision=[Box(size=[0.02, 0.01, 0.05])]
    )
    
    world.add_body(finger1)
    world.add_body(finger2)
    
    # Create prismatic joints for gripper
    finger1_dof = DegreeOfFreedom(
        name="finger_1_joint",
        lower_limit=0.0,
        upper_limit=0.04,
        velocity_limit=0.1
    )
    
    finger2_dof = DegreeOfFreedom(
        name="finger_2_joint", 
        lower_limit=0.0,
        upper_limit=0.04,
        velocity_limit=0.1
    )
    
    # Prismatic connections for gripper
    from semantic_world.connections import PrismaticConnection
    
    finger1_joint = PrismaticConnection(
        parent=gripper_base,
        child=finger1,
        dof=finger1_dof,
        axis=[0, 1, 0]  # Y-axis movement
    )
    
    finger2_joint = PrismaticConnection(
        parent=gripper_base,
        child=finger2,
        dof=finger2_dof,
        axis=[0, -1, 0]  # Opposite Y-axis movement
    )
    
    world.add_connection(finger1_joint)
    world.add_connection(finger2_joint)

print(f"Complete robot has {len(world.bodies)} bodies and {len(world.connections)} connections")
```

## Validating the Robot

Always validate the world after construction:

```python
# Validate the world structure
try:
    world.validate()
    print("✓ Robot world validation successful")
except Exception as e:
    print(f"✗ Validation failed: {e}")
```

## Controlling the Robot

Now let's control the robot joints:

```python
# Set robot to a specific pose
target_positions = {
    world.connections[0]: np.pi/4,    # Base rotation: 45 degrees
    world.connections[1]: np.pi/6,    # Shoulder: 30 degrees  
    world.connections[2]: -np.pi/4,   # Elbow: -45 degrees
    world.connections[3]: 0.0,        # Wrist 1: 0 degrees
    world.connections[4]: np.pi/6,    # Wrist 2: 30 degrees
    world.connections[5]: 0.0,        # Wrist 3: 0 degrees
}

# Apply positions using the batch update method
world.set_positions_1DOF_connection(target_positions)

print("Robot moved to target configuration")

# Check individual joint positions
for i, connection in enumerate(world.connections[:6]):  # First 6 joints
    if hasattr(connection, 'position'):
        print(f"Joint {i+1}: {np.degrees(connection.position):.1f} degrees")
```

## Forward Kinematics

Compute the end-effector pose:

```python
# Find the gripper base (end-effector)
gripper_base = None
for body in world.bodies:
    if body.name == "gripper_base":
        gripper_base = body
        break

if gripper_base:
    # Compute forward kinematics from base to gripper
    end_effector_pose = world.compute_forward_kinematics(base, gripper_base)
    
    print("End-effector pose:")
    print("Position:", end_effector_pose[:3, 3])
    print("Orientation matrix:")
    print(end_effector_pose[:3, :3])
else:
    print("Gripper base not found")
```

## Gripper Control

Control the gripper opening/closing:

```python
# Open gripper
gripper_opening = 0.03  # 3cm opening

# Find gripper joints
finger_joints = []
for connection in world.connections:
    if hasattr(connection, 'dof') and 'finger' in connection.dof.name:
        finger_joints.append(connection)

if len(finger_joints) >= 2:
    # Set gripper positions
    finger_positions = {}
    finger_positions[finger_joints[0]] = gripper_opening / 2
    finger_positions[finger_joints[1]] = gripper_opening / 2
    
    world.set_positions_1DOF_connection(finger_positions)
    print(f"Gripper opened to {gripper_opening*1000:.1f}mm")
else:
    print("Gripper joints not found")
```

## World State Management

Access the complete robot state:

```python
# Get current state of all degrees of freedom
print("\nComplete robot state:")
for dof in world.degrees_of_freedom:
    state = world.state[dof.name]
    print(f"{dof.name}: {state.position:.3f} rad/m, velocity: {state.velocity:.3f}")

# Create a state snapshot
current_state = {}
for dof in world.degrees_of_freedom:
    current_state[dof.name] = world.state[dof.name].position

print(f"\nState snapshot contains {len(current_state)} DOF values")
```

## Summary

This notebook demonstrated:

1. **Proper World Construction** - Using `world.modify_world()` context manager
2. **Kinematic Chain Building** - Sequential body and joint creation
3. **Mixed Connection Types** - Revolute joints for arm, prismatic for gripper
4. **Batch Control** - Using `set_positions_1DOF_connection()` for efficiency
5. **Forward Kinematics** - Computing end-effector poses
6. **Validation** - Ensuring world structure integrity
7. **State Management** - Accessing and managing DOF states

The robot is now ready for motion planning, control, and simulation tasks!

```python
print(f"Final robot summary:")
print(f"- Bodies: {len(world.bodies)}")  
print(f"- Connections: {len(world.connections)}")
print(f"- Degrees of Freedom: {len(world.degrees_of_freedom)}")
print(f"- Validation: {'✓ Passed' if world.validate() else '✗ Failed'}")
```