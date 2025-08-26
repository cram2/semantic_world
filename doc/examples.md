# Practical Examples

This collection provides practical, real-world examples of using the Semantic World library for common robotics and simulation scenarios.

> **Note**: All examples assume the following common imports:
> ```python
> from semantic_world.world import World
> from semantic_world.world_entity import Body
> from semantic_world.connections import RevoluteConnection, FixedConnection, PrismaticConnection, Connection6DoF
> from semantic_world.degree_of_freedom import DegreeOfFreedom
> from semantic_world.geometry import Box, Cylinder, Sphere, Mesh
> from semantic_world.spatial_types.spatial_types import TransformationMatrix
> from semantic_world.spatial_types.math import rotation_matrix_from_rpy
> import numpy as np
> ```

## Example 1: Building a Robot Manipulator

This example shows how to construct a complete 6-DOF robot manipulator with gripper.

```python
from semantic_world.world import World
from semantic_world.world_entity import Body
from semantic_world.connections import RevoluteConnection, FixedConnection, PrismaticConnection
from semantic_world.degree_of_freedom import DegreeOfFreedom
from semantic_world.geometry import Box, Cylinder, Sphere
import numpy as np

def create_robot_manipulator():
    """Create a 6-DOF robot arm with gripper"""
    
    world = World()
    
    with world.modify_world():
        # Create robot base
        base = Body(
            name="robot_base",
            visual=[Cylinder(radius=0.2, height=0.1)],
            collision=[Cylinder(radius=0.2, height=0.1)]
        )
        world.add_body(base)
    
    # Define DH parameters for the robot
    # [theta, d, a, alpha] for each joint
    dh_params = [
        [0, 0.3, 0, np.pi/2],      # Joint 1: Base rotation
        [0, 0, 0.4, 0],            # Joint 2: Shoulder
        [0, 0, 0.4, 0],            # Joint 3: Elbow  
        [0, 0.3, 0, np.pi/2],      # Joint 4: Wrist 1
        [0, 0, 0, -np.pi/2],       # Joint 5: Wrist 2
        [0, 0.1, 0, 0]             # Joint 6: Wrist 3
    ]
    
    # Create links and joints
    bodies = [base]
    joints = []
    
    with world.modify_world():
        for i, (theta, d, a, alpha) in enumerate(dh_params):
            # Create link body
            if i < 2:  # First two links are larger
                link_visual = Box(size=[0.1, 0.1, 0.3])
            else:  # Smaller wrist links
                link_visual = Box(size=[0.08, 0.08, 0.2])
                
            link = Body(
                name=f"link_{i+1}",
                visual=[link_visual],
                collision=[link_visual]
            )
            world.add_body(link)
            bodies.append(link)
            
            # Create joint DOF
            joint_dof = DegreeOfFreedom(
                name=f"joint_{i+1}",
                lower_limit=-np.pi,
                upper_limit=np.pi,
                velocity_limit=2.0,
                acceleration_limit=5.0
            )
            
            # Create revolute joint
            joint = RevoluteConnection(
                parent=bodies[i],
                child=link,
                dof=joint_dof,
                axis=[0, 0, 1]  # All joints rotate around Z
            )
            
            # Set joint transform based on DH parameters
            transform = np.eye(4)
            # Simplified DH transform (modify as needed)
            transform[2, 3] = d  # Translation in Z
            transform[0, 3] = a  # Translation in X
            joint.origin_expression.matrix = transform
            
            world.add_connection(joint)
            joints.append(joint)
    
    # Add gripper
    gripper_base, gripper_joints = create_parallel_gripper(world, bodies[-1])
    
    return world, bodies, joints + gripper_joints

def create_parallel_gripper(world, parent_body):
    """Create a parallel jaw gripper"""
    
    with world.modify_world():
        # Gripper base
        gripper_base = Body(
            name="gripper_base",
            visual=[Box(size=[0.08, 0.06, 0.04])],
            collision=[Box(size=[0.08, 0.06, 0.04])]
        )
        world.add_body(gripper_base)
        
        # Mount gripper to parent
        gripper_mount = FixedConnection(parent=parent_body, child=gripper_base)
        world.add_connection(gripper_mount)
    
    # Shared DOF for synchronized finger movement
    grip_dof = DegreeOfFreedom(
        name="gripper_width",
        lower_limit=0.0,     # Closed
        upper_limit=0.08,    # 8cm max opening
        velocity_limit=0.1,  # Slow gripper movement
        acceleration_limit=0.5
    )
    
        # Left finger
        left_finger = Body(
            name="left_finger",
            visual=[Box(size=[0.02, 0.01, 0.06])],
            collision=[Box(size=[0.02, 0.01, 0.06])]
        )
        world.add_body(left_finger)
        
        left_joint = PrismaticConnection(
            parent=gripper_base,
            child=left_finger,
            dof=grip_dof,
            axis=[-1, 0, 0]  # Move in -X direction
        )
        world.add_connection(left_joint)
        
        # Right finger (mirror of left)
        right_finger = Body(
            name="right_finger", 
            visual=[Box(size=[0.02, 0.01, 0.06])],
            collision=[Box(size=[0.02, 0.01, 0.06])]
        )
        world.add_body(right_finger)
        
        right_joint = PrismaticConnection(
            parent=gripper_base,
            child=right_finger,
            dof=grip_dof,  # Same DOF for synchronized motion
            axis=[1, 0, 0]  # Move in +X direction
        )
        world.add_connection(right_joint)
        
        return gripper_base, [gripper_mount, left_joint, right_joint]

# Create and configure the robot
robot_world, robot_bodies, robot_joints = create_robot_manipulator()

# Set initial joint positions
initial_positions = {
    robot_joints[0]: 0.0,      # Base rotation
    robot_joints[1]: -np.pi/4, # Shoulder up
    robot_joints[2]: np.pi/2,  # Elbow bent
    robot_joints[3]: 0.0,      # Wrist 1
    robot_joints[4]: np.pi/4,  # Wrist 2
    robot_joints[5]: 0.0       # Wrist 3
}

robot_world.set_positions_1DOF_connection(initial_positions)

# Validate the complete robot
robot_world.validate()
print(f"Robot created with {len(robot_world.bodies)} bodies and {len(robot_world.connections)} connections")

# Test forward kinematics
end_effector = robot_bodies[-1]  # Last link (before gripper)
ee_pose = robot_world.compute_forward_kinematics(robot_world.root, end_effector)
print(f"End effector position: {ee_pose[:3, 3]}")

# Control gripper
gripper_joint = [j for j in robot_joints if "gripper" in j.dof.name][0]
gripper_joint.position = 0.04  # Open gripper to 4cm
print(f"Gripper opening: {gripper_joint.position*2*1000:.1f}mm")  # Both fingers move
```

## Example 2: Mobile Robot with Manipulator

Combining a mobile base with a manipulator arm:

```python
def create_mobile_manipulator():
    """Create a mobile robot with mounted manipulator"""
    
    world = World()
    
    with world.modify_world():
        # === Mobile Base ===
        
        # Main chassis
        chassis = Body(
            name="chassis",
            visual=[Box(size=[0.6, 0.4, 0.2])],
            collision=[Box(size=[0.6, 0.4, 0.2])]
        )
        world.add_body(chassis)
    
    # Create wheels
    wheel_radius = 0.1
    wheel_width = 0.05
    wheel_positions = [
        ("front_left", [0.25, 0.25, -0.1]),
        ("front_right", [0.25, -0.25, -0.1]),
        ("rear_left", [-0.25, 0.25, -0.1]),
        ("rear_right", [-0.25, -0.25, -0.1])
    ]
    
    wheels = []
    wheel_joints = []
    
    for wheel_name, position in wheel_positions:
        # Create wheel body
        wheel = Body(
            name=wheel_name,
            visual=[Cylinder(radius=wheel_radius, height=wheel_width)],
            collision=[Cylinder(radius=wheel_radius, height=wheel_width)]
        )
        world.add_body(wheel)
        wheels.append(wheel)
        
        # Create wheel joint (passive - spins freely)
        wheel_joint = RevoluteConnection(
            parent=chassis,
            child=wheel,
            dof=DegreeOfFreedom(name=f"{wheel_name}_spin", lower_limit=-np.inf, upper_limit=np.inf),
            axis=[0, 1, 0]  # Rotate around Y-axis
        )
        
        # Position wheel
        transform = np.eye(4)
        transform[:3, 3] = position
        wheel_joint.origin_expression.matrix = transform
        
        world.add_connection(wheel_joint)
        wheel_joints.append(wheel_joint)
    
    # === Manipulator Mount ===
    
    # Manipulator base mounted on chassis
    manipulator_mount = Body(
        name="manipulator_mount",
        visual=[Box(size=[0.2, 0.2, 0.1])],
        collision=[Box(size=[0.2, 0.2, 0.1])]
    )
    world.add_body(manipulator_mount)
    
    # Mount manipulator base to chassis
    mount_transform = np.eye(4)
    mount_transform[:3, 3] = [0.0, 0.0, 0.15]  # On top of chassis
    
    mount_connection = FixedConnection(
        parent=chassis,
        child=manipulator_mount,
        origin_expression=TransformationMatrix(mount_transform)
    )
    world.add_connection(mount_connection)
    
    # === Add 4-DOF Manipulator ===
    
    # Simplified 4-DOF arm for mobile robot
    arm_links = []
    arm_joints = []
    
    current_parent = manipulator_mount
    
    # Base rotation
    link1 = Body(name="arm_base", visual=[Cylinder(radius=0.06, height=0.1)])
    world.add_body(link1)
    arm_links.append(link1)
    
    joint1 = RevoluteConnection(
        parent=current_parent,
        child=link1,
        dof=DegreeOfFreedom(name="base_rotation", lower_limit=-np.pi, upper_limit=np.pi),
        axis=[0, 0, 1]
    )
    world.add_connection(joint1)
    arm_joints.append(joint1)
    current_parent = link1
    
    # Shoulder
    link2 = Body(name="upper_arm", visual=[Box(size=[0.05, 0.05, 0.3])])
    world.add_body(link2)
    arm_links.append(link2)
    
    joint2 = RevoluteConnection(
        parent=current_parent,
        child=link2,
        dof=DegreeOfFreedom(name="shoulder", lower_limit=-np.pi/2, upper_limit=np.pi/2),
        axis=[0, 1, 0]
    )
    world.add_connection(joint2)
    arm_joints.append(joint2)
    current_parent = link2
    
    # Elbow
    link3 = Body(name="forearm", visual=[Box(size=[0.04, 0.04, 0.25])])
    world.add_body(link3)
    arm_links.append(link3)
    
    joint3 = RevoluteConnection(
        parent=current_parent,
        child=link3,
        dof=DegreeOfFreedom(name="elbow", lower_limit=0, upper_limit=np.pi),
        axis=[0, 1, 0]
    )
    world.add_connection(joint3)
    arm_joints.append(joint3)
    current_parent = link3
    
    # Wrist
    link4 = Body(name="wrist", visual=[Box(size=[0.03, 0.03, 0.1])])
    world.add_body(link4)
    arm_links.append(link4)
    
    joint4 = RevoluteConnection(
        parent=current_parent,
        child=link4,
        dof=DegreeOfFreedom(name="wrist", lower_limit=-np.pi, upper_limit=np.pi),
        axis=[1, 0, 0]
    )
    world.add_connection(joint4)
    arm_joints.append(joint4)
    
        # Add simple gripper
        gripper, gripper_joints = create_parallel_gripper(world, link4)
    
    return world, {
        'chassis': chassis,
        'wheels': wheels,
        'wheel_joints': wheel_joints,
        'arm_links': arm_links,
        'arm_joints': arm_joints,
        'gripper_joints': gripper_joints
    }

# Create mobile manipulator
mobile_robot_world, components = create_mobile_manipulator()

# Configure initial pose
arm_config = {
    components['arm_joints'][0]: 0.0,      # Base rotation
    components['arm_joints'][1]: np.pi/6,  # Shoulder up
    components['arm_joints'][2]: np.pi/3,  # Elbow bent
    components['arm_joints'][3]: 0.0       # Wrist straight
}

mobile_robot_world.set_positions_1DOF_connection(arm_config)
mobile_robot_world.validate()

print(f"Mobile manipulator created:")
print(f"  - {len(components['wheels'])} wheels")
print(f"  - {len(components['arm_links'])} arm links")
print(f"  - Total bodies: {len(mobile_robot_world.bodies)}")
print(f"  - Total connections: {len(mobile_robot_world.connections)}")
```

## Example 3: Multi-Robot Environment

Creating an environment with multiple robots:

```python
def create_multi_robot_environment():
    """Create environment with multiple robots and objects"""
    
    world = World()
    
    # === Environment Setup ===
    
    # Create floor
    floor = Body(
        name="floor",
        visual=[Box(size=[10, 10, 0.1])],
        collision=[Box(size=[10, 10, 0.1])]
    )
    world.add_body(floor)
    
    # Create table
    table_top = Body(
        name="table_top",
        visual=[Box(size=[1.5, 1.0, 0.05])],
        collision=[Box(size=[1.5, 1.0, 0.05])]
    )
    world.add_body(table_top)
    
    # Position table above floor
    table_connection = FixedConnection(
        parent=floor,
        child=table_top,
        origin_expression=TransformationMatrix(np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0], 
            [0, 0, 1, 0.8],  # 80cm high table
            [0, 0, 0, 1]
        ]))
    )
    world.add_connection(table_connection)
    
    # === Robot 1: Stationary Manipulator ===
    
    robot1_world, _, robot1_joints = create_robot_manipulator()
    
    # Merge robot1 into environment
    robot1_base_pose = np.array([
        [1, 0, 0, 1.0],   # 1m from origin
        [0, 1, 0, -1.5],  # To the left
        [0, 0, 1, 0.05],  # On floor
        [0, 0, 0, 1]
    ])
    world.merge_with_pose(robot1_world, robot1_base_pose)
    
    # === Robot 2: Mobile Manipulator ===
    
    robot2_world, robot2_components = create_mobile_manipulator()
    
    # Position robot2 
    robot2_base_pose = np.array([
        [1, 0, 0, -1.0],  # 1m on other side
        [0, 1, 0, 1.5],   # To the right
        [0, 0, 1, 0.0],   # On floor (chassis bottom)
        [0, 0, 0, 1]
    ])
    world.merge_with_pose(robot2_world, robot2_base_pose)
    
    # === Objects on Table ===
    
    objects = []
    object_positions = [
        ("box1", [0.2, 0.2, 0.85], [0.1, 0.1, 0.1]),
        ("box2", [-0.2, 0.2, 0.85], [0.08, 0.15, 0.08]), 
        ("box3", [0.0, -0.2, 0.85], [0.12, 0.08, 0.15])
    ]
    
    for obj_name, position, size in object_positions:
        obj = Body(
            name=obj_name,
            visual=[Box(size=size)],
            collision=[Box(size=size)]
        )
        world.add_body(obj)
        objects.append(obj)
        
        # Place on table
        obj_transform = np.eye(4)
        obj_transform[:3, 3] = position
        
        obj_connection = FixedConnection(
            parent=floor,  # Actually placed relative to world/floor
            child=obj,
            origin_expression=TransformationMatrix(obj_transform)
        )
        world.add_connection(obj_connection)
    
    return world, {
        'floor': floor,
        'table': table_top,
        'objects': objects,
        'robot1_joints': robot1_joints,
        'robot2_components': robot2_components
    }

# Create multi-robot environment
env_world, env_components = create_multi_robot_environment()

# Configure robots for collaborative task
# Robot 1: Reach toward table
robot1_config = {
    env_components['robot1_joints'][0]: np.pi/4,   # Base toward table
    env_components['robot1_joints'][1]: -np.pi/6,  # Shoulder down
    env_components['robot1_joints'][2]: np.pi/3,   # Elbow up
}

# Robot 2: Extend arm
robot2_config = {
    env_components['robot2_components']['arm_joints'][0]: -np.pi/4,  # Base toward table
    env_components['robot2_components']['arm_joints'][1]: np.pi/4,   # Shoulder up
    env_components['robot2_components']['arm_joints'][2]: np.pi/4,   # Elbow extended
}

env_world.set_positions_1DOF_connection({**robot1_config, **robot2_config})
env_world.validate()

print(f"Multi-robot environment created:")
print(f"  - {len(env_components['objects'])} objects")
print(f"  - 2 robot systems")
print(f"  - Total bodies: {len(env_world.bodies)}")
print(f"  - Total DOFs: {len(env_world.degrees_of_freedom)}")

# Demonstrate forward kinematics for both robots
robot1_ee = None
robot2_ee = None

for body in env_world.bodies:
    if "link_6" in body.name.name:  # Robot 1 end effector
        robot1_ee = body
    elif "wrist" in body.name.name and "robot2" in str(body.name.prefix or ""):  # Robot 2 end effector  
        robot2_ee = body

if robot1_ee:
    r1_pose = robot1_ee.global_pose
    print(f"Robot 1 end effector: {r1_pose[:3, 3]}")

if robot2_ee:
    r2_pose = robot2_ee.global_pose  
    print(f"Robot 2 end effector: {r2_pose[:3, 3]}")
```

## Example 4: Simulation and Control Loop

Example showing how to use semantic worlds in a control loop:

```python
import time

def simulate_pick_and_place(world, robot_joints, target_object):
    """Simulate a pick and place operation"""
    
    print("Starting pick and place simulation...")
    
    # Phase 1: Move to approach pose
    print("Phase 1: Approaching object")
    approach_config = {
        robot_joints[0]: np.pi/6,    # Rotate base toward object
        robot_joints[1]: -np.pi/4,   # Lower shoulder  
        robot_joints[2]: np.pi/2,    # Bend elbow
        robot_joints[3]: 0.0,        # Wrist neutral
    }
    
    # Interpolate to target position
    initial_positions = {joint: joint.position for joint in approach_config.keys()}
    
    steps = 20
    for step in range(steps):
        alpha = step / (steps - 1)  # 0 to 1
        
        interpolated_config = {}
        for joint in approach_config.keys():
            start_pos = initial_positions[joint]
            end_pos = approach_config[joint]
            current_pos = start_pos + alpha * (end_pos - start_pos)
            interpolated_config[joint] = current_pos
        
        world.set_positions_1DOF_connection(interpolated_config)
        
        # Get current end effector pose
        end_effector = get_end_effector(world)
        ee_pose = end_effector.global_pose
        
        print(f"Step {step+1}/{steps}: EE at {ee_pose[:3, 3]}")
        time.sleep(0.1)  # Simulate real-time
    
    # Phase 2: Close gripper
    print("Phase 2: Grasping object")
    gripper_joint = get_gripper_joint(world)
    if gripper_joint:
        gripper_joint.position = 0.02  # Close to 2cm
        print(f"Gripper closed to {gripper_joint.position*1000}mm")
    
    # Phase 3: Lift object
    print("Phase 3: Lifting object")
    lift_config = {
        robot_joints[1]: -np.pi/6,   # Lift shoulder
        robot_joints[2]: np.pi/3,    # Adjust elbow
    }
    world.set_positions_1DOF_connection(lift_config)
    
    # Phase 4: Move to drop location
    print("Phase 4: Moving to drop location")
    drop_config = {
        robot_joints[0]: -np.pi/6,   # Rotate to drop location
        robot_joints[1]: -np.pi/4,   # Lower for drop
    }
    world.set_positions_1DOF_connection(drop_config)
    
    # Phase 5: Release object
    print("Phase 5: Releasing object")
    if gripper_joint:
        gripper_joint.position = 0.06  # Open gripper
        print(f"Gripper opened to {gripper_joint.position*1000}mm")
    
    print("Pick and place completed!")

def get_end_effector(world):
    """Find the end effector body"""
    for body in world.bodies:
        if "link_6" in body.name.name or "wrist" in body.name.name:
            return body
    return None

def get_gripper_joint(world):
    """Find the gripper joint"""
    for connection in world.connections:
        if hasattr(connection, 'dof') and "gripper" in connection.dof.name:
            return connection
    return None

# Run simulation with one of our robots
robot_world, robot_bodies, robot_joints = create_robot_manipulator()

# Create a target object
target = Body(name="target_object", visual=[Box(size=[0.05, 0.05, 0.05])])
robot_world.add_body(target)

# Position target within reach
target_connection = FixedConnection(
    parent=robot_world.root,
    child=target,
    origin_expression=TransformationMatrix(np.array([
        [1, 0, 0, 0.5],   # 50cm in front
        [0, 1, 0, 0.2],   # 20cm to side
        [0, 0, 1, 0.5],   # 50cm high
        [0, 0, 0, 1]
    ]))
)
robot_world.add_connection(target_connection)

# Run the simulation
simulate_pick_and_place(robot_world, robot_joints[:4], target)  # Use first 4 joints
```

## Example 5: Working with Views and Semantic Interpretation

Using semantic views to add meaning to kinematic structures:

```python
from semantic_world.views.views import Table, Drawer, Door

def create_kitchen_environment():
    """Create a kitchen with semantic objects"""
    
    world = World()
    
    # === Kitchen Counter ===
    counter_top = Body(name="counter_top", visual=[Box(size=[2.0, 0.6, 0.05])])
    counter_base = Body(name="counter_base", visual=[Box(size=[1.9, 0.55, 0.8])])
    
    world.add_body(counter_top)
    world.add_body(counter_base)
    
    counter_connection = FixedConnection(parent=counter_base, child=counter_top)
    world.add_connection(counter_connection)
    
    # Create table view for counter
    counter_table = Table(
        bodies=[counter_top, counter_base],
        name="kitchen_counter"
    )
    world.add_view(counter_table)
    
    # === Kitchen Drawer ===
    drawer_frame = Body(name="drawer_frame", visual=[Box(size=[0.4, 0.5, 0.2])])
    drawer_box = Body(name="drawer_box", visual=[Box(size=[0.35, 0.45, 0.18])])
    drawer_handle = Body(name="drawer_handle", visual=[Cylinder(radius=0.01, height=0.1)])
    
    world.add_body(drawer_frame)
    world.add_body(drawer_box)
    world.add_body(drawer_handle)
    
    # Drawer slides in/out
    drawer_dof = DegreeOfFreedom(
        name="drawer_slide",
        lower_limit=0.0,    # Closed
        upper_limit=0.4,    # Fully open
        velocity_limit=0.2
    )
    
    drawer_slide = PrismaticConnection(
        parent=drawer_frame,
        child=drawer_box,
        dof=drawer_dof,
        axis=[1, 0, 0]  # Slide out in X direction
    )
    world.add_connection(drawer_slide)
    
    # Handle attached to drawer box
    handle_connection = FixedConnection(parent=drawer_box, child=drawer_handle)
    world.add_connection(handle_connection)
    
    # Create drawer view
    kitchen_drawer = Drawer(
        bodies=[drawer_frame, drawer_box, drawer_handle],
        name="kitchen_drawer",
        handle=drawer_handle,
        container=drawer_box
    )
    world.add_view(kitchen_drawer)
    
    # === Cabinet Door ===
    door_frame = Body(name="door_frame", visual=[Box(size=[0.5, 0.05, 0.8])])
    door_panel = Body(name="door_panel", visual=[Box(size=[0.48, 0.03, 0.78])])
    door_handle = Body(name="door_handle", visual=[Sphere(radius=0.02)])
    
    world.add_body(door_frame)
    world.add_body(door_panel)
    world.add_body(door_handle)
    
    # Door hinges
    door_dof = DegreeOfFreedom(
        name="door_hinge",
        lower_limit=0.0,       # Closed
        upper_limit=np.pi/2,   # 90 degrees open
        velocity_limit=1.0
    )
    
    door_hinge = RevoluteConnection(
        parent=door_frame,
        child=door_panel,
        dof=door_dof,
        axis=[0, 0, 1]  # Rotate around Z (vertical)
    )
    world.add_connection(door_hinge)
    
    # Handle on door
    handle_mount = FixedConnection(parent=door_panel, child=door_handle)
    world.add_connection(handle_mount)
    
    # Create door view
    cabinet_door = Door(
        bodies=[door_frame, door_panel, door_handle],
        name="cabinet_door",
        handle=door_handle
    )
    world.add_view(cabinet_door)
    
    return world, {
        'counter': counter_table,
        'drawer': kitchen_drawer,
        'door': cabinet_door
    }

# Create kitchen and demonstrate semantic queries
kitchen_world, kitchen_objects = create_kitchen_environment()

print("Kitchen environment created:")
print(f"Views: {[view.name for view in kitchen_world.views]}")

# Demonstrate using views
counter = kitchen_objects['counter']
print(f"Counter has {len(counter.bodies)} bodies")

drawer = kitchen_objects['drawer']
print(f"Drawer handle: {drawer.handle.name if hasattr(drawer, 'handle') else 'None'}")

# Control kitchen objects
drawer_joint = None
door_joint = None

for connection in kitchen_world.connections:
    if hasattr(connection, 'dof'):
        if "drawer" in connection.dof.name:
            drawer_joint = connection
        elif "door" in connection.dof.name:
            door_joint = connection

# Open drawer halfway
if drawer_joint:
    drawer_joint.position = 0.2  # 20cm out
    print(f"Drawer opened {drawer_joint.position*100}cm")

# Open door 45 degrees
if door_joint:
    door_joint.position = np.pi/4  # 45 degrees
    print(f"Door opened {np.degrees(door_joint.position):.1f} degrees")

kitchen_world.validate()
```

These examples demonstrate:

1. **Robot Construction**: Building complete robot systems with proper kinematic structures
2. **Mobile Systems**: Combining mobility with manipulation capabilities
3. **Multi-Robot Environments**: Creating complex environments with multiple agents
4. **Simulation Control**: Using worlds in control loops and simulations
5. **Semantic Interpretation**: Adding meaning to kinematic structures with views

Each example builds on the concepts from the previous guides and shows practical applications of the Semantic World library. The key takeaways are:

- **Start Simple**: Build incrementally and validate frequently
- **Reuse Components**: Create factory functions for common structures
- **Plan Hierarchies**: Design parent-child relationships carefully
- **Add Semantics**: Use views to give meaning to pure kinematic structures
- **Validate Often**: Check world consistency at each step

For more details on specific aspects, refer to:
- [Getting Started Guide](getting_started.md)
- [World Construction Guide](world_construction.md)  
- [Body Management Guide](body_management.md)
- [Connection Types Guide](connection_types.md)