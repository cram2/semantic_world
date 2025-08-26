# Troubleshooting Guide

This guide covers common issues you might encounter when working with the Semantic World library and how to resolve them.

## Installation Issues

### Import Errors

**Problem**: `ImportError: No module named 'semantic_world'`

**Solution**:
```bash
pip install semantic_world
# or for development:
pip install -e .
```

**Problem**: `ImportError: No module named 'rclpy'` or similar ROS dependencies

**Solution**: Some examples use ROS2. Install ROS2 or modify examples to remove ROS dependencies:
```python
# Remove or comment out ROS-related lines:
# import rclpy
# rclpy.init()
# rclpy.shutdown()
```

## World Construction Issues

### Validation Errors

**Problem**: `World validation failed` with cycle detection errors

**Solution**: Check for circular dependencies in your kinematic structure:
```python
# Problematic: body1 -> body2 -> body1 (cycle)
# Fix: Ensure tree structure with single root

# Debug cycles:
try:
    world.validate()
except Exception as e:
    print(f"Validation error: {e}")
    
    # Check each connection
    for conn in world.connections:
        print(f"{conn.parent.name} -> {conn.child.name}")
```

**Problem**: Bodies not connected to world root

**Solution**: Ensure all bodies have a path to the root:
```python
# Check connectivity
for body in world.bodies:
    try:
        path = world.compute_chain_of_bodies(world.root, body)
        print(f"Path to {body.name}: {[b.name for b in path]}")
    except:
        print(f"Body {body.name} is not reachable from root!")
```

### Connection Issues

**Problem**: `ValueError: PrismaticConnection cannot be created without a world if the dof is not provided`

**Solution**: Either add the connection to a world first, or provide the DOF explicitly:
```python
# Method 1: Add to world first
world.add_body(parent)
world.add_body(child)
connection = PrismaticConnection(parent=parent, child=child, axis=[1,0,0])
world.add_connection(connection)

# Method 2: Provide DOF explicitly
dof = DegreeOfFreedom(name="joint1")
connection = PrismaticConnection(parent=parent, child=child, dof=dof, axis=[1,0,0])
```

**Problem**: Joint limits violated

**Solution**: Check and respect DOF limits:
```python
joint = RevoluteConnection(...)

# Check limits before setting
if joint.dof.lower_limit <= angle <= joint.dof.upper_limit:
    joint.position = angle
else:
    print(f"Angle {angle} outside limits [{joint.dof.lower_limit}, {joint.dof.upper_limit}]")
    joint.position = np.clip(angle, joint.dof.lower_limit, joint.dof.upper_limit)
```

## State Management Issues

### State Update Problems

**Problem**: Joint positions not updating properly

**Solution**: Use the correct method for state updates:
```python
# Correct way:
world.set_positions_1DOF_connection({
    joint1: position1,
    joint2: position2
})

# Or individual updates:
joint1.position = position1  # This also works

# Incorrect - don't modify DOF directly:
# joint1.dof.position = position1  # May not trigger updates
```

**Problem**: Forward kinematics returning unexpected results

**Solution**: Check cache invalidation and coordinate frames:
```python
# Clear caches after major changes
world.reset_cache()

# Verify transforms step by step
parent_pose = parent_body.global_pose
child_pose = child_body.global_pose
relative_transform = world.compute_forward_kinematics(parent_body, child_body)

print(f"Parent global pose:\n{parent_pose}")
print(f"Child global pose:\n{child_pose}")
print(f"Relative transform:\n{relative_transform}")
```

## Geometry Issues

### Shape Creation Problems

**Problem**: Geometry not appearing or incorrect size

**Solution**: Check shape parameters and coordinate frames:
```python
# Ensure positive dimensions
box = Box(size=[1.0, 1.0, 1.0])  # All positive

# Check geometry origin
for shape in body.visual:
    print(f"Shape origin:\n{shape.origin.matrix}")
    
    # Reset origin if needed
    shape.origin = TransformationMatrix(np.eye(4))
```

**Problem**: Mesh loading fails

**Solution**: Verify file paths and formats:
```python
import os
from semantic_world.geometry import Mesh

mesh_path = "path/to/mesh.stl"
if not os.path.exists(mesh_path):
    print(f"Mesh file not found: {mesh_path}")
else:
    mesh = Mesh(filename=mesh_path)
    print(f"Mesh loaded: {mesh_path}")
```

## Performance Issues

### Slow Forward Kinematics

**Problem**: Forward kinematics computation is slow

**Solution**: Check caching and avoid unnecessary recomputation:
```python
# Good - computed once, then cached
transform1 = world.compute_forward_kinematics(root, tip)
transform2 = world.compute_forward_kinematics(root, tip)  # Uses cache

# Avoid clearing cache unnecessarily
# world.reset_cache()  # Only when needed

# For repetitive computations, consider pre-compilation
compiled_fk = world.compile_forward_kinematics_expression(root, tip)
```

### Memory Usage

**Problem**: High memory usage with complex worlds

**Solution**: Optimize geometry and state management:
```python
# Use simpler collision geometry than visual
body = Body(
    name="body",
    visual=[detailed_mesh],      # Complex visual
    collision=[simple_box]       # Simple collision
)

# Clean up unused objects
del unused_world
import gc
gc.collect()
```

## Database Issues

### ORM Serialization Problems

**Problem**: `to_dao()` fails with complex worlds

**Solution**: Check object relationships and ORM generation:
```python
try:
    world_dao = to_dao(world)
    session.add(world_dao)
    session.commit()
except Exception as e:
    print(f"Serialization failed: {e}")
    
    # Check world structure
    print(f"Bodies: {len(world.bodies)}")
    print(f"Connections: {len(world.connections)}")
    print(f"Views: {len(world.views)}")
    
    # Try serializing components individually
    for body in world.bodies:
        try:
            body_dao = to_dao(body)
            print(f"Body {body.name} OK")
        except Exception as be:
            print(f"Body {body.name} failed: {be}")
```

**Problem**: Database schema issues

**Solution**: Regenerate ORM interface:
```bash
cd semantic_world
python scripts/generate_orm.py
```

## Views and Reasoning Issues

### View Creation Problems

**Problem**: Views not recognized or invalid

**Solution**: Check view definitions and body relationships:
```python
from semantic_world.views.views import Table

# Ensure bodies exist and are connected properly
table_bodies = [table_top, leg1, leg2, leg3, leg4]
for body in table_bodies:
    if body not in world.bodies:
        print(f"Body {body.name} not in world!")
        world.add_body(body)

# Create view
table = Table(bodies=table_bodies, name="my_table")
world.add_view(table)

# Verify view
print(f"View bodies: {[b.name for b in table.bodies]}")
```

### World Reasoner Issues

**Problem**: Reasoner not classifying correctly

**Solution**: Check rule definitions and world structure:
```python
from semantic_world.reasoner import WorldReasoner

reasoner = WorldReasoner(world)

# Check what the reasoner sees
print("Available attributes:")
for attr in reasoner.available_attributes:
    print(f"  {attr}")

# Debug classification
results = reasoner.classify_attribute("views")
print(f"Classification results: {results}")
```

## Common Debugging Techniques

### Print World Structure

```python
def debug_world_structure(world):
    """Print detailed world structure for debugging"""
    print(f"=== World Structure Debug ===")
    print(f"Bodies ({len(world.bodies)}):")
    for i, body in enumerate(world.bodies):
        print(f"  {i}: {body.name} (index: {body.index})")
        print(f"      Visual: {len(body.visual)} shapes")
        print(f"      Collision: {len(body.collision)} shapes")
    
    print(f"\nConnections ({len(world.connections)}):")
    for i, conn in enumerate(world.connections):
        print(f"  {i}: {conn.parent.name} -> {conn.child.name}")
        print(f"      Type: {type(conn).__name__}")
        if hasattr(conn, 'dof') and conn.dof:
            print(f"      DOF: {conn.dof.name} [{conn.dof.lower_limit}, {conn.dof.upper_limit}]")
    
    print(f"\nDegrees of Freedom ({len(world.degrees_of_freedom)}):")
    for dof in world.degrees_of_freedom:
        state = world.state[dof.name] if dof.name in world.state else None
        pos = state.position if state else "N/A"
        print(f"  {dof.name}: position={pos}")
    
    print(f"\nViews ({len(world.views)}):")
    for view in world.views:
        print(f"  {view.name}: {type(view).__name__} ({len(view.bodies)} bodies)")

# Usage
debug_world_structure(world)
```

### Check Transform Consistency

```python
def verify_transform_consistency(world):
    """Verify that transforms are consistent"""
    print("=== Transform Consistency Check ===")
    
    for connection in world.connections:
        parent_global = connection.parent.global_pose
        child_global = connection.child.global_pose
        relative = connection.origin
        
        # Expected child pose from parent + relative
        expected_child = parent_global @ relative.matrix
        
        # Check if they match
        diff = np.linalg.norm(expected_child - child_global)
        
        if diff > 1e-10:  # Small tolerance for numerical errors
            print(f"MISMATCH in {connection.parent.name} -> {connection.child.name}")
            print(f"  Difference: {diff}")
            print(f"  Expected:\n{expected_child}")
            print(f"  Actual:\n{child_global}")
        else:
            print(f"OK: {connection.parent.name} -> {connection.child.name}")

# Usage  
verify_transform_consistency(world)
```

### Validate Individual Components

```python
def validate_components_individually(world):
    """Validate world components one by one"""
    
    # Check bodies
    print("Validating bodies...")
    for body in world.bodies:
        if not body.name or not body.name.name:
            print(f"  ERROR: Body has invalid name: {body.name}")
        if body.index is None and body in world.kinematic_structure:
            print(f"  ERROR: Body {body.name} in structure but no index")
    
    # Check connections
    print("Validating connections...")
    for conn in world.connections:
        if conn.parent not in world.bodies:
            print(f"  ERROR: Connection parent {conn.parent.name} not in world")
        if conn.child not in world.bodies:
            print(f"  ERROR: Connection child {conn.child.name} not in world")
        if conn.parent == conn.child:
            print(f"  ERROR: Self-connection: {conn.parent.name}")
    
    # Check DOFs
    print("Validating DOFs...")
    for dof in world.degrees_of_freedom:
        if dof.lower_limit >= dof.upper_limit:
            print(f"  ERROR: Invalid limits for {dof.name}: [{dof.lower_limit}, {dof.upper_limit}]")

# Usage
validate_components_individually(world)
```

## Getting Help

If you encounter issues not covered here:

1. **Check the examples** in the documentation for similar usage patterns
2. **Validate your world structure** using the debugging functions above  
3. **Look at the source code** - it's well-documented and can clarify expected behavior
4. **Create minimal test cases** to isolate the problem
5. **Check the GitHub issues** for known problems and solutions

Remember: Most issues stem from incorrect world structure, missing connections, or coordinate frame misunderstandings. The debugging functions above can help identify these problems quickly.