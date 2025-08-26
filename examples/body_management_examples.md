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

# Body Management Examples

This notebook demonstrates comprehensive body creation, configuration, and management in the Semantic World library.

```python
import numpy as np
from semantic_world.world import World
from semantic_world.world_entity import Body
from semantic_world.geometry import Box, Sphere, Cylinder, Mesh
from semantic_world.spatial_types.spatial_types import TransformationMatrix
from semantic_world.prefixed_name import PrefixedName
```

## Basic Body Creation

```python
# Create a world for our examples
world = World()

# Basic body creation
simple_body = Body(name="simple_box")
print(f"Created body: {simple_body.name}")

# Body with automatic naming
auto_named = Body()
print(f"Auto-named body: {auto_named.name}")

# Body with prefixed name for organization  
prefixed_body = Body(name=PrefixedName("arm_link", prefix="robot1"))
print(f"Prefixed body: {prefixed_body.name}")
```

## Bodies with Visual Geometry

```python
with world.modify_world():
    # Body with single visual shape
    box_body = Body(
        name="red_box",
        visual=[Box(size=[1.0, 0.5, 0.2])]
    )
    world.add_body(box_body)
    
    # Body with multiple visual shapes
    complex_visual = Body(
        name="complex_visual",
        visual=[
            Box(size=[0.5, 0.5, 0.1]),
            Sphere(radius=0.2),
            Cylinder(radius=0.1, height=0.8)
        ]
    )
    world.add_body(complex_visual)
    
    print(f"Box body has {len(box_body.visual)} visual shape(s)")
    print(f"Complex body has {len(complex_visual.visual)} visual shapes")
```

## Bodies with Collision Geometry

```python
with world.modify_world():
    # Body with separate visual and collision geometry
    detailed_body = Body(
        name="detailed_object",
        visual=[Box(size=[1.0, 0.8, 0.3])],  # Detailed visual
        collision=[Sphere(radius=0.6)]        # Simplified collision
    )
    world.add_body(detailed_body)
    
    # Body optimized for collision detection
    collision_optimized = Body(
        name="collision_object",
        visual=[Mesh(filename="complex_model.obj")],  # Complex visual mesh
        collision=[Box(size=[0.5, 0.5, 0.5])]        # Simple collision box
    )
    world.add_body(collision_optimized)
    
    print(f"Detailed body - Visual: {len(detailed_body.visual)}, Collision: {len(detailed_body.collision)}")
    print(f"Collision optimized - Visual: {len(collision_optimized.visual)}, Collision: {len(collision_optimized.collision)}")
```

## Geometry with Custom Poses

```python
with world.modify_world():
    # Create transformation matrices for geometry positioning
    offset_transform = np.eye(4)
    offset_transform[:3, 3] = [0.2, 0.0, 0.5]  # Offset in x and z
    
    rotation_transform = np.eye(4)
    rotation_transform[:3, :3] = np.array([
        [0, -1, 0],  # Rotate 90 degrees around Z
        [1, 0, 0],
        [0, 0, 1]
    ])
    
    # Body with positioned geometry
    positioned_body = Body(name="positioned_geometry")
    
    # Add geometry with custom poses
    main_box = Box(size=[0.8, 0.3, 0.2])
    offset_cylinder = Cylinder(radius=0.1, height=0.4)
    offset_cylinder.origin = TransformationMatrix(offset_transform)
    
    rotated_box = Box(size=[0.2, 0.6, 0.1])  
    rotated_box.origin = TransformationMatrix(rotation_transform)
    
    positioned_body.visual = [main_box, offset_cylinder, rotated_box]
    world.add_body(positioned_body)
    
    print(f"Positioned body has {len(positioned_body.visual)} positioned shapes")
```

## Complex Body Assemblies

```python
# Helper function to create a table
def create_table(name_prefix="table"):
    with world.modify_world():
        # Table top
        table_top = Body(
            name=PrefixedName("top", prefix=name_prefix),
            visual=[Box(size=[1.2, 0.8, 0.05])],
            collision=[Box(size=[1.2, 0.8, 0.05])]
        )
        world.add_body(table_top)
        
        # Table legs
        leg_positions = [
            [0.5, 0.35, -0.375],   # Front right
            [-0.5, 0.35, -0.375],  # Front left
            [0.5, -0.35, -0.375],  # Back right
            [-0.5, -0.35, -0.375]  # Back left
        ]
        
        legs = []
        for i, pos in enumerate(leg_positions):
            leg = Body(
                name=PrefixedName(f"leg_{i+1}", prefix=name_prefix),
                visual=[Cylinder(radius=0.02, height=0.75)],
                collision=[Cylinder(radius=0.02, height=0.75)]
            )
            
            # Position the leg geometry
            leg_transform = np.eye(4)
            leg_transform[:3, 3] = pos
            leg.visual[0].origin = TransformationMatrix(leg_transform)
            leg.collision[0].origin = TransformationMatrix(leg_transform)
            
            world.add_body(leg)
            legs.append(leg)
            
        return table_top, legs

# Create table assembly
table_top, table_legs = create_table("dining_table")
print(f"Created table with top and {len(table_legs)} legs")
```

## Body Properties and Attributes

```python
# Examine body properties
sample_body = world.bodies[0]  # Get first body in world

print("Body Properties:")
print(f"Name: {sample_body.name}")
print(f"Visual shapes: {len(sample_body.visual) if sample_body.visual else 0}")
print(f"Collision shapes: {len(sample_body.collision) if sample_body.collision else 0}")
print(f"Index: {sample_body.index}")

# Check if body has geometry
if sample_body.visual:
    print("\nVisual geometry details:")
    for i, shape in enumerate(sample_body.visual):
        print(f"  Shape {i+1}: {type(shape).__name__}")
        if hasattr(shape, 'size'):
            print(f"    Size: {shape.size}")
        elif hasattr(shape, 'radius'):
            print(f"    Radius: {shape.radius}")
            if hasattr(shape, 'height'):
                print(f"    Height: {shape.height}")
```

## Body Validation and Error Handling

```python
# Create bodies for validation testing
with world.modify_world():
    # Valid body
    valid_body = Body(
        name="valid_test_body",
        visual=[Box(size=[0.1, 0.1, 0.1])],
        collision=[Box(size=[0.1, 0.1, 0.1])]
    )
    world.add_body(valid_body)
    
    # Body with no geometry (still valid)
    no_geometry_body = Body(name="abstract_body")
    world.add_body(no_geometry_body)

# Validate world with all bodies
try:
    world.validate()
    print("✓ All bodies passed validation")
except Exception as e:
    print(f"✗ Validation error: {e}")

# Test duplicate name handling
try:
    with world.modify_world():
        duplicate_body = Body(name="valid_test_body")  # Same name as existing
        world.add_body(duplicate_body)
except Exception as e:
    print(f"Duplicate name handling: {e}")
```

## Body Relationships and Queries

```python
# Find bodies by name pattern
def find_bodies_by_pattern(world, pattern):
    matching_bodies = []
    for body in world.bodies:
        if pattern in str(body.name):
            matching_bodies.append(body)
    return matching_bodies

# Find all table-related bodies
table_bodies = find_bodies_by_pattern(world, "table")
print(f"Found {len(table_bodies)} table-related bodies:")
for body in table_bodies:
    print(f"  - {body.name}")

# Count bodies by type (based on name patterns)
body_types = {}
for body in world.bodies:
    body_name = str(body.name)
    if "table" in body_name:
        category = "furniture"
    elif "robot" in body_name or "link" in body_name:
        category = "robot"
    elif "box" in body_name:
        category = "primitive"
    else:
        category = "other"
    
    body_types[category] = body_types.get(category, 0) + 1

print("\nBody distribution by category:")
for category, count in body_types.items():
    print(f"  {category}: {count} bodies")
```

## Memory and Performance Considerations

```python
# Analyze geometry complexity
def analyze_geometry_complexity(body):
    total_shapes = 0
    complex_shapes = 0
    
    for shape_list in [body.visual, body.collision]:
        if shape_list:
            total_shapes += len(shape_list)
            for shape in shape_list:
                if isinstance(shape, Mesh):
                    complex_shapes += 1
    
    return total_shapes, complex_shapes

# Analyze all bodies
total_bodies = len(world.bodies)
total_shapes = 0
complex_shapes = 0

for body in world.bodies:
    shapes, complex = analyze_geometry_complexity(body)
    total_shapes += shapes
    complex_shapes += complex

print(f"\nWorld Complexity Analysis:")
print(f"Total bodies: {total_bodies}")
print(f"Total geometry shapes: {total_shapes}")
print(f"Complex shapes (meshes): {complex_shapes}")
print(f"Average shapes per body: {total_shapes/total_bodies:.1f}")
```

## Body Modification and Updates

```python
# Modify existing body properties
with world.modify_world():
    # Find a body to modify
    target_body = None
    for body in world.bodies:
        if "simple" in str(body.name):
            target_body = body
            break
    
    if target_body:
        print(f"Original body '{target_body.name}' had {len(target_body.visual) if target_body.visual else 0} visual shapes")
        
        # Add geometry to existing body
        if not target_body.visual:
            target_body.visual = []
        
        target_body.visual.append(Sphere(radius=0.3))
        target_body.collision = [Sphere(radius=0.3)]
        
        print(f"Modified body now has {len(target_body.visual)} visual shapes and {len(target_body.collision)} collision shapes")
```

## Best Practices Summary

```python
print("\n=== Body Management Best Practices ===")
print("✓ Use modify_world() context manager for all world modifications")
print("✓ Always validate the world after adding bodies")
print("✓ Use prefixed names for organization in complex worlds")
print("✓ Separate visual and collision geometry for performance")
print("✓ Position geometry relative to body frame using TransformationMatrix")
print("✓ Use simple shapes for collision when possible")
print("✓ Handle duplicate names and validation errors appropriately")

print(f"\nFinal world state: {len(world.bodies)} bodies successfully created and managed")
```