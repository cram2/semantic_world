# Body Management Guide

Bodies are the fundamental building blocks of semantic worlds, representing physical entities that cannot be decomposed into smaller meaningful parts. This guide covers creating, configuring, and managing bodies in detail.

## Body Basics

A `Body` represents a semantic atom in your world - it's the smallest meaningful physical unit:

```python
from semantic_world.world_entity import Body
from semantic_world.prefixed_name import PrefixedName

# Create a simple body
simple_body = Body(name="my_body")

# Bodies are automatically assigned names if not provided
auto_named_body = Body()  # Gets name like "Body_12345"

# Use prefixed names for organization
prefixed_body = Body(name=PrefixedName("arm_link", prefix="robot1"))
print(prefixed_body.name)  # "robot1/arm_link"
```

## Visual and Collision Geometry

Bodies can have separate visual and collision geometry:

```python
from semantic_world.geometry import Box, Sphere, Cylinder, Mesh
from semantic_world.spatial_types.spatial_types import TransformationMatrix
import numpy as np

# Create geometry shapes
visual_box = Box(size=[1.0, 0.5, 0.2])
collision_sphere = Sphere(radius=0.6)
collision_cylinder = Cylinder(radius=0.3, height=1.0)

# Create body with multiple visual and collision shapes
complex_body = Body(
    name="complex_body",
    visual=[visual_box],
    collision=[collision_sphere, collision_cylinder]
)

# Set geometry poses relative to body frame
offset_transform = np.eye(4)
offset_transform[2, 3] = 0.5  # 0.5m offset in z-direction
visual_box.origin = TransformationMatrix(offset_transform)

print(f"Body has {len(complex_body.visual)} visual shapes")
print(f"Body has {len(complex_body.collision)} collision shapes")
```

## Geometry Types

### Primitive Shapes

```python
# Box geometry
box = Box(size=[length, width, height])

# Sphere geometry  
sphere = Sphere(radius=0.5)

# Cylinder geometry
cylinder = Cylinder(radius=0.2, height=1.0)

# All primitives can have custom poses
box_transform = np.eye(4)
box_transform[:3, 3] = [0.1, 0.2, 0.3]  # Translation
box.origin = TransformationMatrix(box_transform)
```

### Mesh Geometry

```python
# Load mesh from file
mesh_shape = Mesh(
    filename="path/to/mesh.stl",
    scale=[1.0, 1.0, 1.0]
)

# Create body with mesh
mesh_body = Body(
    name="mesh_body",
    visual=[mesh_shape],
    collision=[mesh_shape]  # Can reuse same mesh for both
)
```

## Body Properties and State

### Accessing Body Properties

```python
body = Body(name="test_body")

# Basic properties
print(f"Name: {body.name}")
print(f"Hash: {hash(body)}")
print(f"Index in world: {body.index}")  # None until added to world

# Geometry access
for i, shape in enumerate(body.visual):
    print(f"Visual shape {i}: {type(shape).__name__}")

for i, shape in enumerate(body.collision):
    print(f"Collision shape {i}: {type(shape).__name__}")
```

### Global Pose

Once a body is in a world, you can get its global pose:

```python
from semantic_world.world import World
from semantic_world.connections import FixedConnection

world = World()
parent = Body(name="parent")
child = Body(name="child") 

with world.modify_world():
    world.add_body(parent)
    world.add_body(child)

    # Create connection with offset
    connection = FixedConnection(parent=parent, child=child)
    transform = np.eye(4)
    transform[:3, 3] = [1, 2, 3]  # Offset position
    connection.origin_expression.matrix = transform
    world.add_connection(connection)

# Get global poses
parent_pose = parent.global_pose
child_pose = child.global_pose

print(f"Parent global pose:\n{parent_pose}")
print(f"Child global pose:\n{child_pose}")
print(f"Child is offset by: {child_pose[:3, 3] - parent_pose[:3, 3]}")
```

## Working with Body Collections

### Finding Bodies

```python
# Find bodies by name
body = world.get_body_by_name("my_body")

# Find bodies matching pattern
arm_bodies = world.get_bodies_by_name_pattern("arm_*")
left_bodies = world.get_bodies_by_name_pattern("left_*")

# Get all bodies
all_bodies = world.bodies
print(f"World has {len(all_bodies)} bodies")

# Filter bodies by type or property
robot_bodies = [b for b in world.bodies if "robot" in b.name.name]
bodies_with_collision = [b for b in world.bodies if len(b.collision) > 0]
```

### Body Hierarchies

```python
# Build a kinematic chain
def create_body_chain(world, names, parent=None):
    """Create a chain of bodies connected by fixed connections"""
    bodies = []
    
    with world.modify_world():
        for name in names:
            body = Body(name=name)
            world.add_body(body)
            bodies.append(body)
            
            if parent is not None:
                connection = FixedConnection(parent=parent, child=body)
                world.add_connection(connection)
            
            parent = body  # This body becomes parent of next
    
    return bodies

# Create chain: base -> link1 -> link2 -> link3
chain_names = ["base", "link1", "link2", "link3"]
chain_bodies = create_body_chain(world, chain_names)

# Find paths between bodies
path = world.compute_chain_of_bodies(root=chain_bodies[0], tip=chain_bodies[-1])
path_names = [body.name.name for body in path]
print(f"Path from base to tip: {path_names}")
```

## Body Relationships and Connectivity

### Parent-Child Relationships

```python
# Find parent and children of a body
def analyze_body_connectivity(world, body):
    """Analyze how a body connects to others"""
    
    # Find connections where this body is parent
    children = []
    for connection in world.connections:
        if connection.parent == body:
            children.append(connection.child)
    
    # Find connections where this body is child  
    parents = []
    for connection in world.connections:
        if connection.child == body:
            parents.append(connection.parent)
    
    print(f"Body {body.name}:")
    print(f"  Parents: {[p.name for p in parents]}")
    print(f"  Children: {[c.name for c in children]}")
    
    return parents, children

# Analyze connectivity for each body in chain
for body in chain_bodies:
    analyze_body_connectivity(world, body)
```

### Body Neighborhoods

```python
def get_body_neighbors(world, body, max_distance=1):
    """Get bodies within a certain topological distance"""
    neighbors = set()
    current_level = {body}
    
    for distance in range(max_distance):
        next_level = set()
        
        for current_body in current_level:
            # Find directly connected bodies
            for connection in world.connections:
                if connection.parent == current_body:
                    next_level.add(connection.child)
                elif connection.child == current_body:
                    next_level.add(connection.parent)
        
        neighbors.update(next_level)
        current_level = next_level
    
    neighbors.discard(body)  # Remove self
    return neighbors

# Find neighbors
link2 = chain_bodies[2]  # "link2"
neighbors = get_body_neighbors(world, link2, max_distance=2)
print(f"Bodies within distance 2 of {link2.name}: {[b.name for b in neighbors]}")
```

## Body Modification and Updates

### Adding/Removing Geometry

```python
# Start with simple body
body = Body(name="modifiable_body")

# Add geometry dynamically
new_visual = Box(size=[0.1, 0.1, 0.1])
body.visual.append(new_visual)

new_collision = Sphere(radius=0.2) 
body.collision.append(new_collision)

# Remove geometry
if len(body.visual) > 1:
    removed_visual = body.visual.pop()
    print(f"Removed visual: {type(removed_visual).__name__}")

# Clear all geometry
body.visual.clear()
body.collision.clear()
```

### Updating Geometry Poses

```python
# Update poses of existing geometry
for shape in body.visual:
    # Move all visual shapes up by 0.1m
    current_pose = shape.origin.matrix
    current_pose[2, 3] += 0.1
    shape.origin.matrix = current_pose

# Set specific geometry pose
if body.visual:
    first_visual = body.visual[0]
    new_pose = np.eye(4)
    new_pose[:3, 3] = [0.5, 0, 0]  # 0.5m in x
    first_visual.origin = TransformationMatrix(new_pose)
```

## Body Views and Semantic Meaning

Bodies can participate in semantic views:

```python
from semantic_world.views.views import Table, Drawer

# Create bodies for a table
table_top = Body(name="table_top", visual=[Box(size=[1.2, 0.8, 0.05])])
table_leg1 = Body(name="table_leg1", visual=[Cylinder(radius=0.03, height=0.8)])
table_leg2 = Body(name="table_leg2", visual=[Cylinder(radius=0.03, height=0.8)])
table_leg3 = Body(name="table_leg3", visual=[Cylinder(radius=0.03, height=0.8)])
table_leg4 = Body(name="table_leg4", visual=[Cylinder(radius=0.03, height=0.8)])

world = World()
table_bodies = [table_top, table_leg1, table_leg2, table_leg3, table_leg4]

with world.modify_world():
    for body in table_bodies:
        world.add_body(body)

    # Create table view
    table_view = Table(
        bodies=table_bodies,
        name="dining_table"
    )
    world.add_view(table_view)

# Bodies can be in multiple views
print(f"Table top is in {len(table_top._views)} views")

# Find views containing a body
for view in world.views:
    if table_top in view.bodies:
        print(f"Table top is in view: {view.name}")
```

## Body Physics and Properties

### Mass Properties (if available)

```python
# Bodies can have mass properties for physics simulation
class PhysicsBody(Body):
    def __init__(self, name, mass=1.0, inertia=None, **kwargs):
        super().__init__(name=name, **kwargs)
        self.mass = mass
        self.inertia = inertia or np.eye(3)

physics_body = PhysicsBody(
    name="physics_body",
    mass=5.0,
    visual=[Box(size=[0.2, 0.2, 0.2])]
)
```

### Surface Properties

```python
# Add material/surface properties to geometry
class MaterialShape(Box):
    def __init__(self, size, material="default", friction=0.6, **kwargs):
        super().__init__(size=size, **kwargs)
        self.material = material
        self.friction = friction

material_box = MaterialShape(
    size=[1.0, 1.0, 1.0],
    material="wood",
    friction=0.8
)

body_with_material = Body(
    name="material_body",
    visual=[material_box],
    collision=[material_box]
)
```

## Body Collections and Utilities

### Bounding Boxes

```python
from semantic_world.geometry import BoundingBoxCollection

def compute_body_bounds(body):
    """Compute bounding box for a body"""
    if not body.collision:
        return None
    
    # Create collection of collision shapes
    bbox_collection = BoundingBoxCollection(body.collision)
    
    # Get overall bounding box
    min_bounds, max_bounds = bbox_collection.bounds
    size = max_bounds - min_bounds
    
    print(f"Body {body.name} bounds:")
    print(f"  Min: {min_bounds}")
    print(f"  Max: {max_bounds}")  
    print(f"  Size: {size}")
    
    return min_bounds, max_bounds

# Compute bounds for bodies with collision geometry
for body in world.bodies:
    if body.collision:
        compute_body_bounds(body)
```

### Body Grouping

```python
def group_bodies_by_prefix(world):
    """Group bodies by their name prefix"""
    groups = {}
    
    for body in world.bodies:
        prefix = body.name.prefix or "no_prefix"
        if prefix not in groups:
            groups[prefix] = []
        groups[prefix].append(body)
    
    return groups

def group_bodies_by_type(world):
    """Group bodies by their geometry types"""
    groups = {
        'with_mesh': [],
        'with_primitives': [],
        'no_geometry': []
    }
    
    for body in world.bodies:
        has_mesh = any(isinstance(shape, Mesh) for shape in body.visual + body.collision)
        has_geometry = len(body.visual) + len(body.collision) > 0
        
        if has_mesh:
            groups['with_mesh'].append(body)
        elif has_geometry:
            groups['with_primitives'].append(body)
        else:
            groups['no_geometry'].append(body)
    
    return groups

# Group and analyze bodies
prefix_groups = group_bodies_by_prefix(world)
for prefix, bodies in prefix_groups.items():
    print(f"Prefix '{prefix}': {len(bodies)} bodies")

type_groups = group_bodies_by_type(world)
for group_type, bodies in type_groups.items():
    print(f"{group_type}: {len(bodies)} bodies")
```

## Advanced Body Operations

### Body Copying

```python
def copy_body(original_body, new_name):
    """Create a copy of a body with new name"""
    copied_body = Body(
        name=new_name,
        visual=[shape for shape in original_body.visual],  # Shallow copy shapes
        collision=[shape for shape in original_body.collision]
    )
    return copied_body

# Create template and copies
template_body = Body(
    name="template",
    visual=[Box(size=[0.1, 0.1, 0.1])],
    collision=[Box(size=[0.1, 0.1, 0.1])]
)

# Make copies for repeated structures
copies = []
with world.modify_world():
    for i in range(3):
        copy = copy_body(template_body, f"copy_{i}")
        copies.append(copy)
        world.add_body(copy)
```

### Body Validation

```python
def validate_body(body):
    """Validate a body's configuration"""
    issues = []
    
    # Check name
    if not body.name or not body.name.name:
        issues.append("Body has no name")
    
    # Check geometry consistency
    if len(body.visual) == 0 and len(body.collision) == 0:
        issues.append("Body has no geometry (visual or collision)")
    
    # Check geometry origins
    for i, shape in enumerate(body.visual):
        if shape.origin is None:
            issues.append(f"Visual shape {i} has no origin transform")
    
    for i, shape in enumerate(body.collision):
        if shape.origin is None:
            issues.append(f"Collision shape {i} has no origin transform")
    
    return issues

# Validate all bodies in world
for body in world.bodies:
    issues = validate_body(body)
    if issues:
        print(f"Body {body.name} issues:")
        for issue in issues:
            print(f"  - {issue}")
```

## Body Best Practices

1. **Naming Convention**: Use consistent, descriptive names with prefixes for organization
2. **Geometry Separation**: Keep visual and collision geometry separate for performance
3. **Local Origins**: Position geometry relative to meaningful body frames
4. **Validation**: Always validate body configuration after modifications
5. **Memory Management**: Be careful with geometry copying and references
6. **Semantic Meaning**: Use views to give bodies semantic meaning beyond geometry

### Example: Complete Body Management Workflow

```python
def create_robot_link(name, length, radius, mass=1.0):
    """Create a standardized robot link body"""
    
    # Create visual geometry (slightly larger for appearance)
    visual_cylinder = Cylinder(radius=radius*1.1, height=length)
    visual_sphere_start = Sphere(radius=radius*1.2)
    visual_sphere_end = Sphere(radius=radius*1.2)
    
    # Position end spheres
    end_transform = np.eye(4)
    end_transform[2, 3] = length
    visual_sphere_end.origin = TransformationMatrix(end_transform)
    
    # Create collision geometry (simpler)
    collision_cylinder = Cylinder(radius=radius, height=length)
    
    # Create the body
    link = Body(
        name=name,
        visual=[visual_cylinder, visual_sphere_start, visual_sphere_end],
        collision=[collision_cylinder]
    )
    
    # Add mass property (custom attribute)
    link.mass = mass
    
    return link

# Use the factory function
robot_links = [
    create_robot_link(f"link_{i}", length=0.3, radius=0.05, mass=2.0)
    for i in range(4)
]

# Add to world and connect
world = World()
with world.modify_world():
    for link in robot_links:
        world.add_body(link)

    # Connect in chain
    for i in range(1, len(robot_links)):
        connection = FixedConnection(
            parent=robot_links[i-1],
            child=robot_links[i]
        )
        world.add_connection(connection)

print(f"Created robot with {len(robot_links)} links")
world.validate()
```

This guide covers the essential aspects of body management in semantic worlds. Bodies are the foundation of your kinematic structure, so proper management is crucial for building robust and maintainable systems.