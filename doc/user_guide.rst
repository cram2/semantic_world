===========================================================
Semantic World User Guide: From Beginner to World Builder
===========================================================

Welcome to the Semantic World package! This comprehensive guide will take you from installation to creating, manipulating, and saving complex robotic worlds. You'll learn to work with world structures, manage world states, create semantic views, and persist everything to a database.

.. contents::
   :local:
   :depth: 2

What You'll Build
=================

By the end of this tutorial, you'll have built an interactive kitchen scene where:

- A robotic table is loaded from a URDF file
- Custom objects (like a coffee mug and fruit bowl) are placed on the table
- Semantic views identify functional areas (like surfaces and containers)
- The entire world is saved to and retrieved from a database
- You can manipulate and query the world programmatically

Installation and Setup
======================

Prerequisites
-------------

Before we begin, ensure you have:

- Python 3.9 or higher
- Basic knowledge of robotics concepts (frames, transforms, URDF)
- Familiarity with Python data structures

Installing Semantic World
-------------------------

First, install the semantic world package:

.. code-block:: bash

   pip install semantic_world

Setting Up ROS2 (Required)
---------------------------

The semantic world package requires ROS2 for certain functionalities:

.. code-block:: python

   import rclpy
   
   # Initialize ROS2 - call this once at the start of your program
   rclpy.init()

Chapter 1: Loading Your First World from URDF
=============================================

Let's start by loading a simple table from a URDF file. URDF (Unified Robot Description Format) is a standard way to describe robot structures.

Loading a Table World
---------------------

.. code-block:: python

   import os
   from semantic_world.adapters.urdf import URDFParser
   
   # Initialize ROS2
   import rclpy
   rclpy.init()
   
   # Path to the table URDF file
   urdf_path = "resources/urdf/table.urdf"
   
   # Parse and load the world
   parser = URDFParser.from_file(urdf_path)
   world = parser.parse()
   
   print(f"Loaded world: {world.name}")
   print(f"Number of bodies: {len(world.bodies)}")
   print(f"Number of connections: {len(world.connections)}")

Understanding the Loaded World
------------------------------

Now let's explore what we've loaded:

.. code-block:: python

   # Examine the bodies in our world
   print("Bodies in the world:")
   for body in world.bodies:
       print(f"  - {body.name}: {len(body.visual)} visual shapes, {len(body.collision)} collision shapes")
   
   # Look at the connections (joints) between bodies
   print("\nConnections in the world:")
   for connection in world.connections:
       print(f"  - {connection.parent.name} -> {connection.child.name}")
   
   # Find the table top
   table_top = None
   for body in world.bodies:
       if "top" in str(body.name):
           table_top = body
           break
   
   if table_top:
       print(f"\nFound table top: {table_top.name}")
       print(f"Table top pose: {table_top.global_pose}")

Chapter 2: Adding Custom Bodies to Your World
==============================================

Now let's make our world more interesting by adding custom objects to the table.

Creating a Coffee Mug
---------------------

.. code-block:: python

   from semantic_world.world_description.world_entity import Body
   from semantic_world.world_description.geometry import Cylinder, Pose
   from semantic_world.datastructures.prefixed_name import PrefixedName
   from semantic_world.spatial_types import Point3, Quaternion
   import numpy as np
   
   # Create a coffee mug using a cylindrical shape
   mug_body = Body(
       name=PrefixedName("coffee_mug"),
       visual=[
           Cylinder(
               radius=0.04,  # 4cm radius
               height=0.1,   # 10cm height
               origin=Pose(Point3(0, 0, 0.05))  # Center the mug
           )
       ],
       collision=[
           Cylinder(
               radius=0.04,
               height=0.1,
               origin=Pose(Point3(0, 0, 0.05))
           )
       ]
   )
   
   # Position the mug on the table
   mug_body.pose = Pose(
       position=Point3(0.2, 0.1, 0.65),  # 20cm right, 10cm forward, table height + mug height/2
       orientation=Quaternion(0, 0, 0, 1)  # No rotation
   )
   
   # Add the mug to our world
   world.add_body(mug_body)
   print(f"Added {mug_body.name} to the world")

Creating a Fruit Bowl
---------------------

.. code-block:: python

   from semantic_world.world_description.geometry import Sphere
   
   # Create a fruit bowl (represented as a sphere)
   bowl_body = Body(
       name=PrefixedName("fruit_bowl"),
       visual=[
           Sphere(
               radius=0.08,  # 8cm radius
               origin=Pose(Point3(0, 0, 0))
           )
       ],
       collision=[
           Sphere(
               radius=0.08,
               origin=Pose(Point3(0, 0, 0))
           )
       ]
   )
   
   # Position the bowl on the opposite side of the table
   bowl_body.pose = Pose(
       position=Point3(-0.2, -0.1, 0.68),  # Left side of table
       orientation=Quaternion(0, 0, 0, 1)
   )
   
   world.add_body(bowl_body)
   print(f"Added {bowl_body.name} to the world")
   
   # Let's see our updated world
   print(f"\nUpdated world now has {len(world.bodies)} bodies:")
   for body in world.bodies:
       print(f"  - {body.name}")

Chapter 3: Creating and Managing Semantic Views
===============================================

Views in semantic world provide semantic meaning to physical structures. They help us understand what objects are and how they can be used.

Creating a Table View
---------------------

.. code-block:: python

   from semantic_world.views.views import Table
   
   # Find the table top body (we found this earlier)
   if table_top:
       # Create a semantic view that identifies this as a table
       table_view = Table(body=table_top)
       
       # Add the view to our world
       world.add_view(table_view)
       
       print(f"Created table view: {table_view.name}")
       print(f"Table surface area: {table_view.surface_area}")
       print(f"Can place objects: {table_view.can_place_objects}")

Creating Custom Container Views
-------------------------------

.. code-block:: python

   from semantic_world.views.views import Container
   
   # Create a container view for our fruit bowl
   bowl_container = Container(body=bowl_body)
   
   # Add it to the world
   world.add_view(bowl_container)
   
   print(f"Created container view: {bowl_container.name}")
   print(f"Container can hold objects: {bowl_container.can_contain}")

Working with Multiple Views
---------------------------

.. code-block:: python

   # Get all views in the world
   all_views = world.views
   print(f"\nAll views in the world ({len(all_views)}):")
   for view in all_views:
       print(f"  - {view.name} (type: {type(view).__name__})")
   
   # Get views by type
   table_views = world.get_views_by_type(Table)
   container_views = world.get_views_by_type(Container)
   
   print(f"\nTable views: {len(table_views)}")
   print(f"Container views: {len(container_views)}")
   
   # Get a specific view by name
   my_table = world.get_view_by_name("Table_table_top")
   if my_table:
       print(f"Found table view: {my_table.name}")

Chapter 4: World State Management
=================================

World states represent the current configuration of all movable objects and joints in your world.

Understanding World State
-------------------------

.. code-block:: python

   from semantic_world.world_description.world_state import WorldState
   
   # Get the current world state
   current_state = world.get_world_state()
   print(f"Current world state has {len(current_state.values)} state variables")
   
   # Examine what state variables exist
   state_symbols = world.get_world_state_symbols()
   print("State variables:")
   for symbol in state_symbols:
       print(f"  - {symbol}")

Modifying World State
--------------------

.. code-block:: python

   # If your world has movable joints, you can modify their states
   # For our table example, let's modify the position of our mug
   
   # Get mug body and modify its pose
   mug = world.get_kinematic_structure_entity_by_name(PrefixedName("coffee_mug"))
   if mug:
       # Create a new pose for the mug
       new_pose = Pose(
           position=Point3(0.0, 0.0, 0.65),  # Center of table
           orientation=Quaternion(0, 0, 0, 1)
       )
       
       # Update the mug's pose
       mug.pose = new_pose
       print(f"Moved mug to center of table: {new_pose.position}")

Chapter 5: Saving Worlds to Database
====================================

One of the powerful features of semantic world is the ability to persist entire worlds to a database.

Setting Up the Database
-----------------------

.. code-block:: python

   from sqlalchemy import create_engine
   from sqlalchemy.orm import Session
   from semantic_world.orm.ormatic_interface import Base
   from ormatic.dao import to_dao
   
   # Create an in-memory SQLite database (for this tutorial)
   # In production, you'd use a persistent database
   engine = create_engine('sqlite:///:memory:', echo=False)
   session = Session(engine)
   
   # Create all tables
   Base.metadata.create_all(bind=session.bind)
   print("Database initialized successfully!")

Saving Your World
-----------------

.. code-block:: python

   # Convert the world to a Data Access Object (DAO)
   world_dao = to_dao(world)
   
   # Add it to the database session
   session.add(world_dao)
   
   # Commit the transaction
   session.commit()
   
   print(f"World '{world.name}' saved to database!")
   print(f"Saved {len(world.bodies)} bodies and {len(world.views)} views")

Retrieving Worlds from Database
------------------------------

.. code-block:: python

   from sqlalchemy import select
   from semantic_world.orm.ormatic_interface import WorldMappingDAO
   
   # Query the database for our world
   queried_world_dao = session.scalars(select(WorldMappingDAO)).first()
   
   if queried_world_dao:
       # Reconstruct the world from the database
       reconstructed_world = queried_world_dao.from_dao()
       
       print(f"Retrieved world: {reconstructed_world.name}")
       print(f"Bodies: {len(reconstructed_world.bodies)}")
       print(f"Views: {len(reconstructed_world.views)}")
       
       # Verify our views are intact
       reconstructed_tables = reconstructed_world.get_views_by_type(Table)
       reconstructed_containers = reconstructed_world.get_views_by_type(Container)
       
       print(f"Retrieved {len(reconstructed_tables)} table views")
       print(f"Retrieved {len(reconstructed_containers)} container views")
   else:
       print("No world found in database")

Chapter 6: Advanced World Manipulation
======================================

Now that you understand the basics, let's explore more advanced features.

Connecting Bodies with Joints
-----------------------------

.. code-block:: python

   from semantic_world.world_description.connections import FixedConnection
   
   # Create a connection between the mug and table
   # This would make the mug "attached" to the table
   mug_to_table_connection = FixedConnection(
       parent=table_top,
       child=mug_body,
       origin=Pose(Point3(0.2, 0.1, 0.05))  # Relative position
   )
   
   # Add the connection to the world
   world.add_connection(mug_to_table_connection)
   print("Connected mug to table with fixed joint")

Working with World Modifications
--------------------------------

.. code-block:: python

   # The world keeps track of modifications for synchronization
   with world.modify_world():
       # Any changes made in this context are tracked
       
       # Add another object
       plate_body = Body(
           name=PrefixedName("dinner_plate"),
           visual=[
               Cylinder(
                   radius=0.12,  # 12cm radius
                   height=0.02,  # 2cm height (thin plate)
                   origin=Pose(Point3(0, 0, 0.01))
               )
           ]
       )
       
       world.add_body(plate_body)
       print("Added dinner plate to the world")
   
   print("World modification context completed")

Collision Detection Setup
-------------------------

.. code-block:: python

   from semantic_world.world_description.world_entity import CollisionCheckingConfig
   
   # Configure collision checking for our objects
   for body in world.bodies:
       if body.has_collision():
           # Set up collision configuration
           collision_config = CollisionCheckingConfig(
               buffer_zone_distance=0.05,  # 5cm buffer zone
               violated_distance=0.01,     # 1cm violation threshold
               disabled=False              # Enable collision checking
           )
           body.set_static_collision_config(collision_config)
   
   print("Collision checking configured for all bodies")
   
   # Check which bodies have collision enabled
   collision_enabled_bodies = world.bodies_with_enabled_collision
   print(f"Bodies with collision enabled: {len(collision_enabled_bodies)}")

Chapter 7: Practical Robotics Integration
=========================================

Let's explore how to use your semantic world in practical robotics applications.

Querying Spatial Relationships
------------------------------

.. code-block:: python

   # Find objects on the table surface
   table_height = 0.6  # Table height from URDF
   objects_on_table = []
   
   for body in world.bodies:
       if body != table_top and hasattr(body, 'pose'):
           body_z = body.pose.position.z
           # Check if object is roughly at table height
           if abs(body_z - (table_height + 0.05)) < 0.1:  # Within 10cm of table surface
               objects_on_table.append(body)
   
   print(f"Objects on table surface: {len(objects_on_table)}")
   for obj in objects_on_table:
       print(f"  - {obj.name} at position {obj.pose.position}")

Planning with Semantic Information
---------------------------------

.. code-block:: python

   # Use semantic views for planning
   
   # Find all containers that can hold objects
   available_containers = []
   for view in world.get_views_by_type(Container):
       if hasattr(view, 'can_contain') and view.can_contain:
           available_containers.append(view)
   
   print(f"Available containers for object placement: {len(available_containers)}")
   
   # Find all surfaces for placing objects
   available_surfaces = []
   for view in world.get_views_by_type(Table):
       if hasattr(view, 'can_place_objects') and view.can_place_objects:
           available_surfaces.append(view)
   
   print(f"Available surfaces for object placement: {len(available_surfaces)}")

Forward Kinematics Computation
------------------------------

.. code-block:: python

   # Compute the global pose of any body in the world
   for body in world.bodies[:3]:  # Just first 3 bodies
       try:
           global_pose = world.compute_forward_kinematics(world.root, body)
           print(f"{body.name} global pose:")
           print(f"  Position: {global_pose.position}")
           print(f"  Orientation: {global_pose.orientation}")
       except Exception as e:
           print(f"Could not compute FK for {body.name}: {e}")

Troubleshooting Common Issues
============================

Database Connection Issues
--------------------------

If you encounter database connection problems:

.. code-block:: python

   # Check if tables were created properly
   from sqlalchemy import inspect
   
   inspector = inspect(engine)
   table_names = inspector.get_table_names()
   print(f"Tables in database: {table_names}")
   
   if not table_names:
       print("No tables found. Recreating...")
       Base.metadata.create_all(bind=session.bind)

ROS2 Initialization Problems
----------------------------

If ROS2 initialization fails:

.. code-block:: python

   try:
       rclpy.init()
       print("ROS2 initialized successfully")
   except Exception as e:
       print(f"ROS2 initialization failed: {e}")
       print("Make sure ROS2 is properly installed")

URDF Loading Issues
------------------

If URDF files fail to load:

.. code-block:: python

   import os
   
   # Verify file exists
   urdf_path = "resources/urdf/table.urdf"
   if not os.path.exists(urdf_path):
       print(f"URDF file not found: {urdf_path}")
       print("Available URDF files:")
       urdf_dir = "resources/urdf"
       if os.path.exists(urdf_dir):
           for file in os.listdir(urdf_dir):
               if file.endswith('.urdf'):
                   print(f"  - {file}")

Next Steps and Advanced Topics
=============================

Congratulations! You've learned the fundamentals of working with semantic worlds. Here are some advanced topics to explore:

Advanced Views and Reasoning
---------------------------

- Create custom view types for your specific domain
- Use the :class:`semantic_world.reasoner.WorldReasoner` for automated classification
- Implement custom spatial reasoning rules

Multi-Robot Scenarios
---------------------

- Load multiple robot URDF files into a single world
- Manage inter-robot connections and interactions
- Synchronize worlds across multiple processes

Integration with Simulators
---------------------------

- Connect to physics simulators through Multiverse
- Real-time visualization of your semantic worlds
- Sensor simulation and perception integration

Performance Optimization
------------------------

- Efficient collision checking configuration
- Database query optimization for large worlds
- Memory management for complex scenes

Further Reading
===============

For more advanced usage, explore these modules:

- :mod:`semantic_world.world_description.world_entity` - Core world entities
- :mod:`semantic_world.adapters.urdf` - URDF parsing and loading
- :mod:`semantic_world.views.views` - Semantic view definitions
- :mod:`semantic_world.orm.ormatic_interface` - Database ORM interface
- :mod:`semantic_world.reasoner` - World reasoning capabilities

The semantic world package is actively developed. Check the repository for the latest examples and documentation updates.

Happy world building! 🌍🤖