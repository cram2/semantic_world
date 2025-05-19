# Semantic World


Welcome to the semantic world package!
The semantic world is a python package that unifies the access and manipulation of scene graphs.
It features a:

- Unified API to scene graphs
- Forward simulation for worlds and agents
- Semantic interpretation of structures in the world
- Automatic serialization of worlds to databases
- Connecting worlds to different simulator backends through Multiverse {cite:p}`Multiverse`

This package originates from different developments of the [AICOR Institute for Artificial Intelligence](https://ai.uni-bremen.de/). 
Four different projects developed a very similar component for different parts of cognitive modules.
This project aims to unify it under one solution that is flexible enough for all the different applications.

## World

The central datastructure for interaction with a scene is the {py:class}`semantic_world.world.World`.
The world is a mediator for bodies and their connections.
It handles the validation of the world's kinematic structure and the communication between the objects.

Physical Objects can be spawned by constructing a {py:class}`semantic_world.world.Body` and a kinematic chain of 
those elements is added by specifying a {py:class}`semantic_world.world.Connection` between bodies.

All those things have to be added to the world for full functionality.
More information on the kinematic world model can be found [here](kinematic_world.md).

## Views

Views are aggregation objects for bodies and connections in the world. 
Semantically, a `semantic_world.world.View` is an interpretation of a set of links and connections.
For instance, a Drawer can be seen as a view on a handle and a container that is connected via a fixed connection
and where the container has some prismatic connection.

Views can be inferred by specifying rules that make up a view. TODO Bass stuff

## Database

The entire world can be saved to any database
that has an [sqlalchemy](https://docs.sqlalchemy.org/en/20/index.html) connector.
The definitions and relationships for the database are automatically derived from the datastructures
derived in the python package via the [ormatic](https://github.com/tomsch420/ormatic) package.
Since the datastructures for the forward calculations of the world are not defined compatibly, the types
from {py:mod}`semantic_world.spatial_types.spatial_types` are mapped via JSON as columns.
This is due to the fact, that this package uses casadi to speed up forward kinematics.
The types for sqlalchemy are defined in {py:mod}`semantic_world.orm.model`.
The interface to sqlalchemy is auto-generated to {py:mod}`semantic_world.orm.ormatic_interface`.
The script
to recreate the interface is found in [here](https://github.com/tomsch420/semantic_world/blob/main/scripts/generate_orm.py).


