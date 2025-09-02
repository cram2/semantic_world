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

The central datastructure for interaction with a scene is the {py:class}`semantic_world.world_entity.World`.
The world is a mediator for bodies and their connections.
It handles the validation of the world's kinematic structure and the communication between the objects.

Physical Objects can be spawned by constructing a {py:class}`semantic_world.world_entity.Body` and a kinematic chain of 
those elements is added by specifying a {py:class}`semantic_world.world_entity.Connection` between bodies.

All those things have to be added to the world for full functionality.
More information on the kinematic world model can be found [here](kinematic_world.md).


## WorldReasoner

The world reasoner {py:class}`semantic_world.reasoner.WorldReasoner` is a class that uses [Ripple Down Rules](https://github.com/AbdelrhmanBassiouny/ripple_down_rules/tree/main)
to classify concepts and attributes of the world. This is done using a rule based classifier that benefits from incremental
rule addition through querying the system and answering the prompts that pop up using python code.

The benefit of that is the rules of the reasoner are based on the world datastructures and are updates as the datastructures
are updated. Thus, the rules become a part of the semantic world repository and are update, migrated, and versioned with it.

More information about the world reasoner can be found [here](world_reasoner.md).

## Views

A View ({py:class}`semantic_world.world_entity.View`) is a different representation for a part or a collection of parts in the world that has a semantic meaning and
functional purpose in specific contexts.

For example, a Drawer can be seen as a view on a handle and a container that is connected via a fixed connection
and where the container has some prismatic connection.

Views can be inferred by specifying rules that make up a view. More information on how the views are inferred and used
can be found [here](views.md).

## Database

The entire world can be saved to any database
that has an [sqlalchemy](https://docs.sqlalchemy.org/en/20/index.html) connector.
The definitions and relationships for the database are automatically derived from the datastructures
derived in the python package via the [ormatic](https://github.com/tomsch420/ormatic) package.
The types for sqlalchemy are defined in {py:mod}`semantic_world.orm.model`.
The interface to sqlalchemy is auto-generated to {py:mod}`semantic_world.orm.ormatic_interface`.
The script to recreate the interface is found in [here](https://github.com/cram2/semantic_world/blob/main/scripts/generate_orm.py).

Learn more about the ORM in [this tutorial](orm-guide).


