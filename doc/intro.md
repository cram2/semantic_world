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

Here is the relevant class diagram.

## Views

## Database

## Adapters

