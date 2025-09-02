# Views

A View ({py:class}`semantic_world.world_entity.View`) is a different representation for a part or a collection of parts in the world that has a semantic meaning and
functional purpose in specific contexts.

For example, a Drawer can be seen as a view on a handle and a container that is connected via a fixed connection
and where the container has some prismatic connection.

Views can be inferred automatically by specifying rules that make up a view.

## How to use the views

Views and any other attribute of the world that can be inferred or should be inferred through reasoning can be used
through the world reasoner, you can check how to use the world reasoner [here](world_reasoner.md).

Some helper methods exist in the world reasoner just for the views like {py:func}`semantic_world.reasoner.WorldReasoner.infer_views`
and {py:func}`semantic_world.reasoner.WorldReasoner.fit_views`.

