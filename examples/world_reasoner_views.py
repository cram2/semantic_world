from os.path import join, dirname

from semantic_world.reasoning.world_reasoner import WorldReasoner
from semantic_world.adapters.urdf import URDFParser


kitchen_world = URDFParser(
    join(dirname(__file__), "..", "resources", "urdf", "kitchen-small.urdf")
).parse()
reasoner = WorldReasoner(kitchen_world)
found_concepts = reasoner.reason()

# 1st method, access the views directly from the reasoning result
new_views = found_concepts["views"]
assert len(new_views)

# Or 2nd method, access all the views from the world.views,
# but this will include all views not just the new ones.
all_views = kitchen_world.views
assert len(all_views)
