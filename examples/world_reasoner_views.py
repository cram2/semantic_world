from semantic_world.reasoner import WorldReasoner
from semantic_world.adapters.urdf import URDFParser

def create_kitchen_world(kitchen_path: str = '../resources/urdf/kitchen-small.urdf'):
    return URDFParser(kitchen_path).parse()

kitchen_world = create_kitchen_world()
reasoner = WorldReasoner(kitchen_world)
found_concepts = reasoner.reason()

# 1st method, access the views directly from the reasoning result
new_views = found_concepts['views']
assert len(new_views)

# Or 2nd method, access all the views from the world.views, but this will include all views not just the new ones.
all_views = kitchen_world.views
assert len(all_views)
