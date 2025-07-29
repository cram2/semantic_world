from semantic_world.reasoner import WorldReasoner
from semantic_world.adapters.urdf import URDFParser
from semantic_world.views.views import Drawer


def create_kitchen_world(kitchen_path: str = '../resources/urdf/kitchen-small.urdf'):
    return URDFParser(kitchen_path).parse()


kitchen_world = create_kitchen_world()
reasoner = WorldReasoner(kitchen_world)

reasoner.fit_views([Drawer], update_existing_views=True, world_factory=create_kitchen_world)
