from os.path import join, dirname

from semantic_world.reasoning.world_reasoner import WorldReasoner
from semantic_world.adapters.urdf import URDFParser
from semantic_world.views.views import Drawer


# This exists to pass the method to as a world factory.
def create_kitchen_world():
    return URDFParser.from_file(
        join(dirname(__file__), "..", "resources", "urdf", "kitchen-small.urdf")
    ).parse()


kitchen_world = create_kitchen_world()
reasoner = WorldReasoner(kitchen_world)

# If you want to fit a new rule, set `update_existing_views=True`
reasoner.fit_views(
    [Drawer], update_existing_views=False, world_factory=create_kitchen_world
)
