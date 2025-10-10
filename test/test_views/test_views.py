import logging

from entity_query_language import entity, let, in_, infer, rule_mode
from numpy.ma.testutils import (
    assert_equal,
)  # You could replace this with numpy's regular assert for better compatibility

from semantic_world.reasoning.world_reasoner import WorldReasoner
from semantic_world.testing import *
from semantic_world.views.views import *

try:
    from ripple_down_rules.user_interface.gui import RDRCaseViewer
    from PyQt6.QtWidgets import QApplication
except ImportError as e:
    logging.debug(e)
    QApplication = None
    RDRCaseViewer = None

try:
    from semantic_world.reasoning.world_rdr import world_rdr
except ImportError:
    world_rdr = None


@dataclass(eq=False)
class TestView(View):
    """
    A Generic View for multiple bodies.
    """

    _private_entity: KinematicStructureEntity = field(default=None)
    entity_list: List[KinematicStructureEntity] = field(
        default_factory=list, hash=False
    )
    views: List[View] = field(default_factory=list, hash=False)
    root_entity_1: KinematicStructureEntity = field(default=None)
    root_entity_2: KinematicStructureEntity = field(default=None)
    tip_entity_1: KinematicStructureEntity = field(default=None)
    tip_entity_2: KinematicStructureEntity = field(default=None)

    def add_entity(self, body: KinematicStructureEntity):
        self.entity_list.append(body)
        body._views.add(self)

    def add_view(self, view: View):
        self.views.append(view)
        view._views.add(self)

    @property
    def chain(self) -> list[KinematicStructureEntity]:
        """
        Returns itself as a kinematic chain.
        """
        return self._world.compute_chain_of_kinematic_structure_entities(
            self.root_entity_1, self.tip_entity_1
        )

    @property
    def _private_chain(self) -> list[KinematicStructureEntity]:
        """
        Private chain computation.
        """
        return self._world.compute_chain_of_kinematic_structure_entities(
            self.root_entity_2, self.tip_entity_2
        )


def test_view_hash(apartment_world):
    view1 = Handle(body=apartment_world.bodies[0])
    apartment_world.add_view(view1)
    assert hash(view1) == hash((Handle, apartment_world.bodies[0].index))

    view2 = Handle(body=apartment_world.bodies[0])
    assert view1 == view2


def test_aggregate_bodies(kitchen_world):
    world_view = TestView(_world=kitchen_world)

    # Test bodies added to a private dataclass field are not aggregated
    world_view._private_entity = kitchen_world.kinematic_structure_entities[0]

    # Test aggregation of bodies added in custom properties
    world_view.root_entity_1 = kitchen_world.kinematic_structure_entities[1]
    world_view.tip_entity_1 = kitchen_world.kinematic_structure_entities[4]

    # Test aggregation of normal dataclass field
    body_subset = kitchen_world.kinematic_structure_entities[5:10]
    [world_view.add_entity(body) for body in body_subset]

    # Test aggregation of bodies in a new as well as a nested view
    view1 = TestView()
    view1_subset = kitchen_world.kinematic_structure_entities[10:18]
    [view1.add_entity(body) for body in view1_subset]

    view2 = TestView()
    view2_subset = kitchen_world.kinematic_structure_entities[20:]
    [view2.add_entity(body) for body in view2_subset]

    view1.add_view(view2)
    world_view.add_view(view1)

    # Test that bodies added in a custom private property are not aggregated
    world_view.root_entity_2 = kitchen_world.kinematic_structure_entities[18]
    world_view.tip_entity_2 = kitchen_world.kinematic_structure_entities[20]

    assert_equal(
        world_view.kinematic_structure_entities,
        set(kitchen_world.kinematic_structure_entities)
        - {
            kitchen_world.kinematic_structure_entities[0],
            kitchen_world.kinematic_structure_entities[19],
        },
    )


def test_handle_view_eql(apartment_world):
    with rule_mode():
        body = let(
            type_=Body,
        )
        query = infer(entity(Handle(body=body), in_("handle", body.name.name.lower())))

    handles = list(query.evaluate())
    assert len(handles) > 0


@pytest.mark.parametrize(
    "view_type, update_existing_views, scenario",
    [
        (Handle, False, None),
        (Container, False, None),
        (Drawer, False, None),
        (Cabinet, False, None),
        (Door, False, None),
    ],
)
def test_infer_apartment_view(
    view_type, update_existing_views, scenario, apartment_world
):
    fit_rules_and_assert_views(
        apartment_world, view_type, update_existing_views, scenario
    )


@pytest.mark.skipif(world_rdr is None, reason="requires world_rdr")
def test_generated_views(kitchen_world):
    found_views = world_rdr.classify(kitchen_world)["views"]
    drawer_container_names = [
        v.body.name.name for v in found_views if isinstance(v, Container)
    ]
    assert len(drawer_container_names) == 14


@pytest.mark.order("second_to_last")
def test_apartment_views(apartment_world):
    world_reasoner = WorldReasoner(apartment_world)
    world_reasoner.fit_views(
        [Handle, Container, Drawer, Cabinet],
        world_factory=lambda: apartment_world,
        scenario=None,
    )

    found_views = world_reasoner.infer_views()
    drawer_container_names = [
        v.body.name.name for v in found_views if isinstance(v, Container)
    ]
    assert len(drawer_container_names) == 19


def fit_rules_and_assert_views(world, view_type, update_existing_views, scenario):
    world_reasoner = WorldReasoner(world)
    world_reasoner.fit_views(
        [view_type],
        update_existing_views=update_existing_views,
        world_factory=lambda: world,
        scenario=scenario,
    )

    found_views = world_reasoner.infer_views()
    assert any(isinstance(v, view_type) for v in found_views)


def test_view_serde_once(apartment_world):
    handle_body = apartment_world.bodies[0]
    door_body = apartment_world.bodies[1]

    handle = Handle(body=handle_body)
    door = Door(body=door_body, handle=handle)

    apartment_world.add_view(handle)
    apartment_world.add_view(door)

    door_se = door.to_json()
    door_de = Door.from_json(door_se)

    assert door == door_de
    assert type(door.handle) == type(door_de.handle)
    assert type(door.body) == type(door_de.body)


def test_view_serde_multiple(apartment_world):
    handle_body = apartment_world.bodies[0]
    door_body = apartment_world.bodies[1]

    handle = Handle(body=handle_body)
    door = Door(body=door_body, handle=handle)

    apartment_world.add_view(handle)
    apartment_world.add_view(door)

    door_se1 = door.to_json()
    door_de1 = Door.from_json(door_se1)

    assert door == door_de1
    assert type(door.handle) == type(door_de1.handle)
    assert type(door.body) == type(door_de1.body)

    door_se2 = door_de1.to_json()
    door_de2 = Door.from_json(door_se2)

    assert door == door_de2
    assert type(door.handle) == type(door_de2.handle)
    assert type(door.body) == type(door_de2.body)
