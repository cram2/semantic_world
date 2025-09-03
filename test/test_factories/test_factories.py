import unittest

from semantic_world.world_description.geometry import Scale
from semantic_world.datastructures.prefixed_name import PrefixedName
from semantic_world.spatial_types.spatial_types import TransformationMatrix
from semantic_world.views.views import Handle, Door, Container, Drawer, Dresser, Wall
from semantic_world.views.factories import (
    HandleFactory,
    Direction,
    DoorFactory,
    ContainerFactory,
    DoubleDoorFactory,
    DrawerFactory,
    DresserFactory,
    WallFactory,
)


class TestFactories(unittest.TestCase):
    def test_handle_factory(self):

        factory = HandleFactory(name=PrefixedName("handle"))
        world = factory.create()
        handle_views = world.get_views_by_type(Handle)
        self.assertEqual(len(handle_views), 1)

        handle: Handle = handle_views[0]
        self.assertEqual(world.root, handle.body)

        # this belongs into whatever tests merge_world, and with dummy objects, not handles
        for i in range(10):
            factory = HandleFactory(name=PrefixedName(f"handle_{i}"))
            world.merge_world(factory.create())

        self.assertEqual(world.root.name.name, "handle")
        handle_views = world.get_views_by_type(Handle)
        self.assertEqual(11, len(handle_views))
        self.assertEqual(11, len(world.bodies))

    def test_door_factory(self):
        factory = DoorFactory(
            name=PrefixedName("door"),
            handle_factory=HandleFactory(name=PrefixedName("handle")),
            handle_direction=Direction.Y,
        )
        world = factory.create()
        door_views = world.get_views_by_type(Door)
        self.assertEqual(len(door_views), 1)

        door: Door = door_views[0]
        self.assertEqual(world.root, door.body)
        self.assertIsInstance(door.handle, Handle)

    def test_double_door_factory(self):
        factory = DoubleDoorFactory(
            name=PrefixedName("double_door"),
            handle_factory=HandleFactory(name=PrefixedName("handle")),
        )
        world = factory.create()
        door_views = world.get_views_by_type(Door)
        self.assertEqual(len(door_views), 2)

        doors: list[Door] = door_views
        self.assertEqual(
            set(world.root.child_kinematic_structure_entities),
            {doors[0].body, doors[1].body},
        )
        self.assertIsInstance(doors[0].handle, Handle)
        self.assertIsInstance(doors[1].handle, Handle)
        self.assertNotEqual(doors[0].handle, doors[1].handle)

    def test_container_factory(self):
        factory = ContainerFactory(name=PrefixedName("container"))
        world = factory.create()
        container_views = world.get_views_by_type(Container)
        self.assertEqual(len(container_views), 1)

        container: Container = container_views[0]
        self.assertEqual(world.root, container.body)

    def test_drawer_factory(self):

        factory = DrawerFactory(
            name=PrefixedName("drawer"),
            container_factory=ContainerFactory(name=PrefixedName("container")),
            handle_factory=HandleFactory(name=PrefixedName("handle")),
        )
        world = factory.create()
        drawer_views = world.get_views_by_type(Drawer)
        self.assertEqual(len(drawer_views), 1)

        drawer: Drawer = drawer_views[0]
        self.assertEqual(world.root, drawer.container.body)

    def test_dresser_factory(self):
        drawer_factory = DrawerFactory(
            name=PrefixedName("drawer"),
            container_factory=ContainerFactory(name=PrefixedName("drawer_container")),
            handle_factory=HandleFactory(name=PrefixedName("drawer_handle")),
        )
        drawer_transform = TransformationMatrix()

        door_factory = DoorFactory(
            name=PrefixedName("door"),
            handle_factory=HandleFactory(name=PrefixedName("door_handle")),
            handle_direction=Direction.Y,
        )

        door_transform = TransformationMatrix()

        container_factory = ContainerFactory(name=PrefixedName("dresser_container"))

        dresser_factory = DresserFactory(
            name=PrefixedName("dresser"),
            drawer_transforms=[drawer_transform],
            drawers_factories=[drawer_factory],
            door_transforms=[door_transform],
            door_factories=[door_factory],
            container_factory=container_factory,
        )

        world = dresser_factory.create()
        dresser_views = world.get_views_by_type(Dresser)
        drawers_views = world.get_views_by_type(Drawer)
        door_views = world.get_views_by_type(Door)
        self.assertEqual(len(drawers_views), 1)
        self.assertEqual(len(dresser_views), 1)
        self.assertEqual(len(door_views), 1)
        dresser: Dresser = dresser_views[0]
        self.assertEqual(world.root, dresser.container.body)

    def test_wall_factory(self):

        door_factory = DoorFactory(
            name=PrefixedName("door"),
            handle_factory=HandleFactory(name=PrefixedName("handle")),
            handle_direction=Direction.Y,
        )

        door_transform = TransformationMatrix()

        double_door_factory = DoubleDoorFactory(
            name=PrefixedName("double_door"),
            handle_factory=HandleFactory(name=PrefixedName("handle")),
        )
        double_door_transform = TransformationMatrix()

        factory = WallFactory(
            name=PrefixedName("wall"),
            scale=Scale(0.1, 4, 2),
            door_transforms=[door_transform, double_door_transform],
            door_factories=[door_factory, double_door_factory],
        )
        world = factory.create()
        wall_views = world.get_views_by_type(Wall)
        self.assertEqual(len(wall_views), 1)

        wall: Wall = wall_views[0]
        self.assertEqual(world.root, wall.body)


if __name__ == "__main__":
    unittest.main()
