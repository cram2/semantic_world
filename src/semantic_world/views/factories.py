from dataclasses import dataclass, field
from enum import IntEnum
from typing_extensions import TypeVar, Generic

from numpy import ndarray
from random_events.product_algebra import *

from ..world_description.connections import (
    PrismaticConnection,
    FixedConnection,
    RevoluteConnection,
)
from ..world_description import DegreeOfFreedom, Scale, BoundingBoxCollection, Box, Body
from ..datastructures import PrefixedName, SpatialVariables
from ..spatial_types.derivatives import DerivativeMap
from ..spatial_types.spatial_types import (
    TransformationMatrix,
    Vector3,
    Point3,
)
from ..utils import IDGenerator
from ..views.views import (
    Container,
    Handle,
    Dresser,
    Drawer,
    Door,
    Wall,
    DoubleDoor,
)
from ..world import World

id_generator = IDGenerator()


class Direction(IntEnum):
    X = 0
    Y = 1
    Z = 2
    NEGATIVE_X = 3
    NEGATIVE_Y = 4
    NEGATIVE_Z = 5


def event_from_scale(scale: Scale):
    return SimpleEvent(
        {
            SpatialVariables.x.value: closed(-scale.x / 2, scale.x / 2),
            SpatialVariables.y.value: closed(-scale.y / 2, scale.y / 2),
            SpatialVariables.z.value: closed(-scale.z / 2, scale.z / 2),
        }
    )


T = TypeVar("T")


@dataclass
class ViewFactory(Generic[T], ABC):
    """
    Abstract factory for the creation of worlds containing a single view of type T.
    """

    name: PrefixedName
    """
    The name of the view.
    """

    @abstractmethod
    def _create(self, world: World) -> World:
        """
        Create the world containing a view of type T.
        Put the custom logic in here.

        :param world: The world to create the view in.
        :return: The world.
        """
        raise NotImplementedError()

    def create(self) -> World:
        """
        Create the world containing a view of type T.

        :return: The world.
        """
        world = World()
        with world.modify_world():
            world = self._create(world)
        return world


@dataclass
class ContainerFactory(ViewFactory[Container]):
    """
    Factory for creating a container with walls of a specified thickness and its opening in direction.
    """

    scale: Scale = field(default_factory=lambda: Scale(1.0, 1.0, 1.0))
    """
    The scale of the container, defining its size in the world.
    """

    wall_thickness: float = 0.05
    """
    The thickness of the walls of the container.
    """

    direction: Direction = Direction.X
    """
    The direction in which the container is open.
    """

    def _create(self, world: World) -> World:
        """
        Return a world with a container body at its root.
        """

        container_event = self.create_container_event()

        container_body = Body(name=self.name)
        collision_shapes = BoundingBoxCollection.from_event(
            container_body, container_event
        ).as_shapes()
        container_body.collision = collision_shapes
        container_body.visual = collision_shapes

        container_view = Container(body=container_body, name=self.name)

        world.add_kinematic_structure_entity(container_body)
        world.add_view(container_view)

        return world

    def create_container_event(self) -> Event:
        """
        Return an event representing a container with walls of a specified thickness.
        """
        outer_box = event_from_scale(self.scale)
        inner_scale = Scale(
            self.scale.x - self.wall_thickness,
            self.scale.y - self.wall_thickness,
            self.scale.z - self.wall_thickness,
        )
        inner_box = event_from_scale(inner_scale)

        inner_box = self.extend_inner_event_in_direction(
            inner_event=inner_box, inner_scale=inner_scale
        )

        container_event = outer_box.as_composite_set() - inner_box.as_composite_set()

        return container_event

    def extend_inner_event_in_direction(
        self, inner_event: SimpleEvent, inner_scale: Scale
    ) -> SimpleEvent:
        """
        Extend the inner event in the specified direction to create the container opening in that direction.

        :param inner_event: The inner event representing the inner box.
        :param inner_scale: The scale of the inner box used how far to extend the inner event.

        :return: The modified inner event with the specified direction extended.
        """

        match self.direction:
            case Direction.X:
                inner_event[SpatialVariables.x.value] = closed(
                    -inner_scale.x / 2, self.scale.x / 2
                )
            case Direction.Y:
                inner_event[SpatialVariables.y.value] = closed(
                    -inner_scale.y / 2, self.scale.y / 2
                )
            case Direction.Z:
                inner_event[SpatialVariables.z.value] = closed(
                    -inner_scale.z / 2, self.scale.z / 2
                )
            case Direction.NEGATIVE_X:
                inner_event[SpatialVariables.x.value] = closed(
                    -self.scale.x / 2, inner_scale.x / 2
                )
            case Direction.NEGATIVE_Y:
                inner_event[SpatialVariables.y.value] = closed(
                    -self.scale.y / 2, inner_scale.y / 2
                )
            case Direction.NEGATIVE_Z:
                inner_event[SpatialVariables.z.value] = closed(
                    -self.scale.z / 2, inner_scale.z / 2
                )

        return inner_event


@dataclass
class HandleFactory(ViewFactory[Handle]):
    """
    Factory for creating a handle with a specified scale and thickness.
    The handle is represented as a box with an inner cutout to create the handle shape.
    """

    scale: Scale = field(default_factory=lambda: Scale(0.05, 0.1, 0.02))
    """
    The scale of the handle.
    """

    thickness: float = 0.01
    """
    Thickness of the handle bar.
    """

    def _create(self, world: World) -> World:
        """
        Create a world with a handle body at its root.
        """

        handle_event = self.create_handle_event()

        handle = Body(name=self.name)
        collision = BoundingBoxCollection.from_event(handle, handle_event).as_shapes()
        handle.collision = collision
        handle.visual = collision

        handle_view = Handle(name=self.name, body=handle)

        world.add_kinematic_structure_entity(handle)
        world.add_view(handle_view)
        return world

    def create_handle_event(self) -> Event:
        """
        Return an event representing a handle.
        """

        handle_event = self.create_outer_box_event().as_composite_set()

        inner_box = self.create_inner_box_event().as_composite_set()

        handle_event -= inner_box

        return handle_event

    def create_outer_box_event(self) -> SimpleEvent:
        """
        Return an event representing the main body of a handle.
        """
        x_interval = closed(0, self.scale.x)
        y_interval = closed(-self.scale.y / 2, self.scale.y / 2)
        z_interval = closed(-self.scale.z / 2, self.scale.z / 2)

        handle_event = SimpleEvent(
            {
                SpatialVariables.x.value: x_interval,
                SpatialVariables.y.value: y_interval,
                SpatialVariables.z.value: z_interval,
            }
        )

        return handle_event

    def create_inner_box_event(self) -> SimpleEvent:
        """
        Return an event used to cut out the inner part of the handle.
        """
        x_interval = closed(0, self.scale.x - self.thickness)
        y_interval = closed(
            -self.scale.y / 2 + self.thickness, self.scale.y / 2 - self.thickness
        )
        z_interval = closed(-self.scale.z, self.scale.z)

        inner_box = SimpleEvent(
            {
                SpatialVariables.x.value: x_interval,
                SpatialVariables.y.value: y_interval,
                SpatialVariables.z.value: z_interval,
            }
        )

        return inner_box


@dataclass
class EntryWayFactory(ViewFactory[T], ABC):
    """
    Abstract factory for creating an entryway with a body.
    """

    scale: Scale = field(default_factory=lambda: Scale(0.03, 1.0, 2.0))
    """
    The scale of the entryway.
    """


@dataclass
class DoorFactory(EntryWayFactory[Door]):
    """
    Factory for creating a door with a handle. The door is defined by its scale and handle direction.
    The doors origin is at the pivot point of the door, not at the center.
    """

    handle_factory: HandleFactory = field(default=None)
    """
    The factory used to create the handle of the door.
    """

    handle_direction: Direction = Direction.Y
    """
    The direction on the door in which the handle positioned.
    """

    def _create(self, world: World) -> World:
        """
        Return a world with a door body at its root. The door has a handle and is defined by its scale and handle direction.
        """

        door_event = self.create_door_event().as_composite_set()

        body = Body(name=self.name)
        bounding_box_collection = BoundingBoxCollection.from_event(body, door_event)
        collision = bounding_box_collection.as_shapes()
        body.collision = collision
        body.visual = collision

        world.add_kinematic_structure_entity(body)

        handle_world = self.handle_factory.create()
        handle_view: Handle = handle_world.get_views_by_type(Handle)[0]
        door_T_handle = self.create_door_T_handle()
        connection_door_T_handle = FixedConnection(
            world.root, handle_world.root, door_T_handle
        )

        world.merge_world(handle_world, connection_door_T_handle)
        world.add_view(Door(name=self.name, handle=handle_view, body=body))

        return world

    def create_door_event(self) -> SimpleEvent:
        """
        Return an event representing a door with a specified scale and handle direction. The origin of the door is not
        at the center of the door, but at the pivot point of the door.
        """

        x_interval = closed(-self.scale.x / 2, self.scale.x / 2)
        y_interval = closed(-self.scale.y / 2, self.scale.y / 2)
        z_interval = closed(-self.scale.z / 2, self.scale.z / 2)

        match self.handle_direction:
            case Direction.X:
                x_interval = closed(0, self.scale.x)
            case Direction.Y:
                y_interval = closed(0, self.scale.y)
            case Direction.Z:
                raise NotImplementedError(
                    f"Door Creation for handle_direction Z is not implemented yet"
                )
            case Direction.NEGATIVE_X:
                x_interval = closed(-self.scale.x, 0)
            case Direction.NEGATIVE_Y:
                y_interval = closed(-self.scale.y, 0)
            case Direction.NEGATIVE_Z:
                raise NotImplementedError(
                    f"Door Creation for handle_direction NEGATIVE_Z is not implemented yet"
                )

        door_event = SimpleEvent(
            {
                SpatialVariables.x.value: x_interval,
                SpatialVariables.y.value: y_interval,
                SpatialVariables.z.value: z_interval,
            }
        )

        return door_event

    def create_door_T_handle(self) -> Optional[TransformationMatrix]:
        """
        Return a transformation matrix that defines the position and orientation of the handle relative to the door.
        :raises: NotImplementedError if the handle direction is Z or NEGATIVE_Z.
        """
        match self.handle_direction:
            case Direction.X:
                return TransformationMatrix.from_xyz_rpy(
                    self.scale.x - 0.1, 0.05, 0, 0, 0, np.pi / 2
                )
            case Direction.Y:
                return TransformationMatrix.from_xyz_rpy(
                    0.05, (self.scale.y - 0.1), 0, 0, 0, 0
                )
            case Direction.Z:
                raise NotImplementedError(
                    f"Handle Creation for handle_direction Z is not implemented yet"
                )
            case Direction.NEGATIVE_X:
                return TransformationMatrix.from_xyz_rpy(
                    -(self.scale.x - 0.1), 0.05, 0, 0, 0, np.pi / 2
                )
            case Direction.NEGATIVE_Y:
                return TransformationMatrix.from_xyz_rpy(
                    0.05, -(self.scale.y - 0.1), 0, 0, 0, 0
                )
            case Direction.NEGATIVE_Z:
                raise NotImplementedError(
                    f"Handle Creation for handle_direction NEGATIVE_Z is not implemented yet"
                )


@dataclass
class DoubleDoorFactory(EntryWayFactory[DoubleDoor]):
    """
    Factory for creating a double door with two doors and their handles.
    """

    handle_factory: HandleFactory = field(default=None)
    """
    The factory used to create the handles of the doors.
    """

    one_door_scale: Scale = field(init=False)
    """
    The scale of a single door, which is half the width of the double door.
    """

    def __post_init__(self):
        """
        Precompute the scale for a single door based on the double door scale.
        """
        self.one_door_scale = Scale(self.scale.x, self.scale.y / 2, self.scale.z)

    def _create(self, world: World) -> World:
        """
        Return a world with a virtual body at its root that is the parent of the two doors making up the double door.
        """
        door_factories = self.create_door_factories()

        double_door_body = Body(name=self.name)
        world.add_kinematic_structure_entity(double_door_body)

        assert len(door_factories) == 2, "Double door must have exactly two doors"

        left_door, right_door = self.add_doors_to_world(
            parent_world=world, door_factories=door_factories
        )

        double_door_view = DoubleDoor(
            body=double_door_body, left_door=left_door, right_door=right_door
        )
        world.add_view(double_door_view)

        return world

    def create_door_factories(self) -> List[DoorFactory]:
        """
        Returns two door factories for the double door, one for handle direction Y, and one for handle direction NEGATIVE_Y.
        Creates one handle for each door.
        """
        handle_directions = [Direction.Y, Direction.NEGATIVE_Y]
        door_factories = []

        for index, direction in enumerate(handle_directions):
            handle_name = PrefixedName(
                self.name.name + f"_{index}_handle", self.name.prefix
            )
            handle_factory = HandleFactory(
                handle_name,
                self.handle_factory.scale,
                self.handle_factory.thickness,
            )

            door_name = PrefixedName(self.name.name + f"_{index}", self.name.prefix)
            door_factory = DoorFactory(
                name=door_name,
                scale=self.one_door_scale,
                handle_factory=handle_factory,
                handle_direction=direction,
            )
            door_factories.append(door_factory)

        return door_factories

    def add_doors_to_world(
        self, parent_world: World, door_factories: List[DoorFactory]
    ) -> tuple[Door, Door]:
        """
        Adds doors to the parent world.
        """
        left_door = None
        right_door = None

        for door_factory in door_factories:
            y_direction: float = self.one_door_scale.y / 2
            if door_factory.handle_direction == Direction.Y:
                y_direction = -y_direction

            parent_T_door = TransformationMatrix.from_point_rotation_matrix(
                Point3(
                    self.one_door_scale.x / 2, y_direction, self.one_door_scale.z / 2
                )
            )

            door = add_door_to_world(
                door_factory=door_factory,
                parent_T_door=parent_T_door,
                parent_world=parent_world,
            )
            if door_factory.handle_direction == Direction.Y:
                right_door = door
            else:
                left_door = door

        assert (left_door is not None) and (right_door is not None)
        return left_door, right_door


@dataclass
class DrawerFactory(ViewFactory[Drawer]):
    """
    Factory for creating a drawer with a handle and a container.
    """

    handle_factory: HandleFactory = field(default=None)
    """
    The factory used to create the handle of the drawer.
    """

    container_factory: ContainerFactory = field(default=None)
    """
    The factory used to create the container of the drawer.
    """

    def _create(self, world: World) -> World:
        """
        Return a world with a drawer at its root. The drawer consists of a container and a handle.
        """
        container_world = self.container_factory.create()
        container_view: Container = container_world.get_views_by_type(Container)[0]

        handle_world = self.handle_factory.create()
        handle_view: Handle = handle_world.get_views_by_type(Handle)[0]

        drawer_T_handle = TransformationMatrix.from_xyz_rpy(
            (self.container_factory.scale.x / 2) + 0.03, 0, 0, 0, 0, 0
        )
        connection_drawer_T_handle = FixedConnection(
            parent=container_world.root,
            child=handle_world.root,
            origin_expression=drawer_T_handle,
        )

        container_world.merge_world(handle_world, connection_drawer_T_handle)
        drawer_view = Drawer(
            name=self.name, container=container_view, handle=handle_view
        )
        container_world.add_view(drawer_view)

        return container_world


@dataclass
class DresserFactory(ViewFactory[Dresser]):
    """
    Factory for creating a dresser with drawers, and doors.
    """

    container_factory: ContainerFactory = field(default=None)
    """
    The factory used to create the container of the dresser.
    """

    drawers_factories: List[DrawerFactory] = field(default_factory=list, hash=False)
    """
    The factories used to create the drawers of the dresser.
    """

    drawer_transforms: List[TransformationMatrix] = field(
        default_factory=list, hash=False
    )
    """
    The transformations for the drawers relative to the dresser container.
    """

    door_factories: List[DoorFactory] = field(default_factory=list, hash=False)
    """
    The factories used to create the doors of the dresser.
    """

    door_transforms: List[TransformationMatrix] = field(
        default_factory=list, hash=False
    )
    """
    The transformations for the doors relative to the dresser container.
    """

    def _create(self, world: World) -> World:
        """
        Return a world with a dresser at its root. The dresser consists of a container, potentially drawers, and doors.
        Assumes that the number of drawers matches the number of drawer transforms.
        """
        assert len(self.drawers_factories) == len(
            self.drawer_transforms
        ), "Number of drawers must match number of transforms"

        dresser_world = self.make_dresser_world()

        return self.make_interior(dresser_world)

    def make_dresser_world(self) -> World:
        """
        Create a world with a dresser view that contains a container, drawers, and doors, but no interior yet.
        """
        dresser_world = self.container_factory.create()
        container_view: Container = dresser_world.get_views_by_type(Container)[0]

        for door_factory, parent_T_door in zip(
            self.door_factories, self.door_transforms
        ):
            add_door_to_world(door_factory, parent_T_door, dresser_world)

        self.add_drawers_to_world(dresser_world)

        dresser_view = Dresser(
            name=self.name,
            container=container_view,
            drawers=[drawer for drawer in dresser_world.get_views_by_type(Drawer)],
            doors=[door for door in dresser_world.get_views_by_type(Door)],
        )
        dresser_world.add_view(dresser_view, exists_ok=True)

        return dresser_world

    def add_drawers_to_world(self, parent_world: World):
        """
        Adds drawers to the parent world. A prismatic connection is created for each drawer.
        """
        for drawer_factory, transform in zip(
            self.drawers_factories, self.drawer_transforms
        ):
            drawer_world = drawer_factory.create()

            drawer_view: Drawer = drawer_world.get_views_by_type(Drawer)[0]
            drawer_body = drawer_view.container.body

            lower_limits, upper_limits = self.create_drawer_upper_lower_limits(
                drawer_factory=drawer_factory
            )

            dof = DegreeOfFreedom(
                name=PrefixedName(
                    f"{drawer_body.name.name}_connection", drawer_body.name.prefix
                ),
                lower_limits=lower_limits,
                upper_limits=upper_limits,
            )

            connection = PrismaticConnection(
                parent=parent_world.root,
                child=drawer_body,
                origin_expression=transform,
                multiplier=1.0,
                offset=0.0,
                axis=Vector3.X(),
                dof=dof,
            )

            parent_world.merge_world(drawer_world, connection)

    def create_drawer_upper_lower_limits(
        self, drawer_factory: DrawerFactory
    ) -> Tuple[DerivativeMap[float], DerivativeMap[float]]:
        """
        Return the upper and lower limits for the drawer's degree of freedom.
        """
        lower_limits = DerivativeMap[float]()
        lower_limits.position = 0.0
        upper_limits = DerivativeMap[float]()
        upper_limits.position = drawer_factory.container_factory.scale.x * 0.75

        return lower_limits, upper_limits

    def make_interior(self, world: World) -> World:
        """
        Create the interior of the dresser by subtracting the drawers and doors from the container, and filling  with
        the remaining space.

        :param world: The world containing the dresser body as its root.
        """
        dresser_body: Body = world.root
        container_event = dresser_body.as_bounding_box_collection_in_frame(
            dresser_body
        ).event

        container_footprint = self.subtract_bodies_from_container_footprint(
            world, container_event
        )

        container_event = self.fill_container_body(container_footprint, container_event)

        collision_shapes = BoundingBoxCollection.from_event(
            dresser_body, container_event
        ).as_shapes()
        dresser_body.collision = collision_shapes
        dresser_body.visual = collision_shapes
        return world

    def subtract_bodies_from_container_footprint(
        self, world: World, container_event: Event
    ) -> Event:
        """
        Subtract the bounding boxes of all bodies in the world from the container event,
        except for the dresser body itself. This creates a frontal footprint of the container

        :param world: The world containing the dresser body as its root.
        :param container_event: The event representing the container.

        :return: An event representing the footprint of the container after subtracting other bodies.
        """
        dresser_body = world.root

        container_footprint = container_event.marginal(SpatialVariables.yz)

        for body in world.bodies:
            if body == dresser_body:
                continue
            body_footprint = body.as_bounding_box_collection_in_frame(
                dresser_body
            ).event.marginal(SpatialVariables.yz)
            container_footprint -= body_footprint

        return container_footprint

    def fill_container_body(
        self, container_footprint: Event, container_event: Event
    ) -> Event:
        """
        Expand container footprint into 3d space and fill the space of the resulting container body.

        :param container_footprint: The footprint of the container in the yz-plane.
        :param container_event: The event representing the container.

        :return: An event representing the container body with the footprint filled in the x-direction.
        """

        container_footprint.fill_missing_variables([SpatialVariables.x.value])

        depth_interval = container_event.bounding_box()[SpatialVariables.x.value]
        limiting_event = SimpleEvent(
            {SpatialVariables.x.value: depth_interval}
        ).as_composite_set()
        limiting_event.fill_missing_variables(SpatialVariables.yz)

        container_event |= container_footprint & limiting_event

        return container_event


@dataclass
class WallFactory(ViewFactory[Wall]):

    scale: Scale
    """
    The scale of the wall.
    """

    door_factories: List[EntryWayFactory] = field(default_factory=list)
    """
    The factories used to create the doors and double doors of the wall.
    """

    door_transforms: List[TransformationMatrix] = field(default_factory=list)
    """
    The transformations for the doors and double doors relative to the wall body.
    """

    def _create(self, world: World) -> World:
        """
        Return a world with the wall body at its root and potentially doors and double doors as children of the wall body.
        """
        wall_world = world
        wall_body = Body(name=self.name)
        wall_collision = self._create_wall_collision(wall_body)
        wall_body.collision = wall_collision
        wall_body.visual = wall_collision
        wall_world.add_kinematic_structure_entity(wall_body)

        self.add_doors_and_double_doors_to_world(wall_world)

        wall = Wall(
            name=self.name,
            body=wall_body,
        )

        wall_world.add_view(wall)

        return wall_world

    def _create_wall_collision(self, reference_frame: Body) -> List[Box]:
        """
        Return the collision shapes for the wall. A wall event is created based on the scale of the wall, and
        doors are removed from the wall event. The resulting bounding box collection is converted to shapes.
        """

        wall_event = self.create_wall_event().as_composite_set()

        wall_event = self.remove_doors_from_wall_event(wall_event)

        bounding_box_collection = BoundingBoxCollection.from_event(
            reference_frame, wall_event
        )

        wall_collision = bounding_box_collection.as_shapes()
        return wall_collision

    def create_wall_event(self) -> SimpleEvent:
        """
        Return a wall event created from its scale. The height origin is on the ground, not in the center of the wall.
        """
        x_interval = closed(-self.scale.x / 2, self.scale.x / 2)
        y_interval = closed(-self.scale.y / 2, self.scale.y / 2)
        z_interval = closed(0, self.scale.z)

        wall_event = SimpleEvent(
            {
                SpatialVariables.x.value: x_interval,
                SpatialVariables.y.value: y_interval,
                SpatialVariables.z.value: z_interval,
            }
        )
        return wall_event

    def remove_doors_from_wall_event(self, wall_event: Event) -> Event:
        """
        Remove doors from the wall event by subtracting the door events from the wall event.
        The doors are created from the door factories and their transforms.
        """
        for door_factory, door_transform in zip(
            self.door_factories, self.door_transforms
        ):
            door_world = door_factory.create()
            doors: List[Door] = door_world.get_views_by_type(Door)
            door_transform = self.get_door_transforms(
                doors, door_factory, door_transform
            )

            temp_world = self.build_temp_world(
                door_world=door_world, door_transform=door_transform
            )

            if isinstance(door_factory, DoorFactory):
                assert door_factory.handle_direction in {
                    Direction.Y,
                    Direction.NEGATIVE_Y,
                }, "Currently only handles are only supported in Y direction"

            door_plane_spatial_variables = SpatialVariables.yz
            door_thickness_spatial_variable = SpatialVariables.x.value

            for door in doors:
                door_event = door.body.as_bounding_box_collection_in_frame(
                    temp_world.root
                ).event
                door_event = door_event.marginal(door_plane_spatial_variables)
                door_event.fill_missing_variables([door_thickness_spatial_variable])

                wall_event -= door_event

        return wall_event

    def get_door_transforms(
        self, doors, door_factory, door_transform
    ) -> TransformationMatrix:
        """
        Calculate the door pivot point based on the door factory and the door transform.
        """
        match door_factory:
            case DoorFactory():
                door_transform = calculate_door_pivot_point(
                    doors[0], door_transform, door_factory.scale
                )
            case DoubleDoorFactory():
                translation = door_transform.to_position().to_np()
                door_transform = TransformationMatrix.from_point_rotation_matrix(
                    Point3(translation[0], translation[1], 0)
                )

        return door_transform

    def build_temp_world(
        self, door_world: World, door_transform: TransformationMatrix
    ) -> World:
        """
        Create a temporary world to merge the door world into the wall world. This temporary world is used to then cut
        out the doors from the wall event.
        """
        temp_world = World()
        with temp_world.modify_world():
            temp_world.add_kinematic_structure_entity(Body())

            connection = FixedConnection(
                parent=temp_world.root,
                child=door_world.root,
                origin_expression=door_transform,
            )

            temp_world.merge_world(door_world, connection)

        return temp_world

    def add_doors_and_double_doors_to_world(self, wall_world: World):
        """
        Adds doors and double doors to the wall world.
        """
        for door_factory, transform in zip(self.door_factories, self.door_transforms):
            match door_factory:
                case DoorFactory():
                    add_door_to_world(door_factory, transform, wall_world)
                case DoubleDoorFactory():
                    door_world = door_factory.create()
                    translation = transform.to_position().to_np()
                    transform = TransformationMatrix.from_point_rotation_matrix(
                        Point3(translation[0], translation[1], 0)
                    )
                    connection = FixedConnection(
                        parent=wall_world.root,
                        child=door_world.root,
                        origin_expression=transform,
                    )

                    wall_world.merge_world(door_world, connection)


def add_door_to_world(
    door_factory: DoorFactory, parent_T_door: TransformationMatrix, parent_world: World
) -> Door:
    """
    Adds a door to the parent world with a revolute connection. The Door's pivot point is on the opposite side of the
    handle.

    :param door_factory: The factory used to create the door.
    :param parent_T_door: The transformation matrix defining the door's position and orientation relative
    to the parent world.
    :param parent_world: The world to which the door will be added.

    :return: The door view that was added to the parent world.
    """
    door_world = door_factory.create()

    door_view: Door = door_world.get_views_by_type(Door)[0]
    door_body = door_view.body

    lower_limits = DerivativeMap[float]()
    upper_limits = DerivativeMap[float]()

    lower_limits.position = -np.pi / 2
    upper_limits.position = 0.0

    if door_factory.handle_direction in {
        Direction.NEGATIVE_X,
        Direction.NEGATIVE_Y,
    }:
        lower_limits.position = 0.0
        upper_limits.position = np.pi / 2

    dof = DegreeOfFreedom(
        name=PrefixedName(f"{door_body.name.name}_connection", door_body.name.prefix),
        lower_limits=lower_limits,
        upper_limits=upper_limits,
    )
    with parent_world.modify_world():
        parent_world.add_degree_of_freedom(dof)

        pivot_point = calculate_door_pivot_point(
            door_view, parent_T_door, door_factory.scale
        )

        connection = RevoluteConnection(
            parent=parent_world.root,
            child=door_body,
            origin_expression=pivot_point,
            multiplier=1.0,
            offset=0.0,
            axis=Vector3.Z(),
            dof=dof,
        )

        parent_world.merge_world(door_world, connection)

        door_view = parent_world.get_views_by_type(Door)[0]

        return door_view


def calculate_door_pivot_point(
    door_view, door_transform: TransformationMatrix, scale: Scale
) -> TransformationMatrix:
    """
    Calculate the door pivot point based on the handle position and the door scale. The pivot point is on the opposite
    side of the handle.

    :param door_view: The door view containing the handle.
    :param door_transform: The transformation matrix defining the door's position and orientation.
    :param scale: The scale of the door.

    :return: The transformation matrix defining the door's pivot point.
    """
    parent_connection = door_view.handle.body.parent_connection
    if parent_connection is None:
        raise ValueError(
            "Handle's body does not have a parent_connection; cannot compute handle_position."
        )
    handle_position: ndarray[float] = (
        parent_connection.origin_expression.to_position().to_np()
    )

    offset = -np.sign(handle_position[1]) * (scale.y / 2)
    door_position = door_transform.to_np()[:3, 3] + np.array([0, offset, 0])

    door_transform = TransformationMatrix.from_point_rotation_matrix(
        Point3(*door_position)
    )

    return door_transform
