from __future__ import annotations

import logging
from dataclasses import dataclass, field
from enum import IntEnum, Enum
from functools import reduce
from operator import or_

from entity_query_language import (
    symbolic_mode,
    let,
    an,
    entity,
    not_,
    in_,
    From,
    merge,
)
from numpy import ndarray
from probabilistic_model.distributions import (
    UnivariateDistribution,
)
from probabilistic_model.distributions.helper import make_dirac
from probabilistic_model.probabilistic_circuit.rx.helper import (
    uniform_measure_of_simple_event,
)
from probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit import (
    ProductUnit,
    leaf,
    ProbabilisticCircuit,
)
from random_events.interval import Bound
from random_events.product_algebra import *
from typing_extensions import Generic, TypeVar

from ..datastructures.prefixed_name import PrefixedName
from ..datastructures.variables import SpatialVariables
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
    Room,
    Floor,
)
from ..world import World
from ..world_description.connections import (
    PrismaticConnection,
    FixedConnection,
    RevoluteConnection,
)
from ..world_description.degree_of_freedom import DegreeOfFreedom
from ..world_description.geometry import Scale
from ..world_description.shape_collection import BoundingBoxCollection, ShapeCollection
from ..world_description.world_entity import Body, Region

id_generator = IDGenerator()


class Direction(IntEnum):
    X = 0
    Y = 1
    Z = 2
    NEGATIVE_X = 3
    NEGATIVE_Y = 4
    NEGATIVE_Z = 5


@dataclass
class HasDoorLikeFactories(ABC):

    door_factories: List[DoorLikeFactory] = field(default_factory=list, hash=False)
    """
    The factories used to create the doors.
    """

    door_transforms: List[TransformationMatrix] = field(
        default_factory=list, hash=False
    )
    """
    The transformations for the doors relative their parent container.
    """

    def _create_door_upper_lower_limits(
        self, door_view: Door
    ) -> Tuple[DerivativeMap[float], DerivativeMap[float]]:
        """
        Return the upper and lower limits for the door's degree of freedom.
        """
        lower_limits = DerivativeMap[float]()
        upper_limits = DerivativeMap[float]()
        lower_limits.position = -np.pi / 2
        upper_limits.position = 0.0

        parent_connection = door_view.handle.body.parent_connection
        door_P_handle: ndarray[float] = (
            parent_connection.origin_expression.to_position().to_np()
        )
        if np.sign(door_P_handle[1]) < 0:
            lower_limits.position = 0.0
            upper_limits.position = np.pi / 2
        return upper_limits, lower_limits

    def _add_hinge_to_door(
        self, door_factory: DoorFactory, parent_T_door: TransformationMatrix
    ):
        """
        Adds a hinge to the door. The hinge's pivot point is on the opposite side of the handle.
        :param door_factory: The factory used to create the door.
        :param parent_T_door: The transformation matrix defining the door's position and orientation relative
        """
        door_world = door_factory.create()
        root = door_world.root
        door_view: Door = door_world.get_views_by_type(Door)[0]

        door_hinge = Body(
            name=PrefixedName(f"{root.name.name}_door_hinge", root.name.prefix)
        )
        parent_T_hinge = self._calculate_door_pivot_point(
            door_view, parent_T_door, door_factory.scale
        )
        hinge_T_door = parent_T_hinge.inverse() @ parent_T_door

        hinge_door_connection = FixedConnection(
            parent=door_hinge, child=root, origin_expression=hinge_T_door
        )
        with door_world.modify_world():
            door_world.add_connection(hinge_door_connection)

        return door_world, parent_T_hinge

    def _add_door_to_world(
        self,
        door_factory: DoorFactory,
        parent_T_door: TransformationMatrix,
        parent_world: World,
    ):
        """
        Adds a door to the parent world using a new door hinge body with a revolute connection.

        :param door_factory: The factory used to create the door.
        :param parent_T_door: The transformation matrix defining the door's position and orientation relative
        to the parent world.
        :param parent_world: The world to which the door will be added.
        """
        with parent_world.modify_world():
            door_world, parent_T_hinge = self._add_hinge_to_door(
                door_factory, parent_T_door
            )
            door_view: Door = door_world.get_views_by_type(Door)[0]
            upper_limits, lower_limits = self._create_door_upper_lower_limits(door_view)

            root = door_world.root
            dof = DegreeOfFreedom(
                name=PrefixedName(f"{root.name.name}_connection", root.name.prefix),
                lower_limits=lower_limits,
                upper_limits=upper_limits,
            )
            connection = RevoluteConnection(
                parent=parent_world.root,
                child=root,
                origin_expression=parent_T_hinge,
                multiplier=1.0,
                offset=0.0,
                axis=Vector3.Z(),
                dof=dof,
            )

            parent_world.merge_world(door_world, connection)

    def add_doorlike_views_to_world(
        self,
        parent_world: World,
    ):
        """
        Adds door-like views to the parent world.
        """
        for door_factory, door_transform in zip(
            self.door_factories, self.door_transforms
        ):
            if isinstance(door_factory, DoorFactory):
                self._add_door_to_world(
                    door_factory=door_factory,
                    parent_T_door=door_transform,
                    parent_world=parent_world,
                )
            elif isinstance(door_factory, DoubleDoorFactory):
                self._add_double_door_to_world(
                    door_factory=door_factory,
                    parent_T_door=door_transform,
                    parent_world=parent_world,
                )

    def _calculate_door_pivot_point(
        self, door_view: Door, parent_T_door: TransformationMatrix, scale: Scale
    ) -> TransformationMatrix:
        """
        Calculate the door pivot point based on the handle position and the door scale. The pivot point is on the opposite
        side of the handle.

        :param door_view: The door view containing the handle.
        :param parent_T_door: The transformation matrix defining the door's position and orientation.
        :param scale: The scale of the door.

        :return: The transformation matrix defining the door's pivot point.
        """
        connection = door_view.handle.body.parent_connection
        door_P_handle: ndarray[float] = (
            connection.origin_expression.to_position().to_np()
        )

        sign = np.sign(door_P_handle[1]) if door_P_handle[1] != 0 else 1
        offset = sign * (scale.y / 2)
        parent_P_hinge = parent_T_door.to_np()[:3, 3] + np.array([0, offset, 0])

        parent_T_hinge = TransformationMatrix.from_point_rotation_matrix(
            Point3(*parent_P_hinge)
        )

        return parent_T_hinge

    def _add_double_door_to_world(
        self,
        door_factory: DoubleDoorFactory,
        parent_T_door: TransformationMatrix,
        parent_world: World,
    ):
        door_world = door_factory.create()
        connection = FixedConnection(
            parent=parent_world.root,
            child=door_world.root,
            origin_expression=parent_T_door,
        )

        parent_world.merge_world(door_world, connection)

    def remove_doors_from_world(
        self, parent_world: World, wall_event_thickness: float = 0.1
    ):
        doors: List[Door] = parent_world.get_views_by_type(Door)
        if not doors:
            return
        all_doors_event = self._build_all_doors_event_from_views(
            doors, wall_event_thickness
        )

        all_bodies_not_door = self._get_all_bodies_excluding_doors_from_world(
            parent_world
        )

        if not all_doors_event.is_empty():
            self._remove_doors_from_bodies(all_bodies_not_door, all_doors_event)

    def _get_all_bodies_excluding_doors_from_world(self, world: World) -> List[Body]:
        with symbolic_mode():
            all_doors = Door(From(world.views))
            door_bodies = merge(all_doors.bodies)
            other_body = let(type_=Body, domain=world.bodies_with_enabled_collision)
            bodies_without_excluded_bodies_query = an(
                entity(other_body, not_(in_(other_body, door_bodies)))
            )

        filtered_bodies = list(bodies_without_excluded_bodies_query.evaluate())
        return filtered_bodies

    def _build_all_doors_event_from_views(
        self, doors: List[Door], wall_event_thickness: float = 0.1
    ) -> Event:
        door_events = [
            self._build_single_door_event(door, wall_event_thickness) for door in doors
        ]
        if door_events:
            return reduce(or_, door_events)
        return Event()

    def _build_single_door_event(
        self, door: Door, wall_event_thickness: float = 0.1
    ) -> Event:
        door_event = door.body.collision.as_bounding_box_collection_in_frame(
            door._world.root
        ).event

        door_plane_spatial_variables = SpatialVariables.yz
        door_thickness_spatial_variable = SpatialVariables.x.value
        door_event = door_event.marginal(door_plane_spatial_variables)
        door_event.fill_missing_variables([door_thickness_spatial_variable])
        thickness_event = SimpleEvent(
            {
                door_thickness_spatial_variable: closed(
                    -wall_event_thickness / 2, wall_event_thickness / 2
                )
            }
        ).as_composite_set()
        thickness_event.fill_missing_variables(door_plane_spatial_variables)
        door_event = door_event & thickness_event

        return door_event

    def _remove_doors_from_bodies(self, bodies: List[Body], all_doors_event: Event):
        for body in bodies:
            self._remove_door_from_body(body, all_doors_event)

    def _remove_door_from_body(self, body: Body, all_doors_event: Event):
        root = body._world.root
        body_event = (
            body.collision.as_bounding_box_collection_in_frame(root).event
            - all_doors_event
        )
        new_collision = BoundingBoxCollection.from_event(root, body_event).as_shapes()
        body.collision = new_collision
        body.visual = new_collision


class SemanticDirection(Enum): ...


class HorizontalSemanticDirection(SemanticDirection): ...


class VerticalSemanticDirection(SemanticDirection): ...


class DirectionDirac(float): ...


class ProabilisticHorizontalDirection(SimpleInterval, HorizontalSemanticDirection):
    LEFT = (0, 1 / 3, Bound.CLOSED, Bound.CLOSED)
    CENTER = (1 / 3, 2 / 3, Bound.OPEN, Bound.OPEN)
    RIGHT = (2 / 3, 1, Bound.CLOSED, Bound.CLOSED)


class ExactHorizontalDirection(DirectionDirac, HorizontalSemanticDirection):
    LEFT = 0
    CENTER = 0.5
    RIGHT = 1


class ProabilisticVerticalDirection(SimpleInterval, VerticalSemanticDirection):
    TOP = (0, 1 / 3, Bound.CLOSED, Bound.CLOSED)
    CENTER = (1 / 3, 2 / 3, Bound.OPEN, Bound.OPEN)
    BOTTOM = (2 / 3, 1, Bound.CLOSED, Bound.CLOSED)


class ExactVerticalDirection(DirectionDirac, VerticalSemanticDirection):
    TOP = 0
    CENTER = 0.5
    BOTTOM = 1


@dataclass
class SemanticPositionDescription:
    """
    Describes a position by mapping semantic concepts (RIGHT, CENTER, LEFT, TOP, BOTTOM) to instances of
    random_events.intervals.SimpleInterval, which are then used to "zoom" into specific regions of an event.
    Each DirectionInterval divides the original event into three parts, either vertically or horizontally, and zooms into
    one of them depending on which specific direction was chosen.
    The sequence of zooms is defined by the order of directions in the horizontal_direction_chain and
    vertical_direction_chain lists.
    Finally, we can sample aa 2d pose from the resulting event
    """

    horizontal_direction_chain: List[HorizontalSemanticDirection]
    """
    Describes the sequence of zooms in the horizontal direction (Y axis).
    """

    vertical_direction_chain: List[VerticalSemanticDirection]
    """
    Describes the sequence of zooms in the vertical direction (Z axis).
    """

    @staticmethod
    def _zoom_interval(base: SimpleInterval, target: SimpleInterval) -> SimpleInterval:
        """
        Zoom 'base' interval by the percentage interval 'target' (0..1),
        preserving the base's boundary styles.
        """
        span = base.upper - base.lower
        new_lower = base.lower + span * target.lower
        new_upper = base.lower + span * target.upper
        return SimpleInterval(new_lower, new_upper, base.left, base.right)

    @staticmethod
    def _zoom_dirac(base: SimpleInterval, target: DirectionDirac) -> float:
        """
        Zoom 'base' interval to the dirac point 'target' (0..1),
        preserving the base's boundary styles.
        """
        span = base.upper - base.lower
        new_point = base.lower + span * target
        return new_point

    def _apply_zoom(
        self, simple_event: SimpleEvent
    ) -> Tuple[SimpleEvent, List[UnivariateDistribution]]:
        """
        Apply zooms in order and return the resulting intervals.
        """

        areas = [
            self._apply_zoom_in_one_direction(
                axis,
                assignment.simple_sets[0],
            )
            for axis, assignment in simple_event.items()
        ]

        simple_events = [area for area in areas if isinstance(area, SimpleEvent)]
        simple_event = reduce(or_, simple_events) if simple_events else SimpleEvent()
        distributions = [
            area for area in areas if isinstance(area, UnivariateDistribution)
        ]

        return simple_event, distributions

    def _build_circuit(
        self, simple_event: SimpleEvent, distributions: List[UnivariateDistribution]
    ) -> ProbabilisticCircuit:

        event_circuit = uniform_measure_of_simple_event(simple_event)
        root: ProductUnit = event_circuit.root
        for distribution in distributions:
            root.add_subcircuit(leaf(distribution, event_circuit))
        return event_circuit

    def _apply_zoom_in_one_direction(
        self, axis: Continuous, current_interval: SimpleInterval
    ) -> Union[SimpleEvent, UnivariateDistribution]:
        current_dirac = None
        if axis == SpatialVariables.y.value:
            directions = self.horizontal_direction_chain
        elif axis == SpatialVariables.z.value:
            directions = self.vertical_direction_chain
        else:
            raise NotImplementedError

        for step in directions:
            match step:
                case SimpleInterval():
                    current_interval = self._zoom_interval(current_interval, step)
                case DirectionDirac():
                    current_dirac = self._zoom_dirac(current_interval, step)
                    current_dirac = make_dirac(axis, float(current_dirac))
                    break

        return current_dirac or SimpleEvent({axis: current_interval})

    def sample_point_from_event(self, event: Event):
        simple_event, distributions = self._apply_zoom(event.bounding_box())
        event_circuit = self._build_circuit(simple_event, distributions)

        return event_circuit.sample(amount=1)[0]


@dataclass
class HasHandleFactory(ABC):
    """
    Mixin for factories receiving a HandleFactory.
    If both parent_T_handle and semantic_position are set, parent_T_handle will be
    prioritized.
    """

    handle_factory: HandleFactory = field(kw_only=True)
    """
    The factory used to create the handle of the door.
    """

    semantic_position: Optional[SemanticPositionDescription] = field(default=None)
    """
    The direction on the door in which the handle positioned.
    """

    parent_T_handle: Optional[TransformationMatrix] = field(default=None)
    """
    The transformation matrix of the handle from the parent
    """

    def __post_init__(self):
        assert (
            self.parent_T_handle is not None or self.semantic_position is not None
        ), "Either parent_T_handle or semantic_position must be set."
        if self.parent_T_handle is not None and self.semantic_position is not None:
            logging.warning(
                f"During the creation of a factory, both parent_T_handle and semantic_position were set. Prioritizing parent_T_handle."
            )

    def create_parent_T_handle_from_parent_scale(
        self, scale: Scale
    ) -> Optional[TransformationMatrix]:
        """
        Return a transformation matrix that defines the position and orientation of the handle relative to its parent.
        :raises: NotImplementedError if the handle direction is Z or NEGATIVE_Z.
        """
        assert self.semantic_position is not None
        sampled_2d_point = self.semantic_position.sample_point_from_event(
            scale.simple_event.as_composite_set().marginal(SpatialVariables.yz)
        )

        return TransformationMatrix.from_xyz_rpy(
            x=0.05, y=sampled_2d_point[0], z=sampled_2d_point[1]
        )

    def add_handle_to_world(
        self,
        parent_T_handle: TransformationMatrix,
        parent_world: World,
    ):
        """
        Adds a handle to the parent world with a fixed connection.

        :param parent_T_handle: The transformation matrix defining the handle's position and orientation relative
        to the parent world.
        :param parent_world: The world to which the handle will be added.
        """
        handle_world = self.handle_factory.create()

        connection = FixedConnection(
            parent=parent_world.root,
            child=handle_world.root,
            origin_expression=parent_T_handle,
        )

        parent_world.merge_world(handle_world, connection)


@dataclass
class HasDrawerFactories(ABC):

    drawers_factories: List[DrawerFactory] = field(default_factory=list, hash=False)
    """
    The factories used to create drawers.
    """

    drawer_transforms: List[TransformationMatrix] = field(
        default_factory=list, hash=False
    )
    """
    The transformations for the drawers their parent container.
    """

    def _add_drawer_to_world(
        self,
        drawer_factory: DrawerFactory,
        parent_T_drawer: TransformationMatrix,
        parent_world: World,
    ):

        lower_limits, upper_limits = self._create_drawer_upper_lower_limits(
            drawer_factory
        )
        drawer_world = drawer_factory.create()
        root = drawer_world.root

        dof = DegreeOfFreedom(
            name=PrefixedName(f"{root.name.name}_connection", root.name.prefix),
            lower_limits=lower_limits,
            upper_limits=upper_limits,
        )

        connection = PrismaticConnection(
            parent=parent_world.root,
            child=root,
            origin_expression=parent_T_drawer,
            multiplier=1.0,
            offset=0.0,
            axis=Vector3.X(),
            dof=dof,
        )

        parent_world.merge_world(drawer_world, connection)

    def add_drawers_to_world(self, parent_world: World):
        """
        Adds drawers to the parent world. A prismatic connection is created for each drawer.
        """

        for drawer_factory, transform in zip(
            self.drawers_factories, self.drawer_transforms
        ):
            self._add_drawer_to_world(
                drawer_factory=drawer_factory,
                parent_T_drawer=transform,
                parent_world=parent_world,
            )

    @staticmethod
    def _create_drawer_upper_lower_limits(
        drawer_factory: DrawerFactory,
    ) -> Tuple[DerivativeMap[float], DerivativeMap[float]]:
        """
        Return the upper and lower limits for the drawer's degree of freedom.
        """
        lower_limits = DerivativeMap[float]()
        upper_limits = DerivativeMap[float]()
        lower_limits.position = 0.0
        upper_limits.position = drawer_factory.container_factory.scale.x * 0.75

        return upper_limits, lower_limits


T = TypeVar("T")


@dataclass
class ViewFactory(Generic[T], ABC):
    """
    Abstract factory for the creation of worlds containing a single view of type T.
    """

    name: PrefixedName = field(kw_only=True)
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
        world = World(name=self.name.name)
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

    direction: Direction = field(default=Direction.X)
    """
    The direction in which the container is open.
    """

    def _create(self, world: World) -> World:
        """
        Return a world with a container body at its root.
        """

        container_event = self._create_container_event()

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

    def _create_container_event(self) -> Event:
        """
        Return an event representing a container with walls of a specified thickness.
        """
        outer_box = self.scale.simple_event
        inner_scale = Scale(
            self.scale.x - self.wall_thickness,
            self.scale.y - self.wall_thickness,
            self.scale.z - self.wall_thickness,
        )
        inner_box = inner_scale.simple_event

        inner_box = self._extend_inner_event_in_direction(
            inner_event=inner_box, inner_scale=inner_scale
        )

        container_event = outer_box.as_composite_set() - inner_box.as_composite_set()

        return container_event

    def _extend_inner_event_in_direction(
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

        handle_event = self._create_handle_event()

        handle = Body(name=self.name)
        collision = BoundingBoxCollection.from_event(handle, handle_event).as_shapes()
        handle.collision = collision
        handle.visual = collision

        handle_view = Handle(name=self.name, body=handle)

        world.add_kinematic_structure_entity(handle)
        world.add_view(handle_view)
        return world

    def _create_handle_event(self) -> Event:
        """
        Return an event representing a handle.
        """

        handle_event = self._create_outer_box_event().as_composite_set()

        inner_box = self._create_inner_box_event().as_composite_set()

        handle_event -= inner_box

        return handle_event

    def _create_outer_box_event(self) -> SimpleEvent:
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

    def _create_inner_box_event(self) -> SimpleEvent:
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
class DoorLikeFactory(ViewFactory[T], ABC):
    """
    Abstract factory for creating door-like factories such as doors or double doors.
    """


@dataclass
class DoorFactory(DoorLikeFactory[Door], HasHandleFactory):
    """
    Factory for creating a door with a handle. The door is defined by its scale and handle direction.
    The doors origin is at the pivot point of the door, not at the center.
    """

    scale: Scale = field(default_factory=lambda: Scale(0.03, 1.0, 2.0))
    """
    The scale of the entryway.
    """

    def _create(self, world: World) -> World:
        """
        Return a world with a door body at its root. The door has a handle and is defined by its scale and handle direction.
        """

        door_event = self.scale.simple_event.as_composite_set()

        body = Body(name=self.name)
        bounding_box_collection = BoundingBoxCollection.from_event(body, door_event)
        collision = bounding_box_collection.as_shapes()
        body.collision = collision
        body.visual = collision

        world.add_kinematic_structure_entity(body)

        door_T_handle = (
            self.parent_T_handle
            or self.create_parent_T_handle_from_parent_scale(self.scale)
        )
        self.add_handle_to_world(door_T_handle, world)
        handle_view: Handle = world.get_views_by_type(Handle)[0]
        world.add_view(Door(name=self.name, handle=handle_view, body=body))

        return world


@dataclass
class DoubleDoorFactory(DoorLikeFactory[DoubleDoor], HasDoorLikeFactories):
    """
    Factory for creating a double door with two doors and their handles.
    """

    def __post_init__(self):
        assert (
            len(self.door_factories) == len(self.door_transforms) == 2
        ), "Double door must have exactly two door factories and transforms"

    def _create(self, world: World) -> World:
        """
        Return a world with a virtual body at its root that is the parent of the two doors making up the double door.
        """

        double_door_body = Body(name=self.name)
        world.add_kinematic_structure_entity(double_door_body)

        self.add_doorlike_views_to_world(
            parent_world=world,
        )

        door_views = world.get_views_by_type(Door)
        assert len(door_views) == 2, "Double door must have exactly two doors views"

        left_door, right_door = door_views
        if (
            left_door.body.parent_connection.origin_expression.y
            > right_door.body.parent_connection.origin_expression.y
        ):
            right_door, left_door = door_views[0], door_views[1]

        double_door_view = DoubleDoor(
            body=double_door_body, left_door=left_door, right_door=right_door
        )
        world.add_view(double_door_view)
        return world


@dataclass
class DrawerFactory(ViewFactory[Drawer], HasHandleFactory):
    """
    Factory for creating a drawer with a handle and a container.
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
        world.merge_world(container_world)

        self.add_handle_to_world(self.parent_T_handle, world)

        container_view: Container = world.get_views_by_type(Container)[0]
        handle_view: Handle = world.get_views_by_type(Handle)[0]
        drawer_view = Drawer(
            name=self.name, container=container_view, handle=handle_view
        )
        world.add_view(drawer_view)

        return world


@dataclass
class DresserFactory(ViewFactory[Dresser], HasDoorLikeFactories, HasDrawerFactories):
    """
    Factory for creating a dresser with drawers, and doors.
    """

    container_factory: ContainerFactory = field(default=None)
    """
    The factory used to create the container of the dresser.
    """

    def _create(self, world: World) -> World:
        """
        Return a world with a dresser at its root. The dresser consists of a container, potentially drawers, and doors.
        Assumes that the number of drawers matches the number of drawer transforms.
        """
        assert len(self.drawers_factories) == len(
            self.drawer_transforms
        ), "Number of drawers must match number of transforms"

        dresser_world = self._make_dresser_world()
        dresser_world.name = world.name
        return self._make_interior(dresser_world)

    def _make_dresser_world(self) -> World:
        """
        Create a world with a dresser view that contains a container, drawers, and doors, but no interior yet.
        """
        dresser_world = self.container_factory.create()
        container_view: Container = dresser_world.get_views_by_type(Container)[0]

        self.add_doorlike_views_to_world(dresser_world)

        self.add_drawers_to_world(dresser_world)

        dresser_view = Dresser(
            name=self.name,
            container=container_view,
            drawers=dresser_world.get_views_by_type(Drawer),
            doors=dresser_world.get_views_by_type(Door),
        )
        dresser_world.add_view(dresser_view, exists_ok=True)
        dresser_world.name = self.name.name

        return dresser_world

    def _make_interior(self, world: World) -> World:
        """
        Create the interior of the dresser by subtracting the drawers and doors from the container, and filling  with
        the remaining space.

        :param world: The world containing the dresser body as its root.
        """
        dresser_body: Body = world.root
        container_event = dresser_body.collision.as_bounding_box_collection_at_origin(
            TransformationMatrix(reference_frame=dresser_body)
        ).event

        container_footprint = self._subtract_bodies_from_container_footprint(
            world, container_event
        )

        container_event = self._fill_container_body(
            container_footprint, container_event
        )

        collision_shapes = BoundingBoxCollection.from_event(
            dresser_body, container_event
        ).as_shapes()
        dresser_body.collision = collision_shapes
        dresser_body.visual = collision_shapes
        return world

    def _subtract_bodies_from_container_footprint(
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
            body_footprint = body.collision.as_bounding_box_collection_at_origin(
                TransformationMatrix(reference_frame=dresser_body)
            ).event.marginal(SpatialVariables.yz)
            container_footprint -= body_footprint

        return container_footprint

    def _fill_container_body(
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
class RoomFactory(ViewFactory[Room]):
    """
    Factory for creating a room with a specific region.
    """

    floor_polytope: List[Point3]
    """
    The region that defines the room's boundaries and reference frame.
    """

    def _create(self, world: World) -> World:
        """
        Return a world with a room view that contains the specified region.
        """
        room_body = Body(name=self.name)
        world.add_kinematic_structure_entity(room_body)

        region = Region.from_3d_points(
            points_3d=self.floor_polytope,
            name=PrefixedName(self.name.name + "_region", self.name.prefix),
            reference_frame=room_body,
        )
        connection = FixedConnection(
            parent=room_body,
            child=region,
            origin_expression=TransformationMatrix(),
        )
        world.add_connection(connection)

        floor = Floor(
            name=PrefixedName(self.name.name + "_floor", self.name.prefix),
            region=region,
        )
        world.add_view(floor)
        room_view = Room(name=self.name, floor=floor)
        world.add_view(room_view)

        return world


@dataclass
class WallFactory(ViewFactory[Wall], HasDoorLikeFactories):

    scale: Scale = field(kw_only=True)
    """
    The scale of the wall.
    """

    def _create(self, world: World) -> World:
        """
        Return a world with the wall body at its root and potentially doors and double doors as children of the wall body.
        """
        wall_world = self._create_wall_world()
        self.add_doorlike_views_to_world(wall_world)
        self.remove_doors_from_world(wall_world)
        world.merge_world(wall_world)

        return world

    def _create_wall_world(self) -> World:
        wall_world = World()
        wall_body = Body(name=self.name)
        wall_collision = self._create_wall_collision(wall_body)
        wall_body.collision = wall_collision
        wall_body.visual = wall_collision
        with wall_world.modify_world():
            wall_world.add_kinematic_structure_entity(wall_body)

        wall = Wall(
            name=self.name,
            body=wall_body,
        )

        wall_world.add_view(wall)

        return wall_world

    def _create_wall_collision(self, reference_frame: Body) -> ShapeCollection:
        """
        Return the collision shapes for the wall. A wall event is created based on the scale of the wall, and
        doors are removed from the wall event. The resulting bounding box collection is converted to shapes.
        """

        wall_event = self._create_wall_event().as_composite_set()

        bounding_box_collection = BoundingBoxCollection.from_event(
            reference_frame, wall_event
        )

        wall_collision = bounding_box_collection.as_shapes()
        return wall_collision

    def _create_wall_event(self) -> SimpleEvent:
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
