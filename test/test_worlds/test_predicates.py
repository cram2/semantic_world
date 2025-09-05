import threading
import time

import pytest
import rclpy

from semantic_world.adapters.viz_marker import VizMarkerPublisher
from semantic_world.world_description.connections import Connection6DoF, FixedConnection
from semantic_world.world_description.geometry import Box, Scale, Color
from semantic_world.reasoning.predicates import (
    contact,
    robot_in_collision,
    get_visible_objects,
)
from semantic_world.datastructures.prefixed_name import PrefixedName
from semantic_world.robots import PR2, Camera
from semantic_world.spatial_types.spatial_types import TransformationMatrix
from semantic_world.testing import pr2_world
from semantic_world.world import World
from semantic_world.world_description.world_entity import Body


@pytest.fixture(scope="session")
def rclpy_node():
    rclpy.init()
    node = rclpy.create_node("test_node")
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_in_contact():
    w = World()

    b1 = Body(name=PrefixedName("b1"))
    collision1 = Box(
        scale=Scale(1.0, 1.0, 1.0),
        origin=TransformationMatrix.from_xyz_rpy(
            0,
            0,
            0.0,
            0,
            0,
            0,
            reference_frame=b1,
        ),
        color=Color(1.0, 0.0, 0.0),
    )
    b1.collision = [collision1]

    b2 = Body(name=PrefixedName("b2"))
    collision2 = Box(
        scale=Scale(1.0, 1.0, 1.0),
        origin=TransformationMatrix.from_xyz_rpy(
            0.9, 0, 0.0, 0, 0, 0, reference_frame=b2
        ),
        color=Color(0.0, 1.0, 0.0),
    )
    b2.collision = [collision2]

    b3 = Body(name=PrefixedName("b3"))
    collision3 = Box(
        scale=Scale(1.0, 1.0, 1.0),
        origin=TransformationMatrix.from_xyz_rpy(
            1.8, 0, 0.0, 0, 0, 0, reference_frame=b3
        ),
        color=Color(0.0, 0.0, 1.0),
    )
    b3.collision = [collision3]

    with w.modify_world():
        w.add_kinematic_structure_entity(b1)
        w.add_kinematic_structure_entity(b2)
        w.add_kinematic_structure_entity(b3)
        w.add_connection(Connection6DoF(b1, b2, _world=w))
        w.add_connection(Connection6DoF(b2, b3, _world=w))
    assert contact(b1, b2)
    assert not contact(b1, b3)
    assert contact(b2, b3)


def test_robot_in_contact(pr2_world: World):
    pr2: PR2 = PR2.from_world(pr2_world)

    body = Body(name=PrefixedName("test_body"))
    collision1 = Box(
        scale=Scale(1.0, 1.0, 1.0),
        origin=TransformationMatrix.from_xyz_rpy(
            0,
            0,
            0.5,
            0,
            0,
            0,
            reference_frame=body,
        ),
        color=Color(1.0, 0.0, 0.0),
    )
    body.collision = [collision1]

    with pr2_world.modify_world():
        pr2_world.add_connection(Connection6DoF(pr2_world.root, body, _world=pr2_world))



    # Ensure the call runs without raising
    assert robot_in_collision(pr2)

    body.parent_connection.origin = TransformationMatrix.from_xyz_rpy(
        4, 0, 0.5, 0, 0, 0, pr2_world.root
    )
    assert not robot_in_collision(pr2)


def test_get_visible_objects(pr2_world: World, rclpy_node):

    pr2: PR2 = PR2.from_world(pr2_world)

    body = Body(name=PrefixedName("test_body"))
    collision1 = Box(
        scale=Scale(1.0, 1.0, 1.0),
        origin=TransformationMatrix.from_xyz_rpy(
            1.,
        1,
            -2.5,
            0,
            0,
            0,
            reference_frame=body,
        ),
        color=Color(1.0, 0.0, 0.0),
    )
    body.collision = [collision1]

    with pr2_world.modify_world():
        pr2_world.add_connection(FixedConnection(pr2_world.root, body, _world=pr2_world))
    viz = VizMarkerPublisher(world=pr2_world, node=rclpy_node)
    camera = pr2_world.get_views_by_type(Camera)[0]
    visible_objects = get_visible_objects(camera)
    assert visible_objects == [body]
    # time.sleep(10)
    viz._stop_publishing()
