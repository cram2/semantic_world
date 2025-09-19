import threading

import numpy as np
import pytest
import rclpy

from semantic_world.datastructures.prefixed_name import PrefixedName
from semantic_world.reasoning.predicates import (
    contact,
    robot_in_collision,
    visible,
    above,
    below,
    left_of,
    right_of,
    behind,
    in_front_of,
    is_body_in_region,
    occluding_bodies,
    is_supported_by,
    _center_of_mass_in_world,
    is_body_in_gripper,
    robot_holds_body,
    reachable,
    blocking,
)
from semantic_world.robots import PR2, Camera, ParallelGripper
from semantic_world.spatial_types.spatial_types import TransformationMatrix
from semantic_world.testing import pr2_world
from semantic_world.world import World
from semantic_world.world_description.connections import Connection6DoF, FixedConnection
from semantic_world.world_description.geometry import Box, Scale, Color
from semantic_world.world_description.shape_collection import ShapeCollection
from semantic_world.world_description.world_entity import Body, Region


@pytest.fixture(scope="session")
def two_block_world():
    def make_body(name: str) -> Body:
        result = Body(name=PrefixedName(name))
        collision = Box(
            scale=Scale(1.0, 1.0, 1.0),
            origin=TransformationMatrix.from_xyz_rpy(reference_frame=result),
        )
        result.collision = ShapeCollection([collision], reference_frame=result)
        return result

    world = World()

    body_1 = make_body("body_1")
    body_2 = make_body("body_2")

    with world.modify_world():
        connection = FixedConnection(
            parent=body_1,
            child=body_2,
            _world=world,
            origin_expression=TransformationMatrix.from_xyz_rpy(
                z=3, reference_frame=body_1
            ),
        )
        world.add_connection(connection)
    return body_1, body_2


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
    b1.collision = ShapeCollection([collision1])

    b2 = Body(name=PrefixedName("b2"))
    collision2 = Box(
        scale=Scale(1.0, 1.0, 1.0),
        origin=TransformationMatrix.from_xyz_rpy(
            0.9, 0, 0.0, 0, 0, 0, reference_frame=b2
        ),
        color=Color(0.0, 1.0, 0.0),
    )
    b2.collision = ShapeCollection([collision2])

    b3 = Body(name=PrefixedName("b3"))
    collision3 = Box(
        scale=Scale(1.0, 1.0, 1.0),
        origin=TransformationMatrix.from_xyz_rpy(
            1.8, 0, 0.0, 0, 0, 0, reference_frame=b3
        ),
        color=Color(0.0, 0.0, 1.0),
    )
    b3.collision = ShapeCollection([collision3])

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
            z=0.5,
            reference_frame=body,
        ),
        color=Color(1.0, 0.0, 0.0),
    )
    body.collision = ShapeCollection([collision1])

    with pr2_world.modify_world():
        pr2_world.add_connection(Connection6DoF(pr2_world.root, body, _world=pr2_world))

    # Ensure the call runs without raising
    assert robot_in_collision(pr2)

    body.parent_connection.origin = TransformationMatrix.from_xyz_rpy(
        4, 0, 0.5, 0, 0, 0, pr2_world.root
    )
    assert not robot_in_collision(pr2)


def test_get_visible_objects(pr2_world: World):

    pr2: PR2 = PR2.from_world(pr2_world)

    body = Body(name=PrefixedName("test_body"))
    collision1 = Box(
        scale=Scale(1.0, 1.0, 1.0),
        origin=TransformationMatrix.from_xyz_rpy(
            x=2.0,
            z=1.0,
            reference_frame=body,
        ),
        color=Color(1.0, 0.0, 0.0),
    )
    body.collision = ShapeCollection([collision1])

    with pr2_world.modify_world():
        pr2_world.add_connection(Connection6DoF(pr2_world.root, body, _world=pr2_world))

    camera = pr2_world.get_views_by_type(Camera)[0]

    assert visible(camera, body)


def test_occluding_bodies(pr2_world: World):
    pr2: PR2 = PR2.from_world(pr2_world)

    def make_body(name: str) -> Body:
        result = Body(name=PrefixedName(name))
        collision = Box(
            scale=Scale(1.0, 1.0, 1.0),
            origin=TransformationMatrix.from_xyz_rpy(reference_frame=result),
        )
        result.collision = ShapeCollection([collision])
        return result

    obstacle = make_body("obstacle")
    occluded_body = make_body("occluded_body")

    with pr2_world.modify_world():
        root = pr2_world.root
        c1 = FixedConnection(
            parent=root,
            child=obstacle,
            _world=pr2_world,
            origin_expression=TransformationMatrix.from_xyz_rpy(
                reference_frame=root, x=3, z=0.8
            ),
        )
        c2 = FixedConnection(
            parent=root,
            child=occluded_body,
            _world=pr2_world,
            origin_expression=TransformationMatrix.from_xyz_rpy(
                reference_frame=root, x=10, z=0.5
            ),
        )
        pr2_world.add_connection(c1)
        pr2_world.add_connection(c2)

    camera = pr2_world.get_views_by_type(Camera)[0]

    bodies = occluding_bodies(camera, occluded_body)
    assert obstacle in bodies
    assert camera not in bodies
    assert occluded_body not in bodies


def test_above_and_below(two_block_world):
    center, top = two_block_world

    pov = TransformationMatrix.from_xyz_rpy(x=-3)
    assert above(top, center, pov)
    assert below(center, top, pov)

    pov = TransformationMatrix.from_xyz_rpy(x=3, yaw=np.pi)
    assert above(top, center, pov)
    assert below(center, top, pov)

    pov = TransformationMatrix.from_xyz_rpy(x=3, roll=np.pi)
    assert above(center, top, pov)
    assert below(top, center, pov)


def test_left_and_right(two_block_world):
    center, top = two_block_world

    pov = TransformationMatrix.from_xyz_rpy(x=3, roll=np.pi / 2)
    assert left_of(top, center, pov)
    assert right_of(center, top, pov)

    pov = TransformationMatrix.from_xyz_rpy(x=3, roll=-np.pi / 2)
    assert right_of(top, center, pov)
    assert left_of(center, top, pov)


def test_behind_and_in_front_of(two_block_world):
    center, top = two_block_world

    pov = TransformationMatrix.from_xyz_rpy(z=-5, pitch=np.pi / 2)
    assert behind(top, center, pov)
    assert in_front_of(center, top, pov)

    pov = TransformationMatrix.from_xyz_rpy(z=5, pitch=-np.pi / 2)
    assert in_front_of(top, center, pov)
    assert behind(center, top, pov)


def test_body_in_region(two_block_world):
    center, top = two_block_world
    region = Region(name=PrefixedName("test_region"))
    region_box = Box(
        scale=Scale(1.0, 1.0, 1.0),
        origin=TransformationMatrix.from_xyz_rpy(reference_frame=region),
    )
    region.area = ShapeCollection([region_box])

    with center._world.modify_world():
        connection = FixedConnection(
            parent=center,
            child=region,
            _world=center._world,
            origin_expression=TransformationMatrix.from_xyz_rpy(
                z=0.5, reference_frame=center
            ),
        )
        center._world.add_connection(connection)
    assert is_body_in_region(center, region) == 0.5
    assert is_body_in_region(top, region) == 0.0


def test_supporting(two_block_world):
    center, top = two_block_world

    with center._world.modify_world():
        top.parent_connection.origin_expression = TransformationMatrix.from_xyz_rpy(
            reference_frame=center, z=1.0
        )
    assert is_supported_by(top, center)
    assert not is_supported_by(center, top)


def test_is_body_in_gripper(
    pr2_world,
):
    pr2: PR2 = PR2.from_world(pr2_world)

    gripper = pr2_world.get_views_by_type(ParallelGripper)

    left_gripper = (
        gripper[0]
        if left_of(gripper[0].root, gripper[1].root, pr2.root.global_pose)
        else gripper[1]
    )

    # Create test box between fingers
    test_box = Body(name=PrefixedName("test_box"))
    box_collision = Box(
        scale=Scale(0.05, 0.01, 0.05),
        origin=TransformationMatrix.from_xyz_rpy(reference_frame=test_box),
        color=Color(1.0, 0.0, 0.0),
    )
    test_box.collision = ShapeCollection([box_collision])

    # Calculate position between fingers
    finger1_pos = _center_of_mass_in_world(left_gripper.finger.tip)
    finger2_pos = _center_of_mass_in_world(left_gripper.thumb.tip)
    between_fingers = (finger1_pos + finger2_pos) / 2.0

    # Add box to world
    with pr2_world.modify_world():
        root = pr2_world.root
        connection = Connection6DoF(
            parent=root,
            child=test_box,
            _world=pr2_world,
        )
        pr2_world.add_connection(connection)
        connection.origin = TransformationMatrix.from_xyz_rpy(
            x=between_fingers[0],
            y=between_fingers[1],
            z=between_fingers[2],
            reference_frame=root,
        )

    assert is_body_in_gripper(test_box, left_gripper) > 0
    assert robot_holds_body(pr2, test_box)
    connection.origin = TransformationMatrix()
    assert is_body_in_gripper(test_box, left_gripper) == 0


def test_reachable(pr2_world):
    pr2: PR2 = PR2.from_world(pr2_world)

    tool_frame_T_reachable_goal = TransformationMatrix.from_xyz_rpy(
        x=-0.2,
        y=0.3,
        reference_frame=pr2.left_arm.manipulator.tool_frame,
    )

    assert reachable(
        tool_frame_T_reachable_goal,
        pr2.left_arm.root,
        pr2.left_arm.manipulator.tool_frame,
    )
    assert not blocking(
        tool_frame_T_reachable_goal,
        pr2.left_arm.root,
        pr2.left_arm.manipulator.tool_frame,
    )
    tool_frame_T_unreachable_goal = TransformationMatrix.from_xyz_rpy(
        x=10, y=10, reference_frame=pr2.left_arm.manipulator.tool_frame
    )
    assert not reachable(
        tool_frame_T_unreachable_goal,
        pr2.left_arm.root,
        pr2.left_arm.manipulator.tool_frame,
    )

    tool_frame_T_rotated_reachable_goal = TransformationMatrix.from_xyz_rpy(
        x=-0.2,
        y=0.3,
        yaw=np.pi / 2,
        reference_frame=pr2.left_arm.manipulator.tool_frame,
    )
    assert reachable(
        tool_frame_T_rotated_reachable_goal,
        pr2.left_arm.root,
        pr2.left_arm.manipulator.tool_frame,
    )

    tool_frame_T_rotated_unreachable_goal = TransformationMatrix.from_xyz_rpy(
        x=-0.2,
        y=0.3,
        yaw=-np.pi / 2,
        reference_frame=pr2.left_arm.manipulator.tool_frame,
    )
    assert not reachable(
        tool_frame_T_rotated_unreachable_goal,
        pr2.left_arm.root,
        pr2.left_arm.manipulator.tool_frame,
    )


def test_blocking(pr2_world):

    pr2: PR2 = PR2.from_world(pr2_world)

    obstacle = Body(name=PrefixedName("obstacle"))
    collision = Box(
        scale=Scale(3.0, 1.0, 1.0),
        origin=TransformationMatrix.from_xyz_rpy(x=1.0, z=0.5),
    )
    obstacle.collision = ShapeCollection([collision])

    with pr2_world.modify_world():
        new_root = Body(name=PrefixedName("new_root"))
        pr2_world.add_connection(
            Connection6DoF(new_root, pr2_world.root, _world=pr2_world)
        )
        pr2_world.add_connection(
            Connection6DoF(
                parent=new_root,
                child=obstacle,
                _world=pr2_world,
            )
        )

    assert obstacle not in pr2.bodies
    assert robot_in_collision(pr2)

    tool_frame_T_reachable_goal = TransformationMatrix.from_xyz_rpy(
        x=-0.2,
        y=0.3,
        reference_frame=pr2.left_arm.manipulator.tool_frame,
    )
    assert blocking(
        tool_frame_T_reachable_goal,
        pr2.left_arm.root,
        pr2.left_arm.manipulator.tool_frame,
    )
