import unittest
from copy import deepcopy

import numpy as np
import pytest

from semantic_world.world_description.connections import (
    PrismaticConnection,
    RevoluteConnection,
    Connection6DoF,
    FixedConnection,
)
from semantic_world.exceptions import (
    AddingAnExistingViewError,
    DuplicateViewError,
    ViewNotFoundError, DuplicateKinematicStructureEntityError,
)
from semantic_world.datastructures.prefixed_name import PrefixedName
from semantic_world.spatial_types.derivatives import Derivatives
from semantic_world.spatial_types.math import rotation_matrix_from_rpy
from semantic_world.spatial_types.spatial_types import TransformationMatrix, Point3, RotationMatrix
from semantic_world.spatial_types.symbol_manager import symbol_manager
from semantic_world.testing import world_setup, pr2_world
from semantic_world.world_description.world_entity import View, Body


def test_set_state(world_setup):
    world, l1, l2, bf, r1, r2 = world_setup
    c1: PrismaticConnection = world.get_connection(l1, l2)
    c1.position = 1.0
    assert c1.position == 1.0
    c2: RevoluteConnection = world.get_connection(r1, r2)
    c2.position = 1337
    assert c2.position == 1337
    c3: Connection6DoF = world.get_connection(world.root, bf)
    transform = rotation_matrix_from_rpy(1, 0, 0)
    transform[0, 3] = 69
    c3.origin = transform
    assert np.allclose(world.compute_forward_kinematics_np(world.root, bf), transform)

    world.set_positions_1DOF_connection({c1: 2})
    assert c1.position == 2.0

    transform[0, 3] += c1.position
    assert np.allclose(l2.global_pose.to_np(), transform)


def test_construction(world_setup):
    world, l1, l2, bf, r1, r2 = world_setup
    world.validate()
    assert len(world.connections) == 5
    assert len(world.kinematic_structure_entities) == 6
    assert world.state.positions[0] == 0
    assert (
        world.get_connection(l1, l2).dof.name == world.get_connection(r1, r2).dof.name
    )


def test_chain_of_bodies(world_setup):
    world, _, l2, _, _, _ = world_setup
    result = world.compute_chain_of_kinematic_structure_entities(
        root=world.root, tip=l2
    )
    result = [x.name for x in result]
    assert result == [
        PrefixedName(name="root", prefix="world"),
        PrefixedName(name="bf", prefix=None),
        PrefixedName(name="l1", prefix=None),
        PrefixedName(name="l2", prefix=None),
    ]


def test_chain_of_connections(world_setup):
    world, _, l2, _, _, _ = world_setup
    result = world.compute_chain_of_connections(root=world.root, tip=l2)
    result = [x.name for x in result]
    assert result == [
        PrefixedName(name="root_T_bf", prefix=None),
        PrefixedName(name="bf_T_l1", prefix=None),
        PrefixedName(name="l1_T_l2", prefix=None),
    ]


def test_split_chain_of_bodies(world_setup):
    world, _, l2, _, _, r2 = world_setup
    result = world.compute_split_chain_of_kinematic_structure_entities(root=r2, tip=l2)
    result = tuple([x.name for x in y] for y in result)
    assert result == (
        [PrefixedName(name="r2", prefix=None), PrefixedName(name="r1", prefix=None)],
        [PrefixedName(name="bf", prefix=None)],
        [PrefixedName(name="l1", prefix=None), PrefixedName(name="l2", prefix=None)],
    )


def test_split_chain_of_bodies_adjacent1(world_setup):
    world, _, _, _, r1, r2 = world_setup
    result = world.compute_split_chain_of_kinematic_structure_entities(root=r2, tip=r1)
    result = tuple([x.name for x in y] for y in result)
    assert result == (
        [PrefixedName(name="r2", prefix=None)],
        [PrefixedName(name="r1", prefix=None)],
        [],
    )


def test_split_chain_of_bodies_adjacent2(world_setup):
    world, _, _, _, r1, r2 = world_setup
    result = world.compute_split_chain_of_kinematic_structure_entities(root=r1, tip=r2)
    result = tuple([x.name for x in y] for y in result)
    assert result == (
        [],
        [PrefixedName(name="r1", prefix=None)],
        [PrefixedName(name="r2", prefix=None)],
    )


def test_split_chain_of_bodies_identical(world_setup):
    world, _, _, _, r1, _ = world_setup
    result = world.compute_split_chain_of_kinematic_structure_entities(root=r1, tip=r1)
    result = tuple([x.name for x in y] for y in result)
    assert result == ([], [PrefixedName(name="r1", prefix=None)], [])


def test_split_chain_of_connections(world_setup):
    world, _, l2, _, _, r2 = world_setup
    result = world.compute_split_chain_of_connections(root=r2, tip=l2)
    result = tuple([x.name for x in y] for y in result)
    assert result == (
        [
            PrefixedName(name="r1_T_r2", prefix=None),
            PrefixedName(name="bf_T_r1", prefix=None),
        ],
        [
            PrefixedName(name="bf_T_l1", prefix=None),
            PrefixedName(name="l1_T_l2", prefix=None),
        ],
    )


def test_split_chain_of_connections_adjacent1(world_setup):
    world, _, _, _, r1, r2 = world_setup
    result = world.compute_split_chain_of_connections(root=r2, tip=r1)
    result = tuple([x.name for x in y] for y in result)
    assert result == ([PrefixedName(name="r1_T_r2", prefix=None)], [])


def test_split_chain_of_connections_adjacent2(world_setup):
    world, _, _, _, r1, r2 = world_setup
    result = world.compute_split_chain_of_connections(root=r1, tip=r2)
    result = tuple([x.name for x in y] for y in result)
    assert result == ([], [PrefixedName(name="r1_T_r2", prefix=None)])


def test_split_chain_of_connections_identical(world_setup):
    world, _, _, _, r1, _ = world_setup
    result = world.compute_split_chain_of_connections(root=r1, tip=r1)
    result = tuple([x.name for x in y] for y in result)
    assert result == ([], [])


def test_compute_fk_connection6dof(world_setup):
    world, _, _, bf, _, _ = world_setup
    fk = world.compute_forward_kinematics_np(world.root, bf)
    np.testing.assert_array_equal(fk, np.eye(4))

    connection: Connection6DoF = world.get_connection(world.root, bf)

    world.state[connection.x.name].position = 1.0
    world.state[connection.qw.name].position = 0
    world.state[connection.qz.name].position = 1
    world.notify_state_change()
    fk = world.compute_forward_kinematics_np(world.root, bf)
    np.testing.assert_array_equal(
        fk,
        [
            [-1.0, 0.0, 0.0, 1.0],
            [0.0, -1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
    )


def test_compute_fk(world_setup):
    world, l1, l2, bf, r1, r2 = world_setup
    fk = world.compute_forward_kinematics_np(l2, r2)
    np.testing.assert_array_equal(fk, np.eye(4))

    connection: PrismaticConnection = world.get_connection(r1, r2)

    world.state[connection.dof.name].position = 1.0
    world.notify_state_change()
    fk = world.compute_forward_kinematics_np(l2, r2)
    assert np.allclose(
        fk,
        np.array(
            [
                [0.540302, -0.841471, 0.0, -1.0],
                [0.841471, 0.540302, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        ),
    )


def test_compute_ik(world_setup):
    world, l1, l2, bf, r1, r2 = world_setup
    target = np.array(
        [
            [0.540302, -0.841471, 0.0, -1.0],
            [0.841471, 0.540302, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    joint_state = world.compute_inverse_kinematics(
        l2, r2, TransformationMatrix(target, reference_frame=l2)
    )
    for joint, state in joint_state.items():
        world.state[joint.name].position = state
    world.notify_state_change()
    assert np.allclose(world.compute_forward_kinematics_np(l2, r2), target, atol=1e-3)


def test_compute_fk_expression(world_setup):
    world, l1, l2, bf, r1, r2 = world_setup
    connection: PrismaticConnection = world.get_connection(r1, r2)
    world.state[connection.dof.name].position = 1.0
    world.notify_state_change()
    fk = world.compute_forward_kinematics_np(r2, l2)
    fk_expr = world.compose_forward_kinematics_expression(r2, l2)
    fk_expr_compiled = fk_expr.compile()
    fk2 = fk_expr_compiled.fast_call(
        *symbol_manager.resolve_symbols(fk_expr_compiled.symbol_parameters)
    )
    np.testing.assert_array_almost_equal(fk, fk2)


def test_apply_control_commands(world_setup):
    world, l1, l2, bf, r1, r2 = world_setup
    connection: PrismaticConnection = world.get_connection(r1, r2)
    cmd = np.array([100.0, 0, 0, 0, 0, 0, 0, 0])
    dt = 0.1
    world.apply_control_commands(cmd, dt, Derivatives.jerk)
    assert world.state[connection.dof.name].jerk == 100.0
    assert world.state[connection.dof.name].acceleration == 100.0 * dt
    assert world.state[connection.dof.name].velocity == 100.0 * dt * dt
    assert world.state[connection.dof.name].position == 100.0 * dt * dt * dt


def test_compute_relative_pose(world_setup):
    world, l1, l2, bf, r1, r2 = world_setup
    connection: PrismaticConnection = world.get_connection(l1, l2)
    world.state[connection.dof.name].position = 1.0
    world.notify_state_change()

    pose = TransformationMatrix(reference_frame=l2)
    relative_pose = world.transform(pose, l1)
    expected_pose = TransformationMatrix(
        [
            [1.0, 0, 0.0, 1.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )

    np.testing.assert_array_almost_equal(relative_pose.to_np(), expected_pose.to_np())


def test_compute_relative_pose_both(world_setup):
    world, l1, l2, bf, r1, r2 = world_setup
    world.get_connection(world.root, bf).origin = np.array(
        [
            [0.0, -1.0, 0.0, 1.0],
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    world.notify_state_change()

    pose = TransformationMatrix.from_xyz_rpy(x=1.0, reference_frame=bf)
    relative_pose = world.transform(pose, world.root)
    # Rotation is 90 degrees around z-axis, translation is 1 along x-axis
    expected_pose = np.array(
        [
            [0.0, -1.0, 0.0, 1.0],
            [1.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )

    np.testing.assert_array_almost_equal(relative_pose.to_np(), expected_pose)


def test_compute_relative_pose_only_translation(world_setup):
    world, l1, l2, bf, r1, r2 = world_setup
    connection: PrismaticConnection = world.get_connection(l1, l2)
    world.state[connection.dof.name].position = 1.0
    world.notify_state_change()

    pose = TransformationMatrix.from_xyz_rpy(x=2.0, reference_frame=l2)
    relative_pose = world.transform(pose, l1)
    expected_pose = np.array(
        [
            [1.0, 0, 0.0, 3.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )

    np.testing.assert_array_almost_equal(relative_pose.to_np(), expected_pose)


def test_compute_relative_pose_only_rotation(world_setup):
    world, l1, l2, bf, r1, r2 = world_setup
    connection: RevoluteConnection = world.get_connection(r1, r2)
    world.state[connection.dof.name].position = np.pi / 2  # 90 degrees
    world.notify_state_change()

    pose = TransformationMatrix(reference_frame=r2)
    relative_pose = world.transform(pose, r1)
    expected_pose = np.array(
        [
            [0.0, -1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )

    np.testing.assert_array_almost_equal(relative_pose.to_np(), expected_pose)


def test_add_view(world_setup):
    world, l1, l2, bf, r1, r2 = world_setup
    v = View(name=PrefixedName("muh"))
    world.add_view(v)
    with pytest.raises(AddingAnExistingViewError):
        world.add_view(v, exists_ok=False)
    assert world.get_view_by_name(v.name) == v


def test_duplicate_view(world_setup):
    world, l1, l2, bf, r1, r2 = world_setup
    v = View(name=PrefixedName("muh"))
    world.add_view(v)
    world.views.append(v)
    with pytest.raises(DuplicateViewError):
        world.get_view_by_name(v.name)


def test_merge_world(world_setup, pr2_world):
    world, l1, l2, bf, r1, r2 = world_setup

    base_link = pr2_world.get_kinematic_structure_entity_by_name(
        PrefixedName("base_link")
    )
    r_gripper_tool_frame = pr2_world.get_kinematic_structure_entity_by_name(
        PrefixedName("r_gripper_tool_frame")
    )
    torso_lift_link = pr2_world.get_kinematic_structure_entity_by_name(
        PrefixedName("torso_lift_link")
    )
    r_shoulder_pan_joint = pr2_world.get_connection(
        torso_lift_link,
        pr2_world.get_kinematic_structure_entity_by_name(
            PrefixedName("r_shoulder_pan_link")
        ),
    )

    l_shoulder_pan_joint = pr2_world.get_connection(
        torso_lift_link,
        pr2_world.get_kinematic_structure_entity_by_name(
            PrefixedName("l_shoulder_pan_link")
        ),
    )

    world.merge_world(pr2_world)

    assert base_link in world.kinematic_structure_entities
    assert r_gripper_tool_frame in world.kinematic_structure_entities
    assert l_shoulder_pan_joint in world.connections
    assert torso_lift_link._world == world
    assert r_shoulder_pan_joint._world == world


def test_merge_with_connection(world_setup, pr2_world):
    world, l1, l2, bf, r1, r2 = world_setup

    base_link = pr2_world.get_kinematic_structure_entity_by_name(
        PrefixedName("base_link")
    )
    r_gripper_tool_frame = pr2_world.get_kinematic_structure_entity_by_name(
        PrefixedName("r_gripper_tool_frame")
    )
    torso_lift_link = pr2_world.get_kinematic_structure_entity_by_name(
        PrefixedName("torso_lift_link")
    )
    r_shoulder_pan_joint = pr2_world.get_connection(
        torso_lift_link,
        pr2_world.get_kinematic_structure_entity_by_name(
            PrefixedName("r_shoulder_pan_link")
        ),
    )

    pose = np.eye(4)
    pose[0, 3] = 1.0

    origin = TransformationMatrix(pose)

    new_connection = FixedConnection(
        parent=world.root, child=pr2_world.root, origin_expression=origin, _world=world
    )
    world.merge_world(pr2_world, new_connection)

    assert base_link in world.kinematic_structure_entities
    assert r_gripper_tool_frame in world.kinematic_structure_entities
    assert new_connection in world.connections
    assert torso_lift_link._world == world
    assert r_shoulder_pan_joint._world == world
    assert world.compute_forward_kinematics_np(world.root, base_link)[
        0, 3
    ] == pytest.approx(1.0, abs=1e-6)


def test_merge_with_pose(world_setup, pr2_world):
    world, l1, l2, bf, r1, r2 = world_setup

    base_link = pr2_world.get_kinematic_structure_entity_by_name(
        PrefixedName("base_link")
    )
    r_gripper_tool_frame = pr2_world.get_kinematic_structure_entity_by_name(
        PrefixedName("r_gripper_tool_frame")
    )
    torso_lift_link = pr2_world.get_kinematic_structure_entity_by_name(
        PrefixedName("torso_lift_link")
    )
    r_shoulder_pan_joint = pr2_world.get_connection(
        torso_lift_link,
        pr2_world.get_kinematic_structure_entity_by_name(
            PrefixedName("r_shoulder_pan_link")
        ),
    )

    pose = np.eye(4)
    pose[0, 3] = 1.0  # Translate along x-axis

    world.merge_world_at_pose(pr2_world, TransformationMatrix(pose))

    assert base_link in world.kinematic_structure_entities
    assert r_gripper_tool_frame in world.kinematic_structure_entities
    assert torso_lift_link._world == world
    assert r_shoulder_pan_joint._world == world
    assert world.compute_forward_kinematics_np(world.root, base_link)[
        0, 3
    ] == pytest.approx(1.0, abs=1e-6)


def test_merge_with_pose_rotation(world_setup, pr2_world):
    world, l1, l2, bf, r1, r2 = world_setup

    base_link = pr2_world.get_kinematic_structure_entity_by_name(
        PrefixedName("base_link")
    )
    r_gripper_tool_frame = pr2_world.get_kinematic_structure_entity_by_name(
        PrefixedName("r_gripper_tool_frame")
    )
    torso_lift_link = pr2_world.get_kinematic_structure_entity_by_name(
        PrefixedName("torso_lift_link")
    )
    r_shoulder_pan_joint = pr2_world.get_connection(
        torso_lift_link,
        pr2_world.get_kinematic_structure_entity_by_name(
            PrefixedName("r_shoulder_pan_link")
        ),
    )
    base_footprint = pr2_world.get_kinematic_structure_entity_by_name(
        PrefixedName("base_footprint")
    )

    # Rotation is 90 degrees around z-axis, translation is 1 along x-axis
    pose = np.array(
        [
            [0.0, -1.0, 0.0, 1.0],
            [1.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )

    world.merge_world_at_pose(pr2_world, TransformationMatrix(pose))

    assert base_link in world.kinematic_structure_entities
    assert r_gripper_tool_frame in world.kinematic_structure_entities
    assert torso_lift_link._world == world
    assert r_shoulder_pan_joint._world == world
    fk_base = world.compute_forward_kinematics_np(world.root, base_footprint)
    assert fk_base[0, 3] == pytest.approx(1.0, abs=1e-6)
    assert fk_base[1, 3] == pytest.approx(1.0, abs=1e-6)
    assert fk_base[2, 3] == pytest.approx(0.0, abs=1e-6)
    np.testing.assert_array_almost_equal(
        rotation_matrix_from_rpy(0, 0, np.pi / 2)[:3, :3], fk_base[:3, :3], decimal=6
    )


def test_remove_connection(world_setup):
    world, l1, l2, bf, r1, r2 = world_setup
    connection = world.get_connection(l1, l2)
    with world.modify_world():
        world.remove_connection(connection)
        world.remove_kinematic_structure_entity(l2)
    assert connection not in world.connections
    # dof should still exist because it was a mimic connection.
    assert connection.dof.name in world.state

    with world.modify_world():
        world.remove_connection(world.get_connection(r1, r2))
        new_connection = FixedConnection(r1, r2)
        world.add_connection(new_connection)

    with pytest.raises(ValueError):
        # if you remove a connection, the child must be connected some other way or deleted
        world.remove_connection(world.get_connection(r1, r2))


def test_copy_world(world_setup):
    world, l1, l2, bf, r1, r2 = world_setup
    world_copy = deepcopy(world)
    assert l2 not in world_copy.bodies
    assert bf.parent_connection not in world_copy.connections
    bf.parent_connection.origin = np.array([[1, 0, 0, 1.5],
                                            [0, 1, 0, 0],
                                            [0, 0, 1, 0],
                                            [0, 0, 0, 1]])
    assert float(world_copy.get_kinematic_structure_entity_by_name("bf").global_pose.to_np()[0, 3]) == 0.0
    assert float(bf.global_pose.to_np()[0, 3]) == 1.5

def test_copy_world_state(world_setup):
    world, l1, l2, bf, r1, r2 = world_setup
    connection: PrismaticConnection = world.get_connection(r1, r2)
    world.state[connection.dof.name].position = 1.
    world.notify_state_change()
    world_copy = deepcopy(world)

    assert world.get_connection(r1, r2).position == 1.0
    assert world_copy.get_connection(r1, r2).position == 1.0

def test_match_index(world_setup):
    world, l1, l2, bf, r1, r2 = world_setup
    world_copy = deepcopy(world)
    for body in world.bodies:
        new_body = world_copy.get_kinematic_structure_entity_by_name(body.name)
        assert body.index == new_body.index

def test_copy_dof(world_setup):
    world, l1, l2, bf, r1, r2 = world_setup
    world_copy = deepcopy(world)
    for dof in world.degrees_of_freedom:
        new_dof = world_copy.get_degree_of_freedom_by_name(dof.name)
        assert dof.name == new_dof.name
        assert dof.lower_limits == new_dof.lower_limits
        assert dof.upper_limits == new_dof.upper_limits

def test_copy_pr2_world(pr2_world):
    pr2_world.state[pr2_world.get_degree_of_freedom_by_name("torso_lift_joint").name].position = 0.3
    pr2_world.notify_state_change()
    pr2_copy = deepcopy(pr2_world)


def test_world_different_entities(world_setup):
    world, l1, l2, bf, r1, r2 = world_setup
    world_copy = deepcopy(world)
    for body in world_copy.bodies:
        assert body not in world.bodies
    for connection in world_copy.connections:
        assert connection not in world.connections
    for dof in world_copy.state:
        assert dof not in world.degrees_of_freedom

@unittest.skip("This test still fails because the origin of connections cannot be deepcopied properly.")
def test_copy_pr2(pr2_world):
    pr2_world.state[pr2_world.get_degree_of_freedom_by_name("torso_lift_joint").name].position = 0.3
    pr2_world.notify_state_change()
    pr2_copy = deepcopy(pr2_world)
    assert pr2_world.get_kinematic_structure_entity_by_name("head_tilt_link").global_pose.to_np()[2, 3] == pytest.approx(1.472, abs=1e-3)
    assert pr2_copy.get_kinematic_structure_entity_by_name("head_tilt_link").global_pose.to_np()[2, 3] == pytest.approx(1.472, abs=1e-3)





def test_add_entity_with_duplicate_name(world_setup):
    world, l1, l2, bf, r1, r2 = world_setup
    body_duplicate = Body(name=PrefixedName("l1"))
    with pytest.raises(DuplicateKinematicStructureEntityError):
        with world.modify_world():
            world.add_kinematic_structure_entity(body_duplicate)
