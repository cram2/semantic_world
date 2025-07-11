from typing import Tuple

import numpy as np
import pytest

from semantic_world.connections import PrismaticConnection, RevoluteConnection, Connection6DoF, FixedConnection
from semantic_world.prefixed_name import PrefixedName
from semantic_world.spatial_types.derivatives import Derivatives
from semantic_world.spatial_types.math import rotation_matrix_from_rpy
from semantic_world.spatial_types.symbol_manager import symbol_manager
from semantic_world.world import World, Body


@pytest.fixture
def world_setup() -> Tuple[World, Body, Body, Body, Body, Body]:
    world = World()
    root = Body(PrefixedName(name='root', prefix='world'))
    l1 = Body(PrefixedName('l1'))
    l2 = Body(PrefixedName('l2'))
    bf = Body(PrefixedName('bf'))
    r1 = Body(PrefixedName('r1'))
    r2 = Body(PrefixedName('r2'))

    with world.modify_world():
        [world.add_body(b) for b in [root, l1, l2, bf, r1, r2]]
        dof = world.create_degree_of_freedom(name=PrefixedName('dof'), lower_limits={Derivatives.velocity: -1},
                                             upper_limits={Derivatives.velocity: 1})

        c_l1_l2 = PrismaticConnection(l1, l2, dof=dof, axis=(1, 0, 0))
        c_r1_r2 = RevoluteConnection(r1, r2, dof=dof, axis=(0, 0, 1))
        bf_root_l1 = FixedConnection(bf, l1)
        bf_root_r1 = FixedConnection(bf, r1)
        world.add_connection(c_l1_l2)
        world.add_connection(c_r1_r2)
        world.add_connection(bf_root_l1)
        world.add_connection(bf_root_r1)
        c_root_bf = Connection6DoF(parent=root, child=bf, _world=world)
        world.add_connection(c_root_bf)

    return world, l1, l2, bf, r1, r2


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
    assert np.allclose(l2.global_pose, transform)


def test_construction(world_setup):
    world, l1, l2, bf, r1, r2 = world_setup
    world.validate()
    assert len(world.connections) == 5
    assert len(world.bodies) == 6
    assert world.state.positions[0] == 0
    assert world.get_connection(l1, l2).dof.name == world.get_connection(r1, r2).dof.name


def test_chain_of_bodies(world_setup):
    world, _, l2, _, _, _ = world_setup
    result = world.compute_chain_of_bodies(root=world.root, tip=l2)
    result = [x.name for x in result]
    assert result == [PrefixedName(name='root', prefix='world'), PrefixedName(name='bf', prefix=None),
                      PrefixedName(name='l1', prefix=None), PrefixedName(name='l2', prefix=None)]


def test_chain_of_connections(world_setup):
    world, _, l2, _, _, _ = world_setup
    result = world.compute_chain_of_connections(root=world.root, tip=l2)
    result = [x.name for x in result]
    assert result == [PrefixedName(name='root_T_bf', prefix=None), PrefixedName(name='bf_T_l1', prefix=None),
                      PrefixedName(name='l1_T_l2', prefix=None)]


def test_split_chain_of_bodies(world_setup):
    world, _, l2, _, _, r2 = world_setup
    result = world.compute_split_chain_of_bodies(root=r2, tip=l2)
    result = tuple([x.name for x in y] for y in result)
    assert result == ([PrefixedName(name='r2', prefix=None), PrefixedName(name='r1', prefix=None)],
                      [PrefixedName(name='bf', prefix=None)],
                      [PrefixedName(name='l1', prefix=None), PrefixedName(name='l2', prefix=None)])


def test_split_chain_of_bodies_adjacent1(world_setup):
    world, _, _, _, r1, r2 = world_setup
    result = world.compute_split_chain_of_bodies(root=r2, tip=r1)
    result = tuple([x.name for x in y] for y in result)
    assert result == ([PrefixedName(name='r2', prefix=None)], [PrefixedName(name='r1', prefix=None)], [])


def test_split_chain_of_bodies_adjacent2(world_setup):
    world, _, _, _, r1, r2 = world_setup
    result = world.compute_split_chain_of_bodies(root=r1, tip=r2)
    result = tuple([x.name for x in y] for y in result)
    assert result == ([], [PrefixedName(name='r1', prefix=None)], [PrefixedName(name='r2', prefix=None)])


def test_split_chain_of_bodies_identical(world_setup):
    world, _, _, _, r1, _ = world_setup
    result = world.compute_split_chain_of_bodies(root=r1, tip=r1)
    result = tuple([x.name for x in y] for y in result)
    assert result == ([], [PrefixedName(name='r1', prefix=None)], [])


def test_split_chain_of_connections(world_setup):
    world, _, l2, _, _, r2 = world_setup
    result = world.compute_split_chain_of_connections(root=r2, tip=l2)
    result = tuple([x.name for x in y] for y in result)
    assert result == ([PrefixedName(name='r1_T_r2', prefix=None), PrefixedName(name='bf_T_r1', prefix=None)],
                      [PrefixedName(name='bf_T_l1', prefix=None), PrefixedName(name='l1_T_l2', prefix=None)])


def test_split_chain_of_connections_adjacent1(world_setup):
    world, _, _, _, r1, r2 = world_setup
    result = world.compute_split_chain_of_connections(root=r2, tip=r1)
    result = tuple([x.name for x in y] for y in result)
    assert result == ([PrefixedName(name='r1_T_r2', prefix=None)], [])


def test_split_chain_of_connections_adjacent2(world_setup):
    world, _, _, _, r1, r2 = world_setup
    result = world.compute_split_chain_of_connections(root=r1, tip=r2)
    result = tuple([x.name for x in y] for y in result)
    assert result == ([], [PrefixedName(name='r1_T_r2', prefix=None)])


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

    world.state[connection.x.name].position = 1.
    world.state[connection.qw.name].position = 0
    world.state[connection.qz.name].position = 1
    world.notify_state_change()
    fk = world.compute_forward_kinematics_np(world.root, bf)
    np.testing.assert_array_equal(fk, [[-1., 0., 0., 1.], [0., -1., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]])


def test_compute_fk(world_setup):
    world, l1, l2, bf, r1, r2 = world_setup
    fk = world.compute_forward_kinematics_np(l2, r2)
    np.testing.assert_array_equal(fk, np.eye(4))

    connection: PrismaticConnection = world.get_connection(r1, r2)

    world.state[connection.dof.name].position = 1.
    world.notify_state_change()
    fk = world.compute_forward_kinematics_np(l2, r2)
    assert np.allclose(fk, np.array([[0.540302, -0.841471, 0., -1.],
                                     [0.841471, 0.540302, 0., 0.],
                                     [0., 0., 1., 0.],
                                     [0., 0., 0., 1.]]))


def test_compute_ik(world_setup):
    world, l1, l2, bf, r1, r2 = world_setup
    target = np.array([[0.540302, -0.841471, 0., -1.],
                       [0.841471, 0.540302, 0., 0.],
                       [0., 0., 1., 0.],
                       [0., 0., 0., 1.]])
    joint_state = world.compute_inverse_kinematics(l2, r2, target)
    for joint, state in joint_state.items():
        world.state[joint.name].position = state
    world.notify_state_change()
    assert np.allclose(world.compute_forward_kinematics_np(l2, r2), target, atol=1e-3)


def test_compute_fk_expression(world_setup):
    world, l1, l2, bf, r1, r2 = world_setup
    connection: PrismaticConnection = world.get_connection(r1, r2)
    world.state[connection.dof.name].position = 1.
    world.notify_state_change()
    fk = world.compute_forward_kinematics_np(r2, l2)
    fk_expr = world.compose_forward_kinematics_expression(r2, l2)
    fk_expr_compiled = fk_expr.compile()
    fk2 = fk_expr_compiled.fast_call(*symbol_manager.resolve_symbols(fk_expr_compiled.symbol_parameters))
    np.testing.assert_array_almost_equal(fk, fk2)


def test_apply_control_commands(world_setup):
    world, l1, l2, bf, r1, r2 = world_setup
    connection: PrismaticConnection = world.get_connection(r1, r2)
    cmd = np.array([100., 0, 0, 0, 0, 0, 0, 0])
    dt = 0.1
    world.apply_control_commands(cmd, dt, Derivatives.jerk)
    assert world.state[connection.dof.name].jerk == 100.
    assert world.state[connection.dof.name].acceleration == 100. * dt
    assert world.state[connection.dof.name].velocity == 100. * dt * dt
    assert world.state[connection.dof.name].position == 100. * dt * dt * dt


def test_compute_relative_pose(world_setup):
    world, l1, l2, bf, r1, r2 = world_setup
    connection: PrismaticConnection = world.get_connection(l1, l2)
    world.state[connection.dof.name].position = 1.
    world.notify_state_change()

    pose = np.eye(4)
    relative_pose = world.compute_relative_pose(pose, l1, l2)
    expected_pose = np.array([[1., 0, 0., 1.],
                              [0., 1., 0., 0.],
                              [0., 0., 1., 0.],
                              [0., 0., 0., 1.]])

    np.testing.assert_array_almost_equal(relative_pose, expected_pose)

def test_compute_relative_pose_both(world_setup):
    world, l1, l2, bf, r1, r2 = world_setup
    world.get_connection(world.root, bf).origin = np.array([[0., -1., 0., 1.],
                                                        [1., 0., 0., 0.],
                                                        [0., 0., 1., 0.],
                                                        [0., 0., 0., 1.]])
    world.notify_state_change()

    pose = np.eye(4)
    pose[0, 3] = 1.0  # Translate along x-axis
    relative_pose = world.compute_relative_pose(pose, world.root, bf)
    expected_pose = np.array([[0., -1., 0., 1.],
                              [1., 0., 0., 1.],
                              [0., 0., 1., 0.],
                              [0., 0., 0., 1.]])

    np.testing.assert_array_almost_equal(relative_pose, expected_pose)

def test_compute_relative_pose_only_translation(world_setup):
    world, l1, l2, bf, r1, r2 = world_setup
    connection: PrismaticConnection = world.get_connection(l1, l2)
    world.state[connection.dof.name].position = 1.
    world.notify_state_change()

    pose = np.eye(4)
    pose[0, 3] = 2.0  # Translate along x-axis
    relative_pose = world.compute_relative_pose(pose, l1, l2)
    expected_pose = np.array([[1., 0, 0., 3.],
                              [0., 1., 0., 0.],
                              [0., 0., 1., 0.],
                              [0., 0., 0., 1.]])

    np.testing.assert_array_almost_equal(relative_pose, expected_pose)

def test_compute_relative_pose_only_rotation(world_setup):
    world, l1, l2, bf, r1, r2 = world_setup
    connection: RevoluteConnection = world.get_connection(r1, r2)
    world.state[connection.dof.name].position = np.pi / 2  # 90 degrees
    world.notify_state_change()

    pose = np.eye(4)
    relative_pose = world.compute_relative_pose(pose, r1, r2)
    expected_pose = np.array([[0., -1., 0., 0.],
                              [1., 0., 0., 0.],
                              [0., 0., 1., 0.],
                              [0., 0., 0., 1.]])

    np.testing.assert_array_almost_equal(relative_pose, expected_pose)