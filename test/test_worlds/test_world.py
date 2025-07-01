import os
import pytest
import numpy as np
from networkx.exception import NetworkXNoPath

from semantic_world.adapters.urdf import URDFParser
from semantic_world.connections import PrismaticConnection, RevoluteConnection, Connection6DoF, OmniDrive, \
    FixedConnection
from semantic_world.prefixed_name import PrefixedName
from semantic_world.spatial_types.derivatives import Derivatives
from semantic_world.spatial_types.symbol_manager import symbol_manager
from semantic_world.world import World, Body, Connection


@pytest.fixture
def world_setup():
    world = World()
    root = Body(PrefixedName(name='root', prefix='world'))
    l1 = Body(PrefixedName('l1'))
    l2 = Body(PrefixedName('l2'))
    bf = Body(PrefixedName('bf'))
    r1 = Body(PrefixedName('r1'))
    r2 = Body(PrefixedName('r2'))

    with world.modify_world():
        [world.add_body(b) for b in [root, l1, l2, bf, r1, r2]]
        dof = world.create_degree_of_freedom(name=PrefixedName('dof'),
                                             lower_limits={Derivatives.velocity: -1},
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


def test_construction(world_setup):
    world, _, _, _, _, _ = world_setup
    world.validate()
    assert len(world.connections) == 5
    assert len(world.bodies) == 6
    assert world.state[Derivatives.position, 0] == 0


def test_chain_of_bodies(world_setup):
    world, _, l2, _, _, _ = world_setup
    result = world.compute_chain_of_bodies(root=world.root, tip=l2)
    result = [x.name for x in result]
    assert result == [PrefixedName(name='root', prefix='world'),
                      PrefixedName(name='bf', prefix=None),
                      PrefixedName(name='l1', prefix=None),
                      PrefixedName(name='l2', prefix=None)]


def test_chain_of_connections(world_setup):
    world, _, l2, _, _, _ = world_setup
    result = world.compute_chain_of_connections(root=world.root, tip=l2)
    result = [x.name for x in result]
    assert result == [PrefixedName(name='root_T_bf', prefix=None),
                      PrefixedName(name='bf_T_l1', prefix=None),
                      PrefixedName(name='l1_T_l2', prefix=None)]


def test_split_chain_of_bodies(world_setup):
    world, _, l2, _, _, r2 = world_setup
    result = world.compute_split_chain_of_bodies(root=r2, tip=l2)
    result = tuple([x.name for x in y] for y in result)
    assert result == ([PrefixedName(name='r2', prefix=None),
                       PrefixedName(name='r1', prefix=None)],
                      [PrefixedName(name='bf', prefix=None)],
                      [PrefixedName(name='l1', prefix=None),
                       PrefixedName(name='l2', prefix=None)])


def test_split_chain_of_bodies_adjacent1(world_setup):
    world, _, _, _, r1, r2 = world_setup
    result = world.compute_split_chain_of_bodies(root=r2, tip=r1)
    result = tuple([x.name for x in y] for y in result)
    assert result == ([PrefixedName(name='r2', prefix=None)],
                      [PrefixedName(name='r1', prefix=None)],
                      [])


def test_split_chain_of_bodies_adjacent2(world_setup):
    world, _, _, _, r1, r2 = world_setup
    result = world.compute_split_chain_of_bodies(root=r1, tip=r2)
    result = tuple([x.name for x in y] for y in result)
    assert result == ([],
                      [PrefixedName(name='r1', prefix=None)],
                      [PrefixedName(name='r2', prefix=None)])


def test_split_chain_of_bodies_identical(world_setup):
    world, _, _, _, r1, _ = world_setup
    result = world.compute_split_chain_of_bodies(root=r1, tip=r1)
    result = tuple([x.name for x in y] for y in result)
    assert result == ([],
                      [PrefixedName(name='r1', prefix=None)],
                      [])


def test_split_chain_of_connections(world_setup):
    world, _, l2, _, _, r2 = world_setup
    result = world.compute_split_chain_of_connections(root=r2, tip=l2)
    result = tuple([x.name for x in y] for y in result)
    assert result == ([PrefixedName(name='r1_T_r2', prefix=None),
                       PrefixedName(name='bf_T_r1', prefix=None)],
                      [PrefixedName(name='bf_T_l1', prefix=None),
                       PrefixedName(name='l1_T_l2', prefix=None)])


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

    world.state[Derivatives.position, 1] = 1.
    world.state[Derivatives.position, -1] = 0
    world.state[Derivatives.position, -2] = 1
    world.notify_state_change()
    np.testing.assert_array_equal(fk, [[-1., 0., 0., 1.],
                                       [0., -1., 0., 0.],
                                       [0., 0., 1., 0.],
                                       [0., 0., 0., 1.]])


def test_compute_fk(world_setup):
    world, _, l2, _, _, r2 = world_setup
    fk = world.compute_forward_kinematics_np(l2, r2)
    np.testing.assert_array_equal(fk, np.eye(4))

    world.state[Derivatives.position, 0] = 1.
    world.notify_state_change()
    fk = world.compute_forward_kinematics_np(l2, r2)
    np.testing.assert_array_almost_equal(fk, np.array([[0.540302, -0.841471, 0., -1.],
                                                       [0.841471, 0.540302, 0., 0.],
                                                       [0., 0., 1., 0.],
                                                       [0., 0., 0., 1.]]))


def test_compute_fk_expression(world_setup):
    world, _, l2, _, _, r2 = world_setup
    world.state[Derivatives.position, 0] = 1.
    world.notify_state_change()
    fk = world.compute_forward_kinematics_np(r2, l2)
    fk_expr = world.compose_forward_kinematics_expression(r2, l2)
    fk_expr_compiled = fk_expr.compile()
    fk2 = fk_expr_compiled.fast_call(symbol_manager.resolve_symbols(fk_expr_compiled.symbol_parameters))
    np.testing.assert_array_almost_equal(fk, fk2)


def test_apply_control_commands(world_setup):
    world, _, _, _, _, _ = world_setup
    cmd = np.array([100., 0, 0, 0, 0, 0, 0, 0])
    dt = 0.1
    world.apply_control_commands(cmd, dt, Derivatives.jerk)
    assert world.state[Derivatives.jerk, 0] == 100.
    assert world.state[Derivatives.acceleration, 0] == 100. * dt
    assert world.state[Derivatives.velocity, 0] == 100. * dt * dt
    assert world.state[Derivatives.position, 0] == 100. * dt * dt * dt
