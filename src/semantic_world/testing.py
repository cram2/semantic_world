import os
from typing import Tuple

import pytest

from .adapters.urdf import URDFParser
from .connections import Connection6DoF, PrismaticConnection, RevoluteConnection, FixedConnection, OmniDrive, UnitVector
from .geometry import Box, Scale, Sphere
from .prefixed_name import PrefixedName
from .spatial_types import TransformationMatrix
from .spatial_types.derivatives import Derivatives, DerivativeMap
from .world import World
from .world_entity import Body

@pytest.fixture
def world_setup() -> Tuple[World, Body, Body, Body, Body, Body]:
    world = World()
    root = Body(name=PrefixedName(name='root', prefix='world'))
    l1 = Body(name=PrefixedName('l1'))
    l2 = Body(name=PrefixedName('l2'))
    bf = Body(name=PrefixedName('bf'))
    r1 = Body(name=PrefixedName('r1'))
    r2 = Body(name=PrefixedName('r2'))

    with world.modify_world():
        [world.add_body(b) for b in [root, l1, l2, bf, r1, r2]]
        lower_limits = DerivativeMap()
        lower_limits.velocity = -1
        upper_limits = DerivativeMap()
        upper_limits.velocity = 1
        dof = world.create_degree_of_freedom(name=PrefixedName('dof'), lower_limits=lower_limits,
                                             upper_limits=upper_limits)

        c_l1_l2 = PrismaticConnection(parent=l1, child=l2, dof=dof, axis=UnitVector(1, 0, 0))
        c_r1_r2 = RevoluteConnection(parent=r1, child=r2, dof=dof, axis=UnitVector(0, 0, 1))
        bf_root_l1 = FixedConnection(parent=bf, child=l1)
        bf_root_r1 = FixedConnection(parent=bf, child=r1)
        world.add_connection(c_l1_l2)
        world.add_connection(c_r1_r2)
        world.add_connection(bf_root_l1)
        world.add_connection(bf_root_r1)
        c_root_bf = Connection6DoF(parent=root, child=bf, _world=world)
        world.add_connection(c_root_bf)

    return world, l1, l2, bf, r1, r2

@pytest.fixture
def world_setup_simple():
    world = World()
    root = Body(name=PrefixedName(name='root', prefix='world'))
    body1 = Body(name=PrefixedName('name1', prefix='test'),
                 collision=[Box(origin=TransformationMatrix.from_xyz_rpy(), scale=Scale(0.25, 0.25, 0.25))])
    body2 = Body(name=PrefixedName('name2', prefix='test'),
                 collision=[Box(origin=TransformationMatrix.from_xyz_rpy(), scale=Scale(0.25, 0.25, 0.25))])
    body3 = Body(name=PrefixedName('name3', prefix='test'),
                 collision=[Sphere(origin=TransformationMatrix.from_xyz_rpy(), radius=0.01)])
    body4 = Body(name=PrefixedName('name4', prefix='test'),
                 collision=[Sphere(origin=TransformationMatrix.from_xyz_rpy(), radius=0.01)])

    with world.modify_world():
        world.add_body(body1)
        world.add_body(body2)
        world.add_body(body3)
        world.add_body(body4)

        c_root_body1 = Connection6DoF(parent=root, child=body1, _world=world)
        c_root_body2 = Connection6DoF(parent=root, child=body2, _world=world)
        c_root_body3 = Connection6DoF(parent=root, child=body3, _world=world)
        c_root_body4 = Connection6DoF(parent=root, child=body4, _world=world)

        world.add_connection(c_root_body1)
        world.add_connection(c_root_body2)
        world.add_connection(c_root_body3)
        world.add_connection(c_root_body4)
    return world, body1, body2, body3, body4


@pytest.fixture
def two_arm_robot_world():
    urdf_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "resources", "urdf")
    robot = os.path.join(urdf_dir, "simple_two_arm_robot.urdf")
    world = World()
    with world.modify_world():
        localization_body = Body(name=PrefixedName('odom_combined'))
        world.add_body(localization_body)

        robot_parser = URDFParser(robot)
        world_with_robot = robot_parser.parse()
        # world_with_pr2.plot_kinematic_structure()
        root = world_with_robot.root
        c_root_bf = OmniDrive(parent=localization_body, child=root, _world=world)
        world.merge_world(world_with_robot, root_connection=c_root_bf)
    return world

@pytest.fixture
def pr2_world():
    urdf_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "resources", "urdf")
    pr2 = os.path.join(urdf_dir, "pr2_kinematic_tree.urdf")
    world = World()
    with world.modify_world():
        localization_body = Body(name=PrefixedName('odom_combined'))
        world.add_body(localization_body)

        pr2_parser = URDFParser(pr2)
        world_with_pr2 = pr2_parser.parse()
        # world_with_pr2.plot_kinematic_structure()
        pr2_root = world_with_pr2.root
        c_root_bf = OmniDrive(parent=localization_body, child=pr2_root, _world=world)
        world.merge_world(world_with_pr2, c_root_bf)

    return world

