import logging
import os
import sys
import unittest

import sqlalchemy
from sqlalchemy import create_engine, select, text
from sqlalchemy.orm import Session

from semantic_world.adapters.urdf import URDFParser
from semantic_world.geometry import Shape, Box, Scale, Color
from semantic_world.orm.model import WorldMapping
from semantic_world.prefixed_name import PrefixedName
from semantic_world.spatial_types.spatial_types import TransformationMatrix
from semantic_world.world import Body
from semantic_world.orm.ormatic_interface import *
from ormatic.dao import to_dao


class ORMTest(unittest.TestCase):
    engine: sqlalchemy.engine
    session: Session

    urdf_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "resources", "urdf")
    table = os.path.join(urdf_dir, "table.urdf")

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.engine = create_engine('sqlite:///:memory:')
        cls.table_world = URDFParser(cls.table).parse()

    def setUp(self):
        super().setUp()
        self.session = Session(self.engine)
        Base.metadata.create_all(bind=self.session.bind)

    def tearDown(self):
        super().tearDown()
        Base.metadata.drop_all(self.session.bind)
        self.session.close()

    def test_table_world(self):
        world_dao: WorldMappingDAO = to_dao(self.table_world)

        for body in world_dao.bodies:
            for collision in body.collision:
                print(collision.origin)

        self.session.add(world_dao)
        self.session.commit()

        bodies_from_db = self.session.scalars(select(BodyDAO)).all()
        self.assertEqual(len(bodies_from_db), len(self.table_world.bodies))

        connections_from_db = self.session.scalars(select(ConnectionDAO)).all()
        self.assertEqual(len(connections_from_db), len(self.table_world.connections))

    def test_insert(self):
        reference_frame = PrefixedName("reference_frame", "world")
        child_frame = PrefixedName("child_frame", "world")
        origin = TransformationMatrix.from_xyz_rpy(1, 2, 3, 1, 2, 3, reference_frame=reference_frame,
                                                   child_frame=child_frame)
        scale = Scale(1., 1., 1.)
        color = Color(0., 1., 1.)
        shape1 = Box(origin=origin, scale=scale, color=color)
        b1 = Body(
            PrefixedName("b1"),
            collision=[shape1]
        )

        dao: BodyDAO = to_dao(b1)

        self.session.add(dao)
        self.session.commit()
        result = self.session.scalar(select(ShapeDAO))
        self.assertIsInstance(result, BoxDAO)