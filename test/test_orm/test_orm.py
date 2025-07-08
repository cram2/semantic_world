import logging
import sys
import unittest

import sqlalchemy
from sqlalchemy import create_engine, select, text
from sqlalchemy.orm import Session

from semantic_world.geometry import Shape, Box, Scale, Color
from semantic_world.prefixed_name import PrefixedName
from semantic_world.spatial_types.spatial_types import TransformationMatrix
from semantic_world.world import Body
from semantic_world.orm.ormatic_interface import *
import ormatic.dao


class ORMTest(unittest.TestCase):
    engine: sqlalchemy.engine
    session: Session

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        handler = logging.StreamHandler(sys.stdout)
        handler.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))
        ormatic.dao.logger.addHandler(handler)
        ormatic.dao.logger.setLevel(logging.DEBUG)
        cls.engine = create_engine('sqlite:///:memory:')

    def setUp(self):
        super().setUp()
        self.session = Session(self.engine)
        Base.metadata.create_all(bind=self.session.bind)

    def tearDown(self):
        super().tearDown()
        Base.metadata.drop_all(self.session.bind)
        self.session.close()

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

        body_dao = BodyDAO.to_dao(b1)
        self.session.add(body_dao)
        self.session.commit()
        result = self.session.scalar(select(ShapeDAO))
        self.assertIsInstance(result, BoxDAO)