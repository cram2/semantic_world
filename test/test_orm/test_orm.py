# import unittest
#
# import sqlalchemy
# from sqlacodegen.generators import Base
# from sqlalchemy import create_engine, select, text, event
# from sqlalchemy.orm import Session
#
# from semantic_world.geometry import Shape, Box, Scale, Color
# from semantic_world.orm.model import Vector3
# from semantic_world.orm.ormatic_interface import mapper_registry
#
# from semantic_world.prefixed_name import PrefixedName
# from semantic_world.spatial_types.spatial_types import TransformationMatrix
# from semantic_world.world import World, Body, Connection
#
#
# class ORMTest(unittest.TestCase):
#
#     engine: sqlalchemy.engine
#     session: Session
#
#     @classmethod
#     def setUpClass(cls):
#         super().setUpClass()
#         cls.engine = create_engine('sqlite:///:memory:')
#
#     def setUp(self):
#         super().setUp()
#         self.mapper_registry = mapper_registry
#         self.session = Session(self.engine)
#         self.mapper_registry.metadata.create_all(bind=self.session.bind)
#
#     def tearDown(self):
#         super().tearDown()
#         self.mapper_registry.metadata.drop_all(self.session.bind)
#         self.session.close()
#
#     def test_insert(self):
#
#         reference_frame = PrefixedName("reference_frame", "world")
#         child_frame = PrefixedName("child_frame", "world")
#         origin = TransformationMatrix.from_xyz_rpy(1,2,3,1,2,3, reference_frame=reference_frame,
#                                                    child_frame=child_frame)
#         scale = Scale(1., 1. ,1.)
#         color = Color(0., 1., 1.)
#         shape1 = Box(origin=origin, scale=scale, color=color)
#         b1 = Body(
#             PrefixedName("b1"),
#             collision=[shape1]
#         )
#
#         self.session.add(b1)
#         self.session.commit()
#         raw =  list(self.session.execute(text("select * from Shape limit 1")))
#         self.assertEqual(len(raw[0]), 5)
#         result = self.session.scalar(select(Shape))
#         self.assertIsInstance(result, Box)
#         self.assertEqual(shape1, result)
#         self.assertIsInstance(result.origin, TransformationMatrix)
#         self.assertEqual(shape1.origin, result.origin)
#
