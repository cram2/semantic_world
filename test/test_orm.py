import unittest

import sqlalchemy
from sqlacodegen.generators import Base
from sqlalchemy import create_engine
from sqlalchemy.orm import Session

from semantic_world.orm.model import Vector3Type, Vector3
from semantic_world.orm.ormatic_interface import mapper_registry

from semantic_world.prefixed_name import PrefixedName

class ORMTest(unittest.TestCase):

    engine: sqlalchemy.engine
    session: Session

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.engine = create_engine('sqlite:///:memory:')

    def setUp(self):
        super().setUp()
        self.mapper_registry = mapper_registry
        self.session = Session(self.engine)
        self.mapper_registry.metadata.create_all(bind=self.session.bind)

    def tearDown(self):
        super().tearDown()
        self.mapper_registry.metadata.drop_all(self.session.bind)
        self.session.close()

    def test_vector3(self):
        v = Vector3.from_xyz(1, 2, 3, PrefixedName("vector", "orm"))
        print(v)