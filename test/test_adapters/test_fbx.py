import os
import unittest
from unittest import skipUnless

from ormatic.dao import to_dao
from sqlalchemy import create_engine, select
from sqlalchemy.orm import Session

from semantic_world.adapters.fbx import FBXParser
from semantic_world.orm.ormatic_interface import *
from semantic_world.orm.utils import persistent_database_available
from semantic_world.utils import rclpy_installed, bpy_installed
from semantic_world.world import World


@skipUnless(persistent_database_available(), "persistent database required")
class FBXParserTest(unittest.TestCase):
    fbx_path = os.path.join(
        os.path.dirname(__file__), "..", "..", "resources", "fbx", "shelves_group.fbx"
    )

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        semantic_world_database_uri = os.environ.get("SEMANTIC_WORLD_DATABASE_URI")
        cls.engine = create_engine(f"mysql+pymysql://{semantic_world_database_uri}")

    def setUp(self):
        super().setUp()
        self.session = Session(self.engine)
        Base.metadata.create_all(bind=self.session.bind)

    @skipUnless(bpy_installed(), "Requires bpy")
    def test_parse_fbx(self):
        parser = FBXParser(self.fbx_path)
        worlds = parser.parse()

        daos = [to_dao(world) for world in worlds]
        self.session.add_all(daos)
        self.session.commit()

    @skipUnless(rclpy_installed(), "Requires rclpy")
    def test_query(self):
        query = select(WorldMappingDAO)
        world_dao = self.session.scalars(query).all()
        world: World = world_dao[0].from_dao()
