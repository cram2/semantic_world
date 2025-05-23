import os
import sys
import unittest

from PyQt6.QtWidgets import QApplication
from typing_extensions import List, Type

from ripple_down_rules.datastructures.dataclasses import CaseQuery
from ripple_down_rules.experts import Human
from ripple_down_rules.rdr import GeneralRDR
from ripple_down_rules.user_interface.gui import RDRCaseViewer
from semantic_world.adapters.urdf import URDFParser
from semantic_world.views import *
from semantic_world.views.view_classifiers import world_rdr
from semantic_world.world import World


class ViewTestCase(unittest.TestCase):
    urdf_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "resources", "urdf")
    kitchen = os.path.join(urdf_dir, "kitchen-small.urdf")
    apartment = os.path.join(urdf_dir, "apartment.urdf")
    main_dir = os.path.join(os.path.dirname(__file__), '..', '..')
    views_dir = os.path.join(main_dir, "src/semantic_world/views/view_classifiers")
    views_json_dir = os.path.join(main_dir, "src/semantic_world/views/rdr_json")
    views_rdr_filename = os.path.join(views_json_dir, "views_rdr")
    test_dir = os.path.join(main_dir, 'tests')
    expert_answers_dir = os.path.join(test_dir, "test_expert_answers")
    app: QApplication
    viewer: RDRCaseViewer

    @classmethod
    def setUpClass(cls):
        cls.kitchen_parser = URDFParser(cls.kitchen)
        cls.apartment_parser = URDFParser(cls.apartment)
        cls.app = QApplication(sys.argv)
        cls.viewer = RDRCaseViewer(save_file=cls.views_json_dir)

    def test_id(self):
        v1 = Handle(1)
        print(hash(v1))
        v2 = Handle(2)
        self.assertTrue(v1 is not v2)

    def test_generated_views(self):
        world = self.kitchen_parser.parse()
        world.validate()

        found_views = world_rdr.classify(world)["views"]

        drawer_container_names = [v.body.name.name for v in found_views if type(v) is Container]
        self.assertTrue(len(drawer_container_names) == 14)

    def test_kitchen_views(self):
        world = self.kitchen_parser.parse()
        world.validate()

        rdr = self.fit_views_and_get_rdr(world, [Handle, Container, Drawer, Cabinet])

        found_views = rdr.classify(world)

        drawer_container_names = [v.body.name.name for values in found_views.values() for v in values if type(v) is Container]
        self.assertTrue(len(drawer_container_names) == 14)

        # print("found types are: ", {type(v).__name__ for values in views.values() for v in values})
        # print("\n".join(drawer_container_names))
        # print(len(drawer_container_names))

    def test_apartment_views(self):
        world = self.apartment_parser.parse()
        world.validate()

        rdr = self.fit_views_and_get_rdr(world,[Handle, Container, Drawer, Cabinet])

        found_views = rdr.classify(world)

        drawer_container_names = [v.body.name.name for values in found_views.values() for v in values if
                                  type(v) is Container]

        self.assertTrue(len(drawer_container_names) == 19)

    def fit_views_and_get_rdr(self, world: World, required_views: List[Type[View]], save_rules: bool = True,
                              use_current_rules: bool = True, update_existing_views: bool = False) -> GeneralRDR:
        expert = Human(viewer=self.viewer)
        if use_current_rules:
            grdr = GeneralRDR.load(self.views_rdr_filename)
            grdr.update_from_python_file(self.views_dir)
            grdr.set_viewer(self.viewer)
        else:
            grdr = GeneralRDR(viewer=self.viewer)

        for view in required_views:
            case_query = CaseQuery(world, "views", (view,), False)
            if update_existing_views or case_query.attribute_name not in grdr.start_rules_dict:
                grdr.fit_case(CaseQuery(world, "views", (view,), False), expert=expert)
        if save_rules:
            grdr.write_to_python_file(self.views_dir)
            grdr.save(self.views_rdr_filename)

        return grdr
