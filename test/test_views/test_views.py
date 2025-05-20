import os
import sys
import unittest

from PyQt6.QtWidgets import QApplication

from ripple_down_rules.datastructures.dataclasses import CaseQuery
from ripple_down_rules.experts import Human
from ripple_down_rules.rdr import GeneralRDR
from ripple_down_rules.user_interface.gui import RDRCaseViewer
from semantic_world.adapters.urdf import URDFParser
from semantic_world.views import *


class ViewTestCase(unittest.TestCase):
    urdf_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "resources", "urdf")
    kitchen = os.path.join(urdf_dir, "kitchen-small.urdf")
    apartment = os.path.join(urdf_dir, "apartment.urdf")
    main_dir = os.path.join(os.path.dirname(__file__), '..')
    expert_answers_dir = os.path.join(main_dir, "test_expert_answers")
    generated_rdrs_dir = os.path.join(main_dir, "test_generated_rdrs")
    saved_rdrs_dir = os.path.join(main_dir, "saved_rdrs")
    app: QApplication
    viewer: RDRCaseViewer

    def setUp(self):
        self.kitchen_parser = URDFParser(self.kitchen)
        self.apartment_parser = URDFParser(self.apartment)
        self.app = QApplication(sys.argv)
        self.viewer = RDRCaseViewer()

    def test_id(self):
        v1 = Handle(1)
        print(hash(v1))
        v2 = Handle(2)
        self.assertTrue(v1 is not v2)

    def test_kitchen_views(self):
        # Views that need to be classified
        needed = ["table", "room", "cabinet"]
        rooms = ["living room", "kitchen"]
        cabinets = ["refrigerator", "oven", "microwave", "dishwasher"]
        shelves = ["shelf", "drawer"]
        tableware = ["plate", "bowl", "cutlery", "cups"]
        cutlery = ["knife", "fork", "spoon"]
        cups = ["cup", "mug"]
        food = ["milk", "cereal"]
        furniture = ["sofa", "chair"]
        world = self.kitchen_parser.parse()
        world.validate()

        rdr_filename = os.path.join(self.saved_rdrs_dir, "kitchen_rdr")

        use_current_rules = False
        save_rules = True
        expert = Human(viewer=self.viewer)

        if use_current_rules:
            grdr = GeneralRDR.load(rdr_filename)
            grdr.update_from_python_file(self.generated_rdrs_dir)
        else:
            grdr = GeneralRDR()
            for view in [Handle, Container, Drawer, Cabinet]:
                grdr.fit_case(CaseQuery(world, "views", (view,), False), expert=expert)

        views = grdr.classify(world)

        if save_rules:
            grdr.write_to_python_file(self.generated_rdrs_dir)
            grdr.save(rdr_filename)

        print("found types are: ", {type(v).__name__ for values in views.values() for v in values})
        drawer_container_names = [v.body.name for values in views.values() for v in values if type(v) is Container]
        print("\n".join(drawer_container_names))
        print(len(drawer_container_names))

    def test_apartment_views(self):
        world = self.apartment_parser.parse()
        world.validate()
        grdr = GeneralRDR()
        use_loaded_answers = True
        save_answers = False
        append = True
        filename = "../test_expert_answers/kitchen_expert_answers_fit"
        filename = os.path.join(os.path.dirname(__file__), filename)
        expert = Human(use_loaded_answers=use_loaded_answers, append=append)
        if use_loaded_answers:
            expert.load_answers(filename)
        for view in [Handle, Container, Drawer, Cabinet]:
            grdr.fit_case(CaseQuery(world, "views", (view,), False), expert=expert)
        if save_answers:
            expert.save_answers(filename)
        views = grdr.classify(world)
        print(views)
        drawer_container_names = [v.body.name for values in views.values() for v in values if type(v) is Container]
        print("\n".join(drawer_container_names))
        print(len(drawer_container_names))
