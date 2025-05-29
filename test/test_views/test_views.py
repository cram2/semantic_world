import logging
import os
import sys
import unittest

try:
    from ripple_down_rules.user_interface.gui import RDRCaseViewer
    from PyQt6.QtWidgets import QApplication
except ImportError as e:
    logging.debug(e)
    QApplication = None
    RDRCaseViewer = None

from typing_extensions import List, Type, Optional

from ripple_down_rules.datastructures.dataclasses import CaseQuery
from ripple_down_rules.rdr import GeneralRDR
from semantic_world.adapters.urdf import URDFParser
from semantic_world.views import *
from semantic_world.views.world_rdr import world_rdr
from semantic_world.world import World


class ViewTestCase(unittest.TestCase):
    """
    **Important**:
    ===============
    If use_gui is set to False, use the command line interface to test the views.

    e.g. from the terminal while at the root of the repository, run:

    cd test/test_views && python -m pytest test_views.py

    ===============
    OR if you want to run only the kitchen views test:

    cd test/test_views && python -m pytest -k "test_kitchen_views"

    """
    urdf_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "resources", "urdf")
    kitchen = os.path.join(urdf_dir, "kitchen-small.urdf")
    apartment = os.path.join(urdf_dir, "apartment.urdf")
    main_dir = os.path.join(os.path.dirname(__file__), '..', '..')
    views_dir = os.path.join(main_dir, "src/semantic_world/views")
    views_rdr_model_name = "world_rdr"
    test_dir = os.path.join(main_dir, 'tests')
    expert_answers_dir = os.path.join(test_dir, "test_expert_answers")
    app: Optional[QApplication] = None
    viewer: Optional[RDRCaseViewer] = None
    use_gui: bool = True

    @classmethod
    def setUpClass(cls):
        cls.kitchen_parser = URDFParser(cls.kitchen)
        cls.apartment_parser = URDFParser(cls.apartment)
        if RDRCaseViewer is not None and QApplication is not None and cls.use_gui:
            cls.app = QApplication(sys.argv)
            cls.viewer = RDRCaseViewer(save_file=cls.views_dir)

    def test_id(self):
        v1 = Handle(1)
        v2 = Handle(2)
        self.assertTrue(v1 is not v2)

    def test_door_view(self):
        world = self.apartment_parser.parse()
        world.validate()
        self.fit_rules_for_a_view_and_assert(world, Door)

    @unittest.skip("Skipping test for wardrobe view as it requires user input")
    def test_wardrobe_view(self):
        world = self.apartment_parser.parse()
        world.validate()
        self.fit_rules_for_a_view_and_assert(world, Wardrobe)

    def test_generated_views(self):
        world = self.kitchen_parser.parse()
        world.validate()

        found_views = world_rdr.classify(world)["views"]

        drawer_container_names = [v.body.name.name for v in found_views if isinstance(v, Container)]
        self.assertTrue(len(drawer_container_names) == 14)

    def test_kitchen_views(self):
        world = self.kitchen_parser.parse()
        world.validate()

        rdr = self.fit_views_and_get_rdr(world, [Handle, Container, Drawer, Cabinet])

        found_views = rdr.classify(world)

        drawer_container_names = [v.body.name.name for values in found_views.values() for v in values if type(v) is Container]
        self.assertTrue(len(drawer_container_names) == 14)

    def test_apartment_views(self):
        world = self.apartment_parser.parse()
        world.validate()

        rdr = self.fit_views_and_get_rdr(world,[Handle, Container, Drawer, Cabinet])

        found_views = rdr.classify(world)

        drawer_container_names = [v.body.name.name for values in found_views.values() for v in values if
                                  type(v) is Container]

        self.assertTrue(len(drawer_container_names) == 19)

    def fit_rules_for_a_view_and_assert(self, world: World, view_type: Type[View], update_existing_views: bool = False):
        """
        Template method to test a specific view type in the given world.
        """
        rdr = self.fit_views_and_get_rdr(world, [view_type], update_existing_views=update_existing_views)

        found_views = rdr.classify(world)['views']

        assert any(isinstance(v, view_type) for v in found_views)

    def fit_views_and_get_rdr(self, world: World, required_views: List[Type[View]],
                              update_existing_views: bool = False) -> GeneralRDR:
        """
        Fit rules to the specified views in the given world and return the RDR.

        :param world: The world to fit the views to.
        :param required_views: A list of view types that the RDR should be fitted to.
        :param update_existing_views: If True, existing views will be updated with new rules, else they will be skipped.
        :return: An instance of GeneralRDR fitted to the specified views.
        """
        rdr = self.load_or_create_rdr()

        self.fit_rdr_to_views(rdr, required_views, world, update_existing_views=update_existing_views)

        rdr.save(self.views_dir, self.views_rdr_model_name)

        return rdr

    def load_or_create_rdr(self) -> GeneralRDR:
        """
        Load an existing RDR or create a new one if it does not exist.

        :return: An instance of GeneralRDR loaded from the specified directory or a new instance of GeneralRDR.
        """
        if not os.path.exists(os.path.join(self.views_dir, self.views_rdr_model_name)):
            return GeneralRDR(save_dir=self.views_dir, model_name=self.views_rdr_model_name, viewer=self.viewer)
        else:
            rdr = GeneralRDR.load(self.views_dir, self.views_rdr_model_name)
            rdr.set_viewer(self.viewer)
        return rdr

    @staticmethod
    def fit_rdr_to_views(rdr: GeneralRDR, required_views: List[Type[View]], world: World,
                         update_existing_views: bool = False) -> None:
        """
        Fits the given RDR to the required views in the world.

        :param rdr: The RDR to fit.
        :param required_views: A list of view types that the RDR should be fitted to.
        :param world: The world that contains or should contain the views.
        :param update_existing_views: If True, existing views will be updated with new rules, else they will be skipped.
        """
        for view in required_views:
            case_query = CaseQuery(world, "views", (view,), False)
            rdr.fit_case(case_query, update_existing_rules=update_existing_views)
