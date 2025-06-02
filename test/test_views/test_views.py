import logging
import os
import sys
import unittest

import pytest

try:
    from ripple_down_rules.user_interface.gui import RDRCaseViewer
    from PyQt6.QtWidgets import QApplication
except ImportError as e:
    logging.debug(e)
    QApplication = None
    RDRCaseViewer = None

from typing_extensions import Type, Optional, Callable

from ripple_down_rules.datastructures.dataclasses import CaseQuery
from ripple_down_rules.rdr import GeneralRDR
from semantic_world.adapters.urdf import URDFParser
from semantic_world.views import *
try:
    from semantic_world.views.world_rdr import world_rdr
except ImportError as e:
    world_rdr = None
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
        cls.kitchen_world = cls.get_kitchen_world()
        cls.apartment_world = cls.get_apartment_world()
        if RDRCaseViewer is not None and QApplication is not None and cls.use_gui:
            cls.app = QApplication(sys.argv)
            cls.viewer = RDRCaseViewer(save_file=cls.views_dir)

    def test_id(self):
        v1 = Handle(1)
        v2 = Handle(2)
        self.assertTrue(v1 is not v2)

    def test_handle_view(self):
        self.fit_rules_for_a_view_in_apartment(Handle, scenario=self.test_handle_view)

    def test_container_view(self):
        self.fit_rules_for_a_view_in_apartment(Container, scenario=self.test_container_view)

    def test_drawer_view(self):
        self.fit_rules_for_a_view_in_apartment(Drawer, scenario=self.test_drawer_view)

    def test_cabinet_view(self):
        self.fit_rules_for_a_view_in_apartment(Cabinet, scenario=self.test_cabinet_view)

    def test_door_view(self):
        self.fit_rules_for_a_view_in_apartment(Door, scenario=self.test_door_view)

    @unittest.skip("Skipping test for wardrobe view as it requires user input")
    def test_wardrobe_view(self):
        self.fit_rules_for_a_view_in_apartment(Wardrobe, scenario=self.test_wardrobe_view)

    @pytest.mark.skipif(world_rdr is None, reason="requires world_rdr")
    def test_generated_views(self):
        found_views = world_rdr.classify(self.kitchen_world)["views"]

        drawer_container_names = [v.body.name.name for v in found_views if isinstance(v, Container)]
        self.assertTrue(len(drawer_container_names) == 14)

    @pytest.mark.order("second_to_last")
    def test_apartment_views(self):
        rdr = self.fit_views_and_get_rdr(self.apartment_world, [Handle, Container, Drawer, Cabinet],
                                         world_factory=self.get_apartment_world, scenario=self.test_apartment_views)

        found_views = rdr.classify(self.apartment_world)

        drawer_container_names = [v.body.name.name for values in found_views.values() for v in values if
                                  type(v) is Container]

        self.assertTrue(len(drawer_container_names) == 19)

    @pytest.mark.order("last")
    def test_kitchen_views(self):
        rdr = self.fit_views_and_get_rdr(self.kitchen_world, [Handle, Container, Drawer, Cabinet],
                                         world_factory=self.get_kitchen_world, scenario=self.test_kitchen_views)

        found_views = rdr.classify(self.kitchen_world)

        drawer_container_names = [v.body.name.name for values in found_views.values() for v in values if
                                  type(v) is Container]
        self.assertTrue(len(drawer_container_names) == 14)

    @classmethod
    def get_kitchen_world(cls) -> World:
        """
        Return the kitchen world parsed from the URDF file.
        """
        parser = URDFParser(cls.kitchen)
        world = parser.parse()
        world.validate()
        return world

    @classmethod
    def get_apartment_world(cls) -> World:
        """
        Return the apartment world parsed from the URDF file.
        """
        parser = URDFParser(cls.apartment)
        world = parser.parse()
        world.validate()
        return world

    def fit_rules_for_a_view_in_kitchen(self, view_type: Type[View], update_existing_views: bool = False,
                                        scenario: Optional[Callable] = None) -> None:
        """
        Template method to test a specific view type in the kitchen world.

        :param view_type: The type of view to fit and assert.
        :param update_existing_views: If True, existing views will be updated with new rules, else they will be skipped.
        :param scenario: Optional callable that represents the test method or scenario that is being executed.
        """
        self.fit_rules_for_a_view_and_assert(self.kitchen_world, view_type, update_existing_views=update_existing_views,
                                             world_factory=self.get_kitchen_world, scenario=scenario)

    def fit_rules_for_a_view_in_apartment(self, view_type: Type[View], update_existing_views: bool = False,
                                          scenario: Optional[Callable] = None) -> None:
        """
        Template method to test a specific view type in the apartment world.

        :param view_type: The type of view to fit and assert.
        :param update_existing_views: If True, existing views will be updated with new rules, else they will be skipped.
        :param scenario: Optional callable that represents the test method or scenario that is being executed.
        """
        self.fit_rules_for_a_view_and_assert(self.apartment_world, view_type,
                                             update_existing_views=update_existing_views,
                                             world_factory=self.get_apartment_world, scenario=scenario)

    def fit_rules_for_a_view_and_assert(self, world: World, view_type: Type[View], update_existing_views: bool = False,
                                        world_factory: Optional[Callable] = None,
                                        scenario: Optional[Callable] = None) -> None:
        """
        Template method to test a specific view type in the given world.

        :param world: The world to fit the view to.
        :param view_type: The type of view to fit and assert.
        :param update_existing_views: If True, existing views will be updated with new rules, else they will be skipped.
        :param world_factory: Optional callable that can be used to recreate the world object.
        :param scenario: Optional callable that represents the test method or scenario that is being executed.
        """
        rdr = self.fit_views_and_get_rdr(world, [view_type], update_existing_views=update_existing_views,
                                         world_factory=world_factory, scenario=scenario)

        found_views = rdr.classify(world)['views']

        assert any(isinstance(v, view_type) for v in found_views)

    def fit_views_and_get_rdr(self, world: World, required_views: List[Type[View]],
                              update_existing_views: bool = False, world_factory: Optional[Callable] = None,
                              scenario: Optional[Callable] = None) -> GeneralRDR:
        """
        Fit rules to the specified views in the given world and return the RDR.

        :param world: The world to fit the views to.
        :param required_views: A list of view types that the RDR should be fitted to.
        :param update_existing_views: If True, existing views will be updated with new rules, else they will be skipped.
        :param world_factory: Optional callable that can be used to recreate the world object.
        :param scenario: Optional callable that represents the test method or scenario that is being executed.
        :return: An instance of GeneralRDR fitted to the specified views.
        """
        rdr = self.load_or_create_rdr()

        self.fit_rdr_to_views(rdr, required_views, world, update_existing_views=update_existing_views,
                              world_factory=world_factory, scenario=scenario)
        rdr.save(self.views_dir, self.views_rdr_model_name, package_name="semantic_world")

        return rdr

    def load_or_create_rdr(self) -> GeneralRDR:
        """
        Load an existing RDR or create a new one if it does not exist.

        :return: An instance of GeneralRDR loaded from the specified directory or a new instance of GeneralRDR.
        """
        if not os.path.exists(os.path.join(self.views_dir, self.views_rdr_model_name)):
            return GeneralRDR(save_dir=self.views_dir, model_name=self.views_rdr_model_name, viewer=self.viewer)
        else:
            rdr = GeneralRDR.load(self.views_dir, self.views_rdr_model_name, package_name="semantic_world")
            rdr.set_viewer(self.viewer)
        return rdr

    @staticmethod
    def fit_rdr_to_views(rdr: GeneralRDR, required_views: List[Type[View]], world: World,
                         update_existing_views: bool = False,
                         world_factory: Optional[Callable] = None,
                         scenario: Optional[Callable] = None) -> None:
        """
        Fits the given RDR to the required views in the world.

        :param rdr: The RDR to fit.
        :param required_views: A list of view types that the RDR should be fitted to.
        :param world: The world that contains or should contain the views.
        :param update_existing_views: If True, existing views will be updated with new rules, else they will be skipped.
        :param world_factory: Optional callable that can be used to recreate the world object.
        :param scenario: Optional callable that represents the test method or scenario that is being executed.
        """
        for view in required_views:
            case_query = CaseQuery(world, "views", (view,), False, case_factory=world_factory, scenario=scenario)
            rdr.fit_case(case_query, update_existing_rules=update_existing_views)
