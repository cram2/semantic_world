import os.path

import pytest

from semantic_world.adapters.urdf import URDFParser
from semantic_world.robots.hsrb import HumanSupportRobotB
from semantic_world.testing import rclpy_node
from semantic_world.utils import get_semantic_world_directory_root


def hsrb_meshes_available():
    path = os.getenv("ROS_PACKAGE_PATH")
    if isinstance(path, str):
        return path.endswith("hsr_meshes")
    elif isinstance(path, list):
        return any(p.endswith("hsr_meshes") for p in path)
    else:
        return False


@pytest.mark.skipif(not hsrb_meshes_available(), reason="HSRB meshes not available.")
def test_hsrb(rclpy_node):
    file = os.path.join(
        get_semantic_world_directory_root(os.path.abspath(__file__)),
        "resources",
        "urdf",
        "robots",
        "hsrb.urdf",
    )

    assert os.path.exists(file)
    world = URDFParser.from_file(file).parse()
    world.plot_kinematic_structure()
    hsr = HumanSupportRobotB.from_world(world=world)
