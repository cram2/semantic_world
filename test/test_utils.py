import os

from semantic_world.utils import get_semantic_world_directory_root


def test_get_semantic_world_directory_root():
    path = os.path.abspath(__file__)
    root = get_semantic_world_directory_root(path)
    assert root == os.path.abspath(os.path.join(os.path.dirname(path), ".."))
