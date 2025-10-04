import json
from semantic_world.adapters.ros.world_fetcher import (
    WorldFetcher,
    fetch_world_from_service,
)
from semantic_world.datastructures.prefixed_name import PrefixedName
from semantic_world.testing import rclpy_node
from semantic_world.world import World
from semantic_world.world_description.connections import Connection6DoF
from semantic_world.world_description.world_entity import Body
from semantic_world.world_description.world_modification import (
    WorldModelModificationBlock,
)

from std_srvs.srv import Trigger


def create_dummy_world():
    """
    Create a simple world with two bodies and a connection.
    """
    world = World()
    body_1 = Body(name=PrefixedName("body_1"))
    body_2 = Body(name=PrefixedName("body_2"))
    with world.modify_world():
        world.add_kinematic_structure_entity(body_1)
        world.add_kinematic_structure_entity(body_2)
        world.add_connection(Connection6DoF(body_1, body_2, _world=world))
    return world


def test_get_modifications_as_json_empty_world(rclpy_node):
    """
    Test that get_modifications_as_json returns an empty list for a world with no modifications.
    """
    world = World()
    fetcher = WorldFetcher(node=rclpy_node, world=world)

    modifications_json = fetcher.get_modifications_as_json()
    modifications_list = json.loads(modifications_json)

    assert modifications_list == []
    fetcher.close()


def test_service_callback_success(rclpy_node):
    """
    Test that the service callback returns success with the modifications JSON.
    """
    world = create_dummy_world()
    fetcher = WorldFetcher(node=rclpy_node, world=world)

    # Create a mock request and response
    request = Trigger.Request()
    response = Trigger.Response()

    # Call the service callback directly
    result = fetcher.service_callback(request, response)

    assert result.success is True

    # Verify the message is valid JSON
    modifications_list = [
        WorldModelModificationBlock.from_json(d) for d in json.loads(result.message)
    ]
    assert isinstance(modifications_list, list)
    assert len(modifications_list) == 1
    assert len(modifications_list[0].modifications) == 10

    fetcher.close()


def test_service_callback_with_multiple_modifications(rclpy_node):
    """
    Test that the service callback returns all modifications when multiple changes are made.
    """
    world = World()
    fetcher = WorldFetcher(node=rclpy_node, world=world)

    # Make multiple modifications
    body_1 = Body(name=PrefixedName("body_1"))
    body_2 = Body(name=PrefixedName("body_2"))
    body_3 = Body(name=PrefixedName("body_3"))

    with world.modify_world():
        world.add_kinematic_structure_entity(body_1)

    with world.modify_world():
        world.add_kinematic_structure_entity(body_2)
        world.add_kinematic_structure_entity(body_3)
        world.add_connection(Connection6DoF(body_1, body_2, _world=world))
        world.add_connection(Connection6DoF(body_2, body_3, _world=world))

    request = Trigger.Request()
    response = Trigger.Response()

    result = fetcher.service_callback(request, response)

    assert result.success is True
    # Verify the message is valid JSON
    modifications_list = [
        WorldModelModificationBlock.from_json(d) for d in json.loads(result.message)
    ]
    assert isinstance(modifications_list, list)
    assert len(modifications_list) == 2
    assert len(modifications_list[0].modifications) == 1
    assert len(modifications_list[1].modifications) == 18
    fetcher.close()


def test_world_fetching(rclpy_node):
    world = create_dummy_world()
    fetcher = WorldFetcher(node=rclpy_node, world=world)

    world2 = fetch_world_from_service(
        rclpy_node,
    )
    assert len(world2.bodies) == 2
    assert len(world2.connections) == 1
    assert len(world2._model_modification_blocks) == 1
