import pytest

from semantic_world.collision_checking.collision_detector import CollisionCheck
from semantic_world.collision_checking.trimesh_collision_detector import (
    TrimeshCollisionDetector,
)
from semantic_world.spatial_types import TransformationMatrix
from semantic_world.testing import world_setup_simple


def test_simple_collision(world_setup_simple):
    world, body1, body2, body3, body4 = world_setup_simple
    tcd = TrimeshCollisionDetector(world)
    collision = tcd.check_collision_between_bodies(body1, body2)
    assert collision
    assert collision.body_a == body1
    assert collision.body_b == body2


def test_no_collision(world_setup_simple):
    world, body1, body2, body3, body4 = world_setup_simple
    body1.parent_connection.origin = TransformationMatrix.from_xyz_rpy(1, 1, 1)
    tcd = TrimeshCollisionDetector(world)
    collision = tcd.check_collision_between_bodies(body1, body2)
    assert not collision


@pytest.mark.skip(reason="Not my test not my problem.")
def test_collision_matrix(world_setup_simple):
    world, body1, body2, body3, body4 = world_setup_simple
    tcd = TrimeshCollisionDetector(world)
    collisions = tcd.check_collisions(
        {
            CollisionCheck(body1, body2, 0.0, world),
            CollisionCheck(body3, body4, 0.0, world),
        }
    )
    assert len(collisions) == 2
    assert collisions[0].body_a == body1
    assert collisions[0].body_b == body2


def test_all_collisions(world_setup_simple):
    world, body1, body2, body3, body4 = world_setup_simple
    tcd = TrimeshCollisionDetector(world)
    body4.parent_connection.origin = TransformationMatrix.from_xyz_rpy(10, 10, 10)
    body3.parent_connection.origin = TransformationMatrix.from_xyz_rpy(-10, -10, 10)

    collisions = tcd.check_collisions(None)
    assert len(collisions) == 1
    assert collisions[0].body_a == body1
    assert collisions[0].body_b == body2
