import numpy as np

from semantic_world.spatial_computations.raytracer import RayTracer
from semantic_world.testing import world_setup_simple


def test_create_segmentation_mask(world_setup_simple):
    world, body1, body2, body3, body4 = world_setup_simple
    rt = RayTracer(world)
    rt.update_scene()

    camera_pose = np.array(
        [
            [1.0, 0.0, 0.0, -2.5],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.2],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )

    world.get_connection(world.root, body2).origin = np.array(
        [[1, 0, 0, -1], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    )

    seg = rt.create_segmentation_mask(camera_pose, resolution=256)
    assert seg.shape == (256, 256)  # Assuming a standard resolution

    hit, index, body = rt.ray_test(np.array([1, 0, 1]), np.array([-1, 0, 1]))
    assert hit is not None
    assert index is not None
    assert body is not None
    assert body1.index in seg
    assert body2.index in seg


def test_create_depth_map(world_setup_simple):
    world, body1, body2, body3, body4 = world_setup_simple
    rt = RayTracer(world)
    rt.update_scene()

    camera_pose = np.array(
        [
            [1.0, 0.0, 0.0, -2.5],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )

    depth_map = rt.create_depth_map(camera_pose, resolution=512)
    assert depth_map is not None
    assert depth_map[0, 0] == -1  # Assuming no objects are hit at the upper left corner
    assert depth_map.shape == (512, 512)
    assert depth_map.max() <= 2.5
    assert depth_map[depth_map > 0].min() >= 2.375


def test_ray_test(world_setup_simple):
    world, body1, body2, body3, body4 = world_setup_simple
    rt = RayTracer(world)
    rt.update_scene()

    hit, index, body = rt.ray_test(np.array([1, 0, 0.1]), np.array([-1, 0, 0.1]))
    assert hit is not None
    assert index is not None
    assert body is not None
    assert body1 in body

    # Test with a ray that does not hit any object
    hit, index, body = rt.ray_test(np.array([10, 10, 10]), np.array([20, 20, 20]))
    assert not np.any(hit)
    assert not np.any(index)
    assert not np.any(body)


def test_ray_test_batch(world_setup_simple):
    world, body1, body2, body3, body4 = world_setup_simple
    world.get_connection(world.root, body1).origin = np.array(
        [[1, 0, 0, 1], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    )
    world.get_connection(world.root, body2).origin = np.array(
        [[1, 0, 0, -1], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    )

    rt = RayTracer(world)
    rt.update_scene()

    rays = np.array([[1, 1, 0.1], [-1, 1, 0.1]])
    targets = np.array([[1, -1, 0.1], [-1, -1, 0.1]])

    hits, indices, bodies = rt.ray_test(rays, targets)
    assert hits is not None
    assert indices is not None
    assert bodies is not None
    assert len(hits) == len(rays)
    assert len(indices) == len(rays)
    assert len(bodies) == len(rays)
    # Test return
    assert bodies[0] == body1
    assert bodies[1] == body2
