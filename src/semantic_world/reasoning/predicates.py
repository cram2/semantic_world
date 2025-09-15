import itertools

import numpy as np
import trimesh.boolean
from entity_query_language import let, an, entity, contains, and_, not_, the
from random_events.interval import Interval
from typing_extensions import List, Optional

from ..collision_checking.collision_detector import CollisionCheck, Collision
from ..collision_checking.trimesh_collision_detector import TrimeshCollisionDetector
from ..datastructures.prefixed_name import PrefixedName
from ..datastructures.variables import SpatialVariables
from ..robots import (
    Camera,
    Manipulator,
    AbstractRobot,
    ParallelGripper,
    Arm,
)
from ..spatial_computations.ik_solver import (
    MaxIterationsException,
    UnreachableException,
)
from ..spatial_computations.raytracer import RayTracer
from ..spatial_types.spatial_types import TransformationMatrix
from ..world import World
from ..world_description.connections import FixedConnection
from ..world_description.world_entity import Body, Region, KinematicStructureEntity


def stable(obj: Body) -> bool:
    """
    Checks if an object is stable in the world. Stable meaning that its position will not change after simulating
    physics in the World. This will be done by simulating the world for 10 seconds and comparing
    the previous coordinates with the coordinates after the simulation.

    :param obj: The object which should be checked
    :return: True if the given object is stable in the world False else
    """
    raise NotImplementedError("Needs multiverse")


def contact(
    body1: Body,
    body2: Body,
    threshold: float = 0.001,
) -> bool:
    """
    Checks if two objects are in contact or not.

    :param body1: The first object
    :param body2: The second object
    :param threshold: The threshold for contact detection
    :return: True if the two objects are in contact False else
    """
    tcd = TrimeshCollisionDetector(body1._world)
    result = tcd.check_collision_between_bodies(body1, body2)

    if result is None:
        return False
    return result.contact_distance < threshold


def robot_in_collision(
    robot: AbstractRobot,
    ignore_collision_with: Optional[List[Body]] = None,
    threshold: float = 0.001,
) -> List[Collision]:
    """
    Check if the robot collides with any object in the world at the given pose.

    :param robot: The robot object
    :param ignore_collision_with: A list of objects to ignore collision with
    :param threshold: The threshold for contact detection
    :return: True if the robot collides with any object, False otherwise
    """

    if ignore_collision_with is None:
        ignore_collision_with = []

    body = let("body", type_=Body, domain=robot._world.bodies_with_enabled_collision)
    possible_collisions_bodies = an(
        entity(
            body,
            and_(
                not_(contains(robot.bodies, body)),
                not_(contains(ignore_collision_with, body)),
            ),
        ),
    ).evaluate()

    tcd = TrimeshCollisionDetector(robot._world)

    collisions = tcd.check_collisions(
        {
            CollisionCheck(robot_body, collision_body, threshold, robot._world)
            for robot_body, collision_body in itertools.product(
                robot.bodies_with_collisions, possible_collisions_bodies
            )
        }
    )
    return collisions


def robot_holds_body(robot: AbstractRobot, body: Body) -> bool:
    """
    Check if a robot is holding an object.

    :param robot: The robot object
    :param body: The body to check if it is picked
    :return: True if the robot is holding the object, False otherwise
    """
    grippers = an(
        entity(
            g := let("gripper", ParallelGripper, robot._world.views), g._robot == robot
        )
    ).evaluate()

    return any([is_body_in_gripper(body, gripper) > 0.0 for gripper in grippers])


def get_visible_bodies(camera: Camera) -> List[KinematicStructureEntity]:
    """
    Get all bodies and regions that are visible from the given camera using a segmentation mask.

    :param camera: The camera for which the visible objects should be returned
    :return: A list of bodies/regions that are visible from the camera
    """
    rt = RayTracer(camera._world)
    rt.update_scene()

    # This ignores the camera orientation and sets it to identity
    cam_pose = np.eye(4, dtype=float)
    cam_pose[:3, 3] = camera.root.global_pose.to_np()[:3, 3]

    seg = rt.create_segmentation_mask(TransformationMatrix(cam_pose, camera._world.root), resolution=256)
    indices = np.unique(seg)
    indices = indices[indices > -1]
    bodies = [camera._world.kinematic_structure[i] for i in indices]

    return bodies


def visible(camera: Camera, obj: KinematicStructureEntity) -> bool:
    """
    Checks if a body/region is visible by the given camera.
    """
    return obj in get_visible_bodies(camera)


def occluding_bodies(camera: Camera, body: Body) -> List[Body]:
    """
    Determines the bodies that occlude a given body in the scene as seen from a specified camera.

    This function uses a ray-tracing approach to check occlusion. Every body that hides anything from the target body
    is an occluding body.

    :param camera: The camera for which the occluding bodies should be returned
    :param body: The body for which the occluding bodies should be returned
    :return: A list of bodies that are occluding the given body.
    """

    # get camera pose
    camera_pose = np.eye(4, dtype=float)
    camera_pose[:3, 3] = camera.root.global_pose.to_np()[:3, 3]
    camera_pose = TransformationMatrix(camera_pose, camera._world.root)

    # create a world only containing the target body
    world_without_occlusion = World()
    root = Body(name=PrefixedName("root"))
    with world_without_occlusion.modify_world():
        world_without_occlusion.add_body(root)
        copied_body = Body.from_json(body.to_json())
        root_to_copied_body = FixedConnection(
            parent=root,
            child=copied_body,
            _world=world_without_occlusion,
            origin_expression=body.global_pose,
        )
        world_without_occlusion.add_connection(root_to_copied_body)

    # get segmentation mask without occlusion
    ray_tracer_without_occlusion = RayTracer(world_without_occlusion)
    ray_tracer_without_occlusion.update_scene()
    segmentation_mask_without_occlusion = (
        ray_tracer_without_occlusion.create_segmentation_mask(
            camera_pose, resolution=256
        )
    )

    # get segmentation mask with occlusion
    ray_tracer_with_occlusion = RayTracer(camera._world)
    ray_tracer_with_occlusion.update_scene()
    segmentation_mask_with_occlusion = (
        ray_tracer_with_occlusion.create_segmentation_mask(camera_pose, resolution=256)
    )

    mask_without_occluders = segmentation_mask_without_occlusion[
        segmentation_mask_without_occlusion == copied_body.index
    ].nonzero()

    mask_with_occluders = segmentation_mask_with_occlusion[
        mask_without_occluders != body.index
    ]
    indices = np.unique(mask_with_occluders)
    indices = indices[indices > -1]
    bodies = [camera._world.kinematic_structure[i] for i in indices]
    return bodies


def reachable(pose: TransformationMatrix, root: Body, tip: Body) -> bool:
    """
    Checks if a manipulator can reach a given position.
    This is determined by inverse kinematics.

    :param pose: The pose to reach
    :param root: The root of the kinematic chain.
    :param tip: The threshold between the end effector and the position.
    :return: True if the end effector is closer than the threshold to the target position, False in every other case
    """
    try:
        root._world.compute_inverse_kinematics(
            root=root, tip=tip, target=pose, max_iterations=1000
        )
    except MaxIterationsException as e:
        return False
    except UnreachableException as e:
        return False
    return True


def blocking(
    pose: TransformationMatrix,
    root: Body,
    tip: Body,
) -> List[Collision]:
    """
    Get the bodies that are blocking the robot from reaching a given position.
    The blocking are all bodies that are in collision with the robot when reaching for the pose.

    :param pose: The pose to reach
    :param root: The root of the kinematic chain.
    :param tip: The threshold between the end effector and the position.
    :return: A list of bodies the robot is in collision with when reaching for the specified object or None if the pose or object is not reachable.
    """
    result = root._world.compute_inverse_kinematics(
        root=root, tip=tip, target=pose, max_iterations=1000
    )
    with root._world.modify_world():
        for dof, state in result.items():
            root._world.state[dof.name].position = state

    robot = the(
        entity(r := let("robot", AbstractRobot, root._world.views), tip in r.bodies)
    ).evaluate()
    return robot_in_collision(robot, [])


def is_supported_by(
    supported_body: Body, supporting_body: Body, max_intersection_height: float = 0.1
) -> bool:
    """
    Checks if one object is supporting another object.

    :param supported_body: Object that is supported
    :param supporting_body: Object that potentially supports the first object
    :param max_intersection_height: Maximum height of the intersection between the two objects.
    If the intersection is higher than this value, the check returns False due to unhandled clipping.
    :return: True if the second object is supported by the first object, False otherwise
    """
    if below(supported_body, supporting_body, supported_body.global_pose):
        return False
    bounding_box_supported_body = supported_body.as_bounding_box_collection_at_origin(
        TransformationMatrix(reference_frame=supported_body)
    ).event
    bounding_box_supporting_body = supporting_body.as_bounding_box_collection_at_origin(
        TransformationMatrix(reference_frame=supported_body)
    ).event

    intersection = (
        bounding_box_supported_body & bounding_box_supporting_body
    ).bounding_box()

    if intersection.is_empty():
        return False

    z_intersection: Interval = intersection[SpatialVariables.z.value]
    size = sum([si.upper - si.lower for si in z_intersection.simple_sets])
    return size < max_intersection_height


def is_body_in_gripper(
    body: Body, gripper: ParallelGripper, sample_size: int = 100
) -> float:
    """
    Check if the body in the gripper.

    This method samples random rays between the finger and the thumb and returns the marginal probability that the rays
    intersect.

    :param body: The body for which the check should be done.
    :param gripper: The gripper for which the check should be done.
    :param sample_size: The number of rays to sample.

    :return: The percentage of rays between the fingers that hit the body.
    """

    # Retrieve meshes in local frames
    thumb_mesh = gripper.thumb.tip.combined_collision_mesh.copy()
    finger_mesh = gripper.finger.tip.combined_collision_mesh.copy()
    body_mesh = body.combined_collision_mesh.copy()

    # Transform copies of the meshes into the world frame
    body_mesh.apply_transform(body.global_pose.to_np())
    thumb_mesh.apply_transform(gripper.thumb.tip.global_pose.to_np())
    finger_mesh.apply_transform(gripper.finger.tip.global_pose.to_np())

    # get random points from thumb mesh
    finger_points = trimesh.sample.sample_surface(finger_mesh, sample_size)[0]
    thumb_points = trimesh.sample.sample_surface(thumb_mesh, sample_size)[0]

    rt = RayTracer(gripper._world)
    rt.update_scene()

    points, index_ray, bodies = rt.ray_test(finger_points, thumb_points)
    return len([b for b in bodies if b == body]) / sample_size


def is_body_in_region(body: Body, region: Region) -> float:
    """
    Check if the body is in the region by computing the fraction of the body's
    collision volume that lies inside the region's area volume.

    Implementation detail: both the body and region meshes are defined in their
    respective local frames; we must transform them into a common (world) frame
    using their global poses before computing the boolean intersection.

    :param body: The body for which the check should be done.
    :param region: The region to check if the body is in.
    :return: The percentage (0.0..1.0) of the body's volume that lies in the region.
    """
    # Retrieve meshes in local frames
    body_mesh_local = body.combined_collision_mesh
    region_mesh_local = region.combined_area_mesh

    # Transform copies of the meshes into the world frame
    body_mesh = body_mesh_local.copy().apply_transform(body.global_pose.to_np())
    region_mesh = region_mesh_local.copy().apply_transform(region.global_pose.to_np())
    intersection = trimesh.boolean.intersection([body_mesh, region_mesh])

    # no body volume -> zero fraction
    body_volume = body_mesh.volume
    if body_volume <= 1e-12:
        return 0.0

    return intersection.volume / body_volume


def left_of(body: Body, other: Body, point_of_view: TransformationMatrix) -> bool:
    """
    Check if the body is left of the other body if you are looking from the point of view.

    The "left" direction is taken as the -Y axis of the given point of view.
    The comparison is done using the centers of mass computed from the bodies' collision geometry.

    :param body: The body for which the check should be done.
    :param other: The other body.
    :param point_of_view: The reference spot from where to look at the bodies.
    :return: True if the body is left of the other body, False otherwise
    """
    return _signed_distance_along_direction(body, other, point_of_view, 1) > 0.0


def right_of(body: Body, other: Body, point_of_view: TransformationMatrix) -> bool:
    """
    Check if the body is right of the other body if you are looking from the point of view.

    The "right" direction is taken as the +Y axis of the given point of view.
    The comparison is done using the centers of mass computed from the bodies' collision geometry.

    :param body: The body for which the check should be done.
    :param other: The other body.
    :param point_of_view: The reference pose that defines the up direction for the comparison.
    :return: True if the body is right of the other body, False otherwise
    """
    return _signed_distance_along_direction(body, other, point_of_view, 1) < 0.0


def above(body: Body, other: Body, point_of_view: TransformationMatrix) -> bool:
    """
    Check if the body is above the other body with respect to the point_of_view's up direction (+Z axis).

    The "up" direction is taken as the +Z axis of the given point_of_view.
    The comparison is done using the centers of mass computed from the bodies' collision geometry.

    :param body: The body for which the check should be done.
    :param other: The other body.
    :param point_of_view: The reference spot from where to look at the bodies.
    :return: True if the center of mass of "body" is above that of "other" along the point_of_view's +Z axis.
    """
    return _signed_distance_along_direction(body, other, point_of_view, 2) > 0.0


def below(body: Body, other: Body, point_of_view: TransformationMatrix) -> bool:
    """
    Check if the body is below the other body with respect to the point of view's up direction (+Z axis).

    The "below" direction is taken as the -Z axis of the given point of view.
    The comparison is done using the centers of mass computed from the bodies' collision geometry.

    :param body: The body for which the check should be done.
    :param other: The other body.
    :param point_of_view: The reference spot from where to look at the bodies.
    :return: True if the center of mass of "body" is below that of "other" along the point_of_view's +Z axis.
    """
    return _signed_distance_along_direction(body, other, point_of_view, 2) < 0.0


def behind(body: Body, other: Body, point_of_view: TransformationMatrix) -> bool:
    """
    Check if the body is behind the other body if you are looking from the point of view.

    The "behind" direction is defined as the -X axis of the given point of view.
    The comparison is done using the centers of mass computed from the bodies' collision
    geometry.

    :param body: The body for which the check should be done.
    :param other: The other body.
    :param point_of_view: The reference spot from where to look at the bodies.
    :return: True if the body is behind the other body, False otherwise
    """
    return _signed_distance_along_direction(body, other, point_of_view, 0) < 0.0


def in_front_of(body: Body, other: Body, point_of_view: TransformationMatrix) -> bool:
    """
    Check if the body is in front of another body if you are looking from the point of view.

    The "front" direction is defined as the +X axis of the given point of view.
    The comparison is done using the centers of mass computed from the bodies' collision
    geometry.

    :param body: The body for which the check should be done.
    :param other: The other body.
    :param point_of_view: The reference spot from where to look at the bodies.
    :return: True if the body is in front of the other body, False otherwise
    """
    return _signed_distance_along_direction(body, other, point_of_view, 0) > 0.0


def _center_of_mass_in_world(b: Body) -> np.ndarray:
    """
    Compute the center of mass of an object in the world coordinate frame.
    :param b: The body to compute the center of mass of.
    :return: The bodies center of mass as a 3D array.
    """
    # Center of mass in the body's local frame (collision geometry)
    com_local = b.combined_collision_mesh.center_mass  # (3,)
    # Transform to world frame using the body's global pose
    T_bw = b.global_pose.to_np()  # body -> world
    com_h = np.array([com_local[0], com_local[1], com_local[2], 1.0], dtype=float)
    return (T_bw @ com_h)[:3]


def _signed_distance_along_direction(
    body: Body, other: Body, point_of_view: TransformationMatrix, index: int
) -> float:
    """
    Calculate the spatial relation between two bodies with respect to a given
    reference point and a specified axis index. This function computes the
    signed distance along a specified direction derived from the reference point
    to compare the positions of the centers of mass of the two bodies.

    :param body: The first body for which the spatial relation is calculated.
    :param other: The second body to which the spatial relation is compared.
    :param point_of_view: Transformation matrix that provides the reference
        frame for the calculation.
    :param index: The index of the axis in the transformation matrix along which
        the spatial relation is computed.
    :return: The signed distance between the first and the second body's centers
        of mass along the given direction. A positive result indicates that the
        first body's center of mass is further along the direction of the
        specified axis than the second body's center of mass.
    """
    ref_np = point_of_view.to_np()
    front_world = ref_np[:3, index]
    front_norm = front_world / (np.linalg.norm(front_world) + 1e-12)

    s_body = float(np.dot(front_norm, _center_of_mass_in_world(body)))
    s_other = float(np.dot(front_norm, _center_of_mass_in_world(other)))
    return s_body - s_other
