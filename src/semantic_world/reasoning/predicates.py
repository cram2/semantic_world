import itertools

import numpy as np
from entity_query_language import let, an, entity, contains, and_, not_
from typing_extensions import List, Optional

from ..spatial_computations.raytracer import RayTracer
from ..collision_checking.collision_detector import CollisionCheck
from ..collision_checking.trimesh_collision_detector import TrimeshCollisionDetector
from ..robots import RobotView, Camera, Manipulator, Finger, AbstractRobot
from ..spatial_types.spatial_types import Point3, TransformationMatrix
from ..world_description.world_entity import Body, Region, KinematicStructureEntity


def stable(obj: Body) -> bool:
    """
    Checks if an object is stable in the world. Stable meaning that it's position will not change after simulating
    physics in the World. This will be done by simulating the world for 10 seconds and compare
    the previous coordinates with the coordinates after the simulation.

    :param obj: The object which should be checked
    :return: True if the given object is stable in the world False else
    """
    raise NotImplementedError


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
    assert body1._world == body2._world, "Both bodies must be in the same world"
    tcd = TrimeshCollisionDetector(body1._world)
    result = tcd.check_collision_between_bodies(body1, body2)

    if result is None:
        return False
    return result.contact_distance < threshold


def robot_in_collision(
    robot: AbstractRobot,
    ignore_collision_with: Optional[List[Body]] = None,
    threshold: float = 0.001,
) -> bool:
    """
    Check if the robot collides with any object in the world at the given pose.

    :param robot: The robot object
    :param ignore_collision_with: A list of objects to ignore collision with
    :param threshold: The threshold for contact detection
    :return: True if the robot collides with any object, False otherwise
    """

    if ignore_collision_with is None:
        ignore_collision_with = []

    body = let("body", type_=Body, domain=robot._world.bodies)
    possible_collisions_bodies = an(
        entity(
            body,
            and_(
                body.has_collision(),
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
    return len(collisions) > 0


def robot_holds_body(robot: RobotView, body: Body) -> bool:
    """
    Check if a robot is holding an object.

    :param robot: The robot object
    :param body: The body to check if it is picked
    :return: True if the robot is holding the object, False otherwise
    """
    raise NotImplementedError


def get_visible_objects(camera: Camera) -> List[KinematicStructureEntity]:
    """
    Get all bodies and regions that are visible from the given camera using a segmentation mask.

    :param camera: The camera for which the visible objects should be returned
    :return: A list of bodies/regions that are visible from the camera
    """
    rt = RayTracer(camera._world)
    rt.update_scene()

    seg = rt.create_segmentation_mask(camera.root.global_pose.to_np(), resolution=256)
    print(seg)
    indices = np.unique(seg)
    indices = indices[indices != -1]
    bodies = [camera._world.kinematic_structure[i] for i in indices]
    return bodies

def visible(camera: Camera, obj: KinematicStructureEntity) -> bool:
    """
    Checks if a body/region is visible by the given camera.
    """
    return obj in get_visible_objects(camera)


def occluding_bodies(camera: Camera, body: Body) -> List[Body]:
    """
    Get all bodies that are occluding the given body.
    :param camera: The camera for which the occluding bodies should be returned
    :param body: The body for which the occluding bodies should be returned
    :return: A list of bodies that are occluding the given body.
    """
    raise NotImplementedError


def reachable(
    position: Point3, manipulator: Manipulator, threshold: float = 0.05
) -> bool:
    """
    Checks if a manipulator can reach a given position. To determine this the inverse kinematics are
    calculated and applied. Afterward the distance between the position and the given manipulator is calculated, if
    it is smaller than the threshold the reasoning query returns True, if not it returns False.

    :param position: The position to reach
    :param manipulator: The manipulator that should reach for the position
    :param threshold: The threshold between the end effector and the position.
    :return: True if the end effector is closer than the threshold to the target position, False in every other case
    """
    raise NotImplementedError


def blocking(position: Point3, manipulator: Manipulator) -> Optional[List[Body]]:
    """
    Checks if any objects are blocking another object when a robot tries to pick it. This works
    similar to the reachable predicate. First the inverse kinematics between the robot and the object will be
    calculated and applied. Then it will be checked if the robot is in contact with any object except the given one.
    If the given pose or Object is not reachable None will be returned

    :param position: The position to reach
    :param manipulator: The manipulator that should reach for the position
    :return: A list of bodies the robot is in collision with when reaching for the specified object or None if the pose or object is not reachable.
    """
    raise NotImplementedError


def supporting(supported_body: Body, supporting_body: Body) -> bool:
    """
    Checks if one object is supporting another object.

    :param supported_body: Object that is supported
    :param supporting_body: Object that potentially supports the first object
    :return: True if the second object is supported by the first object, False otherwise
    """
    raise NotImplementedError


def is_body_between_fingers(body: Body, fingers: List[Finger]) -> bool:
    """
    Check if the body is between the fingers.

    :param body: The body for which the check should be done.
    :param fingers: The fingers that should be checked.
    :return: True if the body is between the fingers, False otherwise
    """
    raise NotImplementedError


def is_body_in_region(body: Body, region: Region) -> bool:
    """
    Check if the body is in the region.

    :param body: The body for which the check should be done.
    :param region: The region to check if the body is in.
    """
    raise NotImplementedError


def left_of(body: Body, other: Body, reference_frame: TransformationMatrix) -> bool:
    """
    Check if the body is left of the other body.

    :param body: The body for which the check should be done.
    :param other: The other body.
    :param reference_frame: The reference frame in which the check
    :return: True if the body is left of the other body, False otherwise
    """
    ...


def right_of(body: Body, other: Body) -> bool:
    """
    Check if the body is right of the other body.

    :param body: The body for which the check should be done.
    :param other: The other body.
    :return: True if the body is right of the other body, False otherwise
    """
    ...


def above(body: Body, other: Body) -> bool:
    """
    Check if the body is above the other body.

    :param body: The body for which the check should be done.
    :param other: The other body.
    :return: True if the body is above the other body, False otherwise
    """


def below(body: Body, other: Body) -> bool:
    """
    Check if the body is below the other body.

    :param body: The body for which the check should be done.
    :param other: The other body.
    :return: True if the body is below the other body, False otherwise
    """
    ...
