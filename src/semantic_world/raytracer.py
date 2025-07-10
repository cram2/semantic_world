from typing import Tuple, List

import numpy as np
import trimesh
from trimesh import Scene

from .world_entity import Body


class RayTracer:

    def __init__(self, world):
        """
        Initializes the RayTracer with the given world.

        :param world: The world to use for ray tracing.
        """
        self.world = world
        self._last_world_model = -1
        self._last_world_state = -1
        self.index_to_body = {}
        self.scene_to_index = {}

        self.scene = Scene()
        self.update_scene()

    def update_scene(self):
        """
        Updates the ray tracer scene with the current state of the world.
        This method should be called whenever the world changes to ensure the ray tracer has the latest information.
        """
        if self._last_world_model is not self.world._model_version:
            self.add_missing_bodies()
            self._last_world_model = self.world._model_version
        if self._last_world_state is not self.world._state_version:
            self.update_transforms()
            self._last_world_state = self.world._state_version

    def add_missing_bodies(self):
        """
        Adds all bodies from the world to the ray tracer scene that are not already present.
        """
        bodies_to_add = [body for body in self.world.bodies if body.name.name not in "\t".join(self.scene.graph.nodes)]
        for body in bodies_to_add:
            for i, collision in enumerate(body.collision):
                self.scene.add_geometry(collision.mesh,
                                        node_name=body.name.name + f"_collision_{i}",
                                        parent_node_name="world",
                                        transform=self.world.compute_forward_kinematics_np(
                                            self.world.root,
                                            body) @ collision.origin.to_np())
                self.scene_to_index[body.name.name + f"_collision_{i}"] = body.index
                self.index_to_body[body.index] = body

    def update_transforms(self):
        """
        Updates the transforms of all bodies in the ray tracer scene.
        This is necessary to ensure that the ray tracing uses the correct positions and orientations.
        """
        for body in self.world.bodies:
            for i, collision in enumerate(body.collision):
                transform = self.world.compute_forward_kinematics_np(self.world.root,
                                                                     body) @ collision.origin.to_np()
                self.scene.graph[body.name.name + f"_collision_{i}"] = transform

    def create_segmentation_mask(self, camera_position: np.ndarray,
                                 resolution: int = 512) -> np.ndarray:
        """
        Creates a segmentation mask for the ray tracer scene from the camera position to the target position. Each pixel
        in the mask corresponds to the index of a  body in the scene or -1 if no body is hit at that pixel.
<
        :param camera_position: The position of the camera.
        :param resolution: The resolution of the segmentation mask.
        :return: A segmentation mask as a numpy array.
        """
        self.update_scene()
        ray_origins, ray_directions, pixels = self.create_camera_rays(camera_position, resolution=resolution)
        points, index_ray, index_tri = self.scene.to_mesh().ray.intersects_location(ray_origins, ray_directions,
                                                                                       multiple_hits=False)
        bodies = self.scene.triangles_node[index_tri]

        # map the name of the scene objects to the index
        bodies = [self.scene_to_index[body] for body in bodies]

        pixel_ray = pixels[index_ray]

        # create a numpy array we can turn into an image
        # doing it with uint8 creates an `L` mode greyscale image
        a = np.zeros(self.scene.camera.resolution, dtype=np.int8) - 1

        # assign bodies to correct pixel locations
        a[pixel_ray[:, 0], pixel_ray[:, 1]] = bodies

        return a

    def create_depth_map(self, camera_position: np.ndarray,
                         resolution: int = 512) -> np.ndarray:
        """
        Creates a depth map for the ray tracer scene from the camera position to the target position. Each pixel in the
        depth map corresponds to the distance from the camera to the closest point on the surface of the scene or -1 if
        no point is hit.

        :param camera_position: The position of the camera.
        :param resolution: The resolution of the depth map.
        :return: A depth map as a numpy array.
        """
        self.update_scene()
        ray_origins, ray_directions, pixels = self.create_camera_rays(camera_position, resolution=resolution)
        # Code from the example in trimesh repo: examples/raytrace.py
        points, index_ray, index_tri = self.scene.to_mesh().ray.intersects_location(ray_origins, ray_directions,
                                                                                    multiple_hits=False)
        depth = trimesh.util.diagonal_dot(points - ray_origins[0], ray_directions[index_ray])
        pixel_ray = pixels[index_ray]

        # create a numpy array we can turn into an image
        # doing it with uint8 creates an `L` mode greyscale image
        a = np.zeros(self.scene.camera.resolution, dtype=np.float32) - 1

        # assign depth to correct pixel locations
        a[pixel_ray[:, 0], pixel_ray[:, 1]] = depth

        return a

    def create_camera_rays(self, camera_poe: np.ndarray,
                           resolution: int = 512, fov=90) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Creates camera rays for the ray tracer scene from the camera position to the target position. Places the camera
        at the given position and orientation view of the camera is along the x-axis.

        :param camera_poe: The position of the camera as a 4x4 transformation matrix.
        :param resolution: The resolution of the camera rays.
        :param fov: The field of view of the camera in degrees.
        :return: The origin points of the rays, the direction vectors of the rays, and the pixel coordinates.
        """
        self.update_scene()
        self.scene.camera.resolution = (resolution, resolution)
        # By default, the camera is looking along the -z axis, so we need to rotate it to look along the x-axis.
        rotate = trimesh.transformations.rotation_matrix(
            angle=np.radians(-90.0), direction=[0, 1, 0], point=self.scene.centroid
        )
        rotate_x = trimesh.transformations.rotation_matrix(
            angle=np.radians(180.0), direction=[1, 0, 0], point=self.scene.centroid
        )

        self.scene.camera.fov = (fov, fov)
        self.scene.camera.resolution = [resolution, resolution]
        self.scene.graph[self.scene.camera.name] = camera_poe @ rotate_x @ rotate

        return self.scene.camera_rays()

    def ray_test(self, origin_points: np.ndarray, target_points: np.ndarray) -> Tuple[np.ndarray, np.ndarray, List[Body]]:
        """
        Performs a ray test from the origin point to the target point in the ray tracer scene.

        :param origin_points: The starting point of the ray.
        :param target_points: The end point of the ray.
        :return: A tuple containing the points where the ray intersects and the indices of rays that hit the scene as well as the bodies that were.
        """
        self.update_scene()
        origin_points = origin_points.reshape((-1, 3))
        target_points = target_points.reshape((-1, 3))
        if origin_points.shape != target_points.shape:
            raise ValueError("Origin and target points must have the same shape.")

        ray_directions = target_points - origin_points
        points, index_ray, index_tri = self.scene.to_mesh().ray.intersects_location(origin_points, ray_directions,
                                                                                   multiple_hits=False)
        bodies = self.scene.triangles_node[index_tri]

        # map the name of the scene objects to the index
        bodies = [self.index_to_body[self.scene_to_index[body]] for body in bodies]

        return points, index_ray, bodies
