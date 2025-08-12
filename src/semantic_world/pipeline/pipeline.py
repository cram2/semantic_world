import re
from abc import ABC
from dataclasses import dataclass, field
from enum import StrEnum
from typing import List, Optional, Callable

import coacd
import numpy as np
import trimesh

from semantic_world.views.factories import ViewFactory
from semantic_world.geometry import TriangleMesh, Mesh
from semantic_world.world import World
from semantic_world.world_entity import Body


@dataclass
class Step(ABC):
    world: Optional[World] = field(init=False, default=None)

    def apply(self) -> World:
        raise NotImplementedError()


@dataclass
class Pipeline:

    steps: List[Step]

    def apply(self, world: Optional[World] = None) -> World:
        for step in self.steps:
            step.world = world
            world = step.apply()
        return world

@dataclass
class BodyFilter(Step):
    condition: Callable[[Body], bool]

    def apply(self) -> World:
        for body in self.world.bodies:
            if not self.condition(body):
                self.world.remove_body(body)
        return self.world


class ApproximationMode(StrEnum):
    """
    Approximation shape type
    """
    BOX = "box"
    CONVEX_HULL = "ch"

class PreprocessingMode(StrEnum):
    """
    Manifold preprocessing mode
    """

    AUTO = "auto"
    """
    Automatically chose based on the geometry.
    """

    ON = "on"
    """
    Force turn on the pre-processing
    """

    OFF = "off"
    """
    Force turn off the pre-processing
    """


@dataclass
class COACDMeshDecomposer(Step):
    """
    COACDMeshDecomposer is a class for decomposing complex 3D meshes into simpler convex components
    using the COACD (Convex Optimization for Approximate Convex Decomposition) algorithm. It is
    designed to preprocess, analyze, and process 3D meshes with a focus on efficiency and scalability
    in fields such as robotics, gaming, and simulation.

    Check https://github.com/SarahWeiii/CoACD for further details.
    """

    threshold: float = 0.05
    """
    Concavity threshold for terminating the decomposition (0.01 - 1)
    """

    max_convex_hull: Optional[int] = None
    """
    Maximum number of convex hulls in the result. 
    Works only when merge is enabled (may introduce convex hull with a concavity larger than the threshold)
    """

    preprocess_mode: PreprocessingMode = PreprocessingMode.AUTO
    """
    Manifold preprocessing mode.
    """

    preprocess_resolution: int = 50
    """
    Resolution for manifold preprocess (20~100)
    """

    resolution: int = 2000
    """
    Sampling resolution for Hausdorff distance calculation (1 000 - 10 000)
    """

    search_nodes: int = 20
    """
    Max number of child nodes in the monte carlo tree search (10 - 40).
    """

    search_iterations: int = 150
    """
    Number of search iterations in the monte carlo tree search (60 - 2000).
    """

    search_depth: int = 3
    """
    Maximum search depth in the monte carlo tree search (2 - 7).
    """

    pca: bool = False
    """
    Enable PCA pre-processing
    """

    merge: bool = True
    """
    Enable merge postprocessing.
    """

    max_convex_hull_vertices: Optional[int] = None
    """
    Maximum vertex value for each convex hull, only when decimate is enabled.
    """

    extrude_margin: Optional[float ] = None
    """
    Extrude margin, only when extrude is enabled
    """

    approximation_mode: ApproximationMode = ApproximationMode.BOX
    """
    Approximation mode to use.
    """

    seed: int = field(default_factory=lambda: np.random.randint(2 ** 32))
    """
    Random seed used for sampling.
    """

    def apply(self) -> World:
        for body in self.world.bodies:
            new_geometry = []

            for shape in body.visual:
                if isinstance(shape, (Mesh, TriangleMesh)):
                    mesh = shape.mesh

                    mesh = coacd.Mesh(mesh.vertices, mesh.faces)
                    if self.max_convex_hull is not None:
                        max_convex_hull = self.max_convex_hull
                    else:
                        max_convex_hull = -1
                    parts = coacd.run_coacd(mesh=mesh, apx_mode=str(self.approximation_mode), threshold=self.threshold,
                                            max_convex_hull=max_convex_hull, preprocess_mode=str(self.preprocess_mode),
                                            resolution=self.resolution, mcts_nodes=self.search_nodes,
                                            mcts_iterations=self.search_iterations,
                                            mcts_max_depth=self.search_depth,
                                            pca=self.pca,
                                            merge=self.merge,
                                            decimate=self.max_convex_hull_vertices is not None,
                                            max_ch_vertex=self.max_convex_hull_vertices or 256,
                                            extrude=self.extrude_margin is not None,
                                            extrude_margin=self.extrude_margin or 0.01,
                                            seed=self.seed
                                            )

                    for vs, fs in parts:
                        new_geometry.append(TriangleMesh(mesh=trimesh.Trimesh(vs, fs), origin=shape.origin))
                else:
                    new_geometry.append(shape)

            body.collision = new_geometry

        return self.world

@dataclass
class BodyFactoryReplace(Step):
    body_condition: Callable[[Body], bool] = lambda x: bool(re.compile(r'^dresser_\d+.*$').fullmatch(x.name.name))
    factory_creator: Callable[[Body], ViewFactory] = None


    def apply(self) -> World:

        filtered_bodies =[body for body in self.world.bodies if self.body_condition(body)]

        for body in filtered_bodies:

            factory = self.factory_creator(body)

            parent_connection = body.parent_connection
            if parent_connection is None:
                return factory.create()

            with self.world.modify_world():
                [self.world.remove_body(b) for b in body.recursive_child_bodies]
                self.world.remove_body(body)

            new_world = factory.create()
            self.world.merge_world(new_world, parent_connection)



        return self.world
