import re
from typing import List

import rustworkx as rx
import trimesh.boolean

from ...views.factories import (
    HandleFactory,
    ContainerFactory,
    Direction,
    DrawerFactory,
    DoorFactory,
    DresserFactory,
)
from ...geometry import Scale, TriangleMesh, Mesh
from ...pipeline.pipeline import Step
from ...prefixed_name import PrefixedName
from ...world import World
from ...world_entity import Body


def dresser_factory_replace(dresser: Body) -> DresserFactory:

    drawer_pattern = re.compile(r"^.*_drawer_.*$")
    door_pattern = re.compile(r"^.*_door_.*$")

    drawer_factories = []
    drawer_transforms = []
    door_factories = []
    door_transforms = []
    for child in dresser.child_bodies:
        if bool(drawer_pattern.fullmatch(child.name.name)):
            drawer_transforms.append(child.parent_connection.origin_expression)

            handle_factory = HandleFactory(
                name=PrefixedName(child.name.name + "_handle", child.name.prefix),
                scale=Scale(0.1, 0.1, 0.1),
            )
            container_factory = ContainerFactory(
                name=PrefixedName(child.name.name + "_container", child.name.prefix),
                scale=child.as_bounding_box_collection(dresser._world.root)
                .bounding_boxes[0]
                .scale,
                direction=Direction.Z,
            )
            drawer_factory = DrawerFactory(
                name=child.name,
                handle_factory=handle_factory,
                container_factory=container_factory,
            )
            drawer_factories.append(drawer_factory)
        elif bool(door_pattern.fullmatch(child.name.name)):
            door_transforms.append(child.parent_connection.origin_expression)
            handle_factory = HandleFactory(
                PrefixedName(child.name.name + "_handle", child.name.prefix), Scale(0.1, 0.1, 0.1)
            )

            door_factory = DoorFactory(
                name=child.name,
                scale=child.as_bounding_box_collection(dresser._world.root)
                .bounding_boxes[0]
                .scale,
                handle_factory=handle_factory,
                handle_direction=Direction.Y,
            )
            door_factories.append(door_factory)

    dresser_container_factory = ContainerFactory(
        name=PrefixedName(dresser.name.name + "_container", dresser.name.prefix),
        scale=dresser.as_bounding_box_collection(dresser._world.root)
        .bounding_boxes[0]
        .scale,
        direction=Direction.X,
    )
    dresser_factory = DresserFactory(
        name=dresser.name,
        container_factory=dresser_container_factory,
        drawers_factories=drawer_factories,
        drawer_transforms=drawer_transforms,
        door_factories=door_factories,
        door_transforms=door_transforms,
    )

    return dresser_factory

class ExcludeChildMeshesFromParentMeshes(Step):
    """
    This is not really tested.
    """

    def apply(self) -> World:
        for body_index in reversed(rx.topological_sort(self.world.kinematic_structure)):
            body: Body = self.world.kinematic_structure[body_index]
            children = body.recursive_child_bodies

            if len(children) == 0:
                continue

            child_meshes = [s.mesh.bounding_box for c in children for s in c.collision if isinstance(s, (TriangleMesh, Mesh))]

            own_mesh = [s.mesh for s in body.collision if isinstance(s, (TriangleMesh, Mesh))]
            own_mesh = trimesh.boolean.union(own_mesh)
            new_own_mesh = trimesh.boolean.difference([own_mesh, *child_meshes])

            body.collision = [TriangleMesh(mesh=new_own_mesh)]
        return self.world