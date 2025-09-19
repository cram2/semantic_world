import os
import unittest

import trimesh.boolean

from semantic_world.adapters.mesh import STLParser
from semantic_world.world_description.geometry import Box
from semantic_world.datastructures.prefixed_name import PrefixedName
from semantic_world.spatial_types.spatial_types import TransformationMatrix
from semantic_world.world_description.shape_collection import ShapeCollection
from semantic_world.world_description.world_entity import Body


class JSONTestCase(unittest.TestCase):

    def test_json_serialization(self):
        body = Body(name=PrefixedName("body"))
        collision = [
            Box(origin=TransformationMatrix.from_xyz_rpy(0, 1, 0, 0, 0, 1, body))
        ]
        body.collision = ShapeCollection(collision, reference_frame=body)

        json_data = body.to_json()
        body2 = Body.from_json(json_data)

        for c1 in body.collision:
            for c2 in body2.collision:
                self.assertEqual(c1, c2)

        self.assertEqual(body, body2)

    def test_json_serialization_with_mesh(self):
        body: Body = (
            STLParser(
                os.path.join(
                    os.path.dirname(__file__),
                    "..",
                    "..",
                    "resources",
                    "stl",
                    "milk.stl",
                )
            )
            .parse()
            .root
        )

        json_data = body.to_json()
        body2 = Body.from_json(json_data)

        for c1 in body.collision:
            for c2 in body2.collision:
                assert (trimesh.boolean.difference([c1.mesh, c2.mesh])).is_empty
