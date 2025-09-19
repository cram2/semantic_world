from semantic_world.datastructures.prefixed_name import PrefixedName
from semantic_world.spatial_types.spatial_types import TransformationMatrix
from semantic_world.world import World
from semantic_world.world_description.connections import FixedConnection
from semantic_world.world_description.geometry import Sphere
from semantic_world.world_description.shape_collection import ShapeCollection
from semantic_world.world_description.world_entity import Body


def test_post_init_transformation():
    w = World()
    root = Body(name=PrefixedName("root"))
    b1 = Body(name=PrefixedName("b1"))

    with w.modify_world():
        w.add_connection(
            FixedConnection(
                parent=root,
                child=b1,
                _world=w,
                origin_expression=TransformationMatrix.from_xyz_rpy(
                    x=1, reference_frame=root
                ),
            )
        )

    shape = Sphere(
        radius=1, origin=TransformationMatrix.from_xyz_rpy(x=3, reference_frame=root)
    )
    shape_collection = ShapeCollection(
        shapes=[shape],
        reference_frame=b1,
    )
    shape_collection.transform_all_shapes_to_own_frame()
    assert shape.origin.reference_frame == b1
    assert shape.origin.to_position().x == 2.0

    shape = Sphere(
        radius=1, origin=TransformationMatrix.from_xyz_rpy(x=3, reference_frame=root)
    )

    shape_collection = ShapeCollection(reference_frame=b1)
    shape_collection.append(shape)
    shape_collection.transform_all_shapes_to_own_frame()
    assert shape.origin.reference_frame == b1
    assert shape.origin.to_position().x == 2.0
