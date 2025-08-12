import os
import time

import rclpy
from  entity_query_language.entity import an, entities
from entity_query_language.symbolic import in_
from sqlalchemy import create_engine, select
from sqlalchemy.orm import Session

from semantic_world.adapters.procthor.procthor_interface import ProcTHORInterface
from semantic_world.adapters.urdf import URDFParser
from semantic_world.adapters.viz_marker import VizMarkerPublisher
from semantic_world.geometry import Mesh
from semantic_world.orm.ormatic_interface import *
from semantic_world.spatial_types import TransformationMatrix
from semantic_world.world import World


def main():
    procthor_interface = ProcTHORInterface(
        base_url="https://user.informatik.uni-bremen.de/~luc_kro/procthor_environments/"
    )
    house_number = 1
    resource_path, sampled_world = procthor_interface.sample_environment(
        house_number, keep_environment=True
    )

    semantic_world_database_uri = os.environ.get("SEMANTIC_WORLD_DATABASE_URI")

    # Create database engine and session
    engine = create_engine(f"mysql+pymysql://{semantic_world_database_uri}")
    session = Session(engine)

    world = URDFParser(
        os.path.join(
            resource_path, f"dataset_house_{house_number}", f"{sampled_world}.urdf"
        )
    ).parse()
    dressers = [b for b in world.bodies if "dresser" in b.name.name.lower()]

    for dresser in dressers:
        collision: Mesh = dresser.collision[0]
        dresser_name = collision.filename.split("/")[-1].split(".")[0].lower()
        print(dresser_name)

        dressers = an(DresserDAO, domain=session.scalars(select(DresserDAO)).all())
        worlds = an(
            WorldMappingDAO, domain=session.scalars(select(WorldMappingDAO)).all()
        )

        result = entities(
            (worlds, dressers),
            (dressers.name.name == dresser_name) & (in_(dressers, worlds.views)),
        )
        result = next(result)[worlds]
        current_dresser_world: World = result.from_dao()

        #
        #
        # dressers_on_bodies = (
        #     select(BodyDAO.id)
        #     .join(ContainerDAO, ContainerDAO.body_id == BodyDAO.id)  # body ↔ container
        #     .join(
        #         DresserDAO, DresserDAO.container_id == ContainerDAO.id
        #     )  # container ↔ dresser
        #     .join(
        #         PrefixedNameDAO, DresserDAO.name_id == PrefixedNameDAO.id
        #     )  # dresser ↔ its PrefixedName
        #     .where(
        #         PrefixedNameDAO.name == dresser_name,
        #         BodyDAO.worldmappingdao_bodies_id == WorldMappingDAO.id,
        #     )  # body belongs to this world
        # )
        #
        # # outer query: worlds that have at least one such body
        # query = select(WorldMappingDAO).where(exists(dressers_on_bodies))
        #
        # result = session.scalars(query).one()

        parent_connection = dresser.parent_connection

        with world.modify_world():
            other_world_root = current_dresser_world.root
            parent_connection.child = other_world_root
            world.merge_world(current_dresser_world, parent_connection)
            world.remove_body(dresser)
            new_origin = TransformationMatrix.from_xyz_rpy(
                parent_connection.origin_expression.x,
                parent_connection.origin_expression.y,
                parent_connection.origin_expression.z,
            )
            parent_connection.origin_expression = new_origin

    rclpy.init()

    node = rclpy.create_node("viz_marker")

    p = VizMarkerPublisher(world, node)
    time.sleep(100)
    p._stop_publishing()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
