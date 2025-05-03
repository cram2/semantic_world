import logging
import os
import sys
from sqlacodegen.generators import TablesGenerator
from sqlalchemy import create_engine
from sqlalchemy.orm import registry, Session
from ormatic.ormatic import logger, ORMatic
from semantic_world.geometry import Color, Scale, Shape, Mesh, Primitive, Sphere, Cylinder, Box
from semantic_world.world import World, WorldEntity, Body, View, Connection

# ----------------------------------------------------------------------------------------------------------------------
# This script generates the ORM classes for the semantic_world package.
# Dataclasses can be mapped automatically to the ORM model
# using the ORMatic library, they just have to be registered in the classes list.
# Classes that are self_mapped and explicitly_mapped are already mapped in the model.py file. Look there for more
# information on how to map them.
# ----------------------------------------------------------------------------------------------------------------------

classes = [Color, Scale, Shape, Mesh, Primitive, Sphere, Cylinder, Box, World, WorldEntity, Body, View, Connection]

def generate_orm():
    """
    Generate the ORM classes for the pycram package.
    """
    # Set up logging
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))

    logger.addHandler(handler)
    logger.setLevel(logging.INFO)

    mapper_registry = registry()
    engine = create_engine('sqlite:///:memory:')
    session = Session(engine)

    # Create an ORMatic object with the classes to be mapped
    ormatic = ORMatic(classes, mapper_registry)

    # Generate the ORM classes
    ormatic.make_all_tables()

    # Create the tables in the database
    mapper_registry.metadata.create_all(session.bind)

    # Write the generated code to a file
    generator = TablesGenerator(mapper_registry.metadata, session.bind, [])

    path = os.path.abspath(os.path.join(os.getcwd(), '../src/semantic_world/orm/'))
    with open(os.path.join(path, 'ormatic_interface.py'), 'w') as f:
        ormatic.to_python_file(generator, f)


if __name__ == '__main__':
    generate_orm()