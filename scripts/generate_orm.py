import os
from enum import Enum

import semantic_world.degree_of_freedom
import semantic_world.geometry
import semantic_world.views.views
import semantic_world.world_entity
from ormatic.ormatic import ORMatic
from ormatic.utils import classes_of_module, recursive_subclasses
from semantic_world.connections import FixedConnection
from semantic_world.orm.model import *
from semantic_world.prefixed_name import PrefixedName
from semantic_world.world import *

# ----------------------------------------------------------------------------------------------------------------------
# This script generates the ORM classes for the semantic_world package.
# Dataclasses can be mapped automatically to the ORM model
# using the ORMatic library, they just have to be registered in the classes list.
# Classes that are self_mapped and explicitly_mapped are already mapped in the model.py file. Look there for more
# information on how to map them.
# ----------------------------------------------------------------------------------------------------------------------

# create of classes that should be mapped
classes = set(recursive_subclasses(AlternativeMapping))
classes |= set(classes_of_module(semantic_world.geometry))
classes |= set(classes_of_module(semantic_world.world))
classes |= set(classes_of_module(semantic_world.prefixed_name))
classes |= set(classes_of_module(semantic_world.world_entity))
classes |= set(classes_of_module(semantic_world.connections))
classes |= set(classes_of_module(semantic_world.views.views))
classes |= set(classes_of_module(semantic_world.degree_of_freedom))

# remove classes that should not be mapped
classes -= {ResetStateContextManager, WorldModelUpdateContextManager, HasUpdateState,
            World, ForwardKinematicsVisitor, Has1DOFState, DegreeOfFreedom}
classes -= set(recursive_subclasses(Enum))


def generate_orm():
    """
    Generate the ORM classes for the pycram package.
    """

    # Create an ORMatic object with the classes to be mapped
    ormatic = ORMatic(list(classes))

    # Generate the ORM classes
    ormatic.make_all_tables()

    path = os.path.abspath(os.path.join(os.getcwd(), '../src/semantic_world/orm/'))
    with open(os.path.join(path, 'ormatic_interface.py'), 'w') as f:
        ormatic.to_sqlalchemy_file(f)


if __name__ == '__main__':
    generate_orm()
