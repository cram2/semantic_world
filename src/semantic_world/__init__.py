__version__ = "0.0.3"


import logging

from .reasoner import WorldReasoner

logger = logging.Logger("semantic_world")
logger.setLevel(logging.INFO)

from .connections import *
from .views import *
from .robots import *
