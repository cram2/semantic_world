from typing_extensions import Dict, Any
from ripple_down_rules.rdr import GeneralRDR
from ripple_down_rules.datastructures.case import Case, create_case
from semantic_world.world import World
from . import world_views_mcrdr_of_grdr as world_views_classifier


classifiers_dict = dict()
classifiers_dict['World.views'] = world_views_classifier


def classify(case: World) -> Dict[str, Any]:
    if not isinstance(case, Case):
        case = create_case(case, max_recursion_idx=3)
    return GeneralRDR._classify(classifiers_dict, case)
