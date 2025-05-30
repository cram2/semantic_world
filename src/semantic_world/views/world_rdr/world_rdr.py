from typing_extensions import Dict, Any
from ripple_down_rules.helpers import general_rdr_classify
from ripple_down_rules.datastructures.case import Case, create_case
from semantic_world.world import World
from . import world_views_mcrdr as views_classifier


classifiers_dict = dict()
classifiers_dict['views'] = views_classifier


def classify(case: World, **kwargs) -> Dict[str, Any]:
    if not isinstance(case, Case):
        case = create_case(case, max_recursion_idx=3)
    return general_rdr_classify(classifiers_dict, case, **kwargs)
