from ripple_down_rules.helpers import general_rdr_classify
from typing_extensions import Any, Dict
from ...world import World
from ripple_down_rules.datastructures.case import Case, create_case
from . import world_views_mcrdr as views_classifier

name = 'world'
case_type = World
case_name = 'World'
classifiers_dict = dict()
classifiers_dict['views'] = views_classifier


def classify(case: World, **kwargs) -> Dict[str, Any]:
    if not isinstance(case, Case):
        case = create_case(case, max_recursion_idx=3)
    return general_rdr_classify(classifiers_dict, case, **kwargs)
