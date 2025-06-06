from ripple_down_rules.datastructures.case import Case, create_case
from typing_extensions import Optional, Set
from ripple_down_rules.utils import make_set
from .world_views_mcrdr_defs import *


attribute_name = 'views'
conclusion_type = (Door, Leg, Drawer, set, Surface, list, Roots, Handle, Cabinet, Container, Windows, Walls, Table, Fridge,)
mutually_exclusive = False


def classify(case: World, **kwargs) -> Set[Union[Door, Leg, Drawer, Surface, Roots, Handle, Cabinet, Container, Windows, Walls, Table, Fridge]]:
    if not isinstance(case, Case):
        case = create_case(case, max_recursion_idx=3)
    conclusions = set()

    if conditions_90574698325129464513441443063592862114(case):
        conclusions.update(make_set(conclusion_90574698325129464513441443063592862114(case)))

    if conditions_14920098271685635920637692283091167284(case):
        conclusions.update(make_set(conclusion_14920098271685635920637692283091167284(case)))

    if conditions_331345798360792447350644865254855982739(case):
        conclusions.update(make_set(conclusion_331345798360792447350644865254855982739(case)))

    if conditions_35528769484583703815352905256802298589(case):
        conclusions.update(make_set(conclusion_35528769484583703815352905256802298589(case)))

    if conditions_59112619694893607910753808758642808601(case):
        conclusions.update(make_set(conclusion_59112619694893607910753808758642808601(case)))

    if conditions_10840634078579061471470540436169882059(case):
        conclusions.update(make_set(conclusion_10840634078579061471470540436169882059(case)))

    if conditions_100363513934529269865524874913381333593(case):
        conclusions.update(make_set(conclusion_100363513934529269865524874913381333593(case)))

    if conditions_32110951838731034027817851716447618523(case):
        conclusions.update(make_set(conclusion_32110951838731034027817851716447618523(case)))

    if conditions_284429630184552508120710178948116682797(case):
        conclusions.update(make_set(conclusion_284429630184552508120710178948116682797(case)))

    if conditions_248275187658510121717149881382454303958(case):
        conclusions.update(make_set(conclusion_248275187658510121717149881382454303958(case)))

    if conditions_193930575337302892359597988013972475184(case):
        conclusions.update(make_set(conclusion_193930575337302892359597988013972475184(case)))

    if conditions_170324961522120215684260382462355539164(case):
        conclusions.update(make_set(conclusion_170324961522120215684260382462355539164(case)))
    return conclusions
