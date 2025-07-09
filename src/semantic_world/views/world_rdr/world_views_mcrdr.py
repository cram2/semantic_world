from ripple_down_rules.datastructures.case import Case, create_case
from ripple_down_rules.utils import make_set
from typing_extensions import Optional, Set
from .world_views_mcrdr_defs import *


attribute_name = 'views'
conclusion_type = (Armchair, Windows, Surface, Cooktop, Roots, DetailedTable, Countertop, Walls, Sofa, Handle, Hotplates, Table, Oven, Sides, Leg, Drawer, list, Sink, Door, set, Cabinet, Container, DetailedCooktop, Fridge,)
mutually_exclusive = False


def classify(case: World, **kwargs) -> Set[Union[Armchair, Windows, Surface, Cooktop, Roots, DetailedTable, Countertop, Walls, Sofa, Handle, Hotplates, Table, Oven, Sides, Leg, Drawer, Sink, Door, Cabinet, Container, DetailedCooktop, Fridge]]:
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

    if conditions_60036782519178869340024639996969746162(case):
        conclusions.update(make_set(conclusion_60036782519178869340024639996969746162(case)))

    if conditions_288834628390404953380091650362350670286(case):
        conclusions.update(make_set(conclusion_288834628390404953380091650362350670286(case)))

    if conditions_4158886126664067940413153133916413099(case):
        conclusions.update(make_set(conclusion_4158886126664067940413153133916413099(case)))

    if conditions_108388395825945088225234017461290017170(case):
        conclusions.update(make_set(conclusion_108388395825945088225234017461290017170(case)))

    if conditions_264797936917281996243473352554665640669(case):
        conclusions.update(make_set(conclusion_264797936917281996243473352554665640669(case)))

    if conditions_307594597833937094056821330832591786276(case):
        conclusions.update(make_set(conclusion_307594597833937094056821330832591786276(case)))

    if conditions_12040197202477135811012547523782725108(case):
        conclusions.update(make_set(conclusion_12040197202477135811012547523782725108(case)))

    if conditions_271228146146808625034016035840195655394(case):
        conclusions.update(make_set(conclusion_271228146146808625034016035840195655394(case)))

    if conditions_63632553025287441678606225860106156564(case):
        conclusions.update(make_set(conclusion_63632553025287441678606225860106156564(case)))

    if conditions_79493164061606858785270566977975488425(case):
        conclusions.update(make_set(conclusion_79493164061606858785270566977975488425(case)))

    if conditions_318258669409180299293189838893993003603(case):
        conclusions.update(make_set(conclusion_318258669409180299293189838893993003603(case)))

    if conditions_22893914216819585647057541370007578525(case):
        conclusions.update(make_set(conclusion_22893914216819585647057541370007578525(case)))
    return conclusions
