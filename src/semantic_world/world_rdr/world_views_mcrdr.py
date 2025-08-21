from ripple_down_rules.utils import make_set
from typing_extensions import Optional, Set
from ripple_down_rules.helpers import get_an_updated_case_copy
from ripple_down_rules.datastructures.case import Case, create_case
from .world_views_mcrdr_defs import *


attribute_name = 'views'
conclusion_type = (Countertop, Table, Cooktop, Oven, Drawer, Container, Root, Sofa, Window, Hotplate, Armchair, list, Fridge, Sides, Sink, Cabinet, Leg, Surface, Door, Handle, set, Wall,)
mutually_exclusive = False


def classify(case: World, **kwargs) -> Set[Union[Countertop, Table, Cooktop, Oven, Drawer, Container, Root, Sofa, Window, Hotplate, Armchair, Fridge, Sides, Sink, Cabinet, Leg, Surface, Door, Handle, Wall]]:
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

    if conditions_298893002154434174914882697804321727044(case):
        conclusions.update(make_set(conclusion_298893002154434174914882697804321727044(case)))

    if conditions_95749960275749164559369305418046530921(case):
        conclusions.update(make_set(conclusion_95749960275749164559369305418046530921(case)))

    if conditions_56044667384119005486660532922350093576(case):
        conclusions.update(make_set(conclusion_56044667384119005486660532922350093576(case)))

    if conditions_192414996332569377000417356221731692718(case):
        conclusions.update(make_set(conclusion_192414996332569377000417356221731692718(case)))

    if conditions_295145914874381034058732290061016216922(case):
        conclusions.update(make_set(conclusion_295145914874381034058732290061016216922(case)))

    if conditions_76734351718414165115161689080544922137(case):
        conclusions.update(make_set(conclusion_76734351718414165115161689080544922137(case)))

    if conditions_276870538550900219151385672580612993791(case):
        conclusions.update(make_set(conclusion_276870538550900219151385672580612993791(case)))

    if conditions_272346104137726532028618426380451371764(case):
        conclusions.update(make_set(conclusion_272346104137726532028618426380451371764(case)))

    if conditions_125114902106458283648997265474531641579(case):
        conclusions.update(make_set(conclusion_125114902106458283648997265474531641579(case)))

    if conditions_39216173054479960152579027281679377283(case):
        conclusions.update(make_set(conclusion_39216173054479960152579027281679377283(case)))

    if conditions_56932022228577464889761478858979829689(case):
        conclusions.update(make_set(conclusion_56932022228577464889761478858979829689(case)))

    if conditions_304562387165012343799590402190050617477(case):
        conclusions.update(make_set(conclusion_304562387165012343799590402190050617477(case)))

    if conditions_49636263552498546597487129028109645395(case):
        conclusions.update(make_set(conclusion_49636263552498546597487129028109645395(case)))

    if conditions_180013694937072261831339544475640316362(case):
        conclusions.update(make_set(conclusion_180013694937072261831339544475640316362(case)))
    return conclusions
