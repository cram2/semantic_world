from semantic_world.views.views import Handle
from semantic_world.connections import PrismaticConnection
from typing import Union
from semantic_world.connections import FixedConnection
from semantic_world.views.views import Container
from semantic_world.views.views import Drawer
from semantic_world.world import World
from semantic_world.views.views import Cabinet
from typing_extensions import Union


def conditions_90574698325129464513441443063592862114(case):
    return True


def conclusion_90574698325129464513441443063592862114(case):
    def get_value_for_world_views_of_type_handle(case: World) -> Union[set, list, Handle]:
        """Get possible value(s) for World.views of types list/set of Handle"""
        return [Handle(b) for b in case.bodies if "handle" in b.name.name.lower()]
    
    return get_value_for_world_views_of_type_handle(case)


def conditions_14920098271685635920637692283091167284(case):
    return len([v for v in case.views if type(v) is Handle]) > 0


def conclusion_14920098271685635920637692283091167284(case):
    def get_value_for_world_views_of_type_container(case: World) -> Union[set, Container, list]:
        """Get possible value(s) for World.views of types list/set of Container"""
        prismatic_connections = [c for c in case.connections if isinstance(c, PrismaticConnection)]
        fixed_connections = [c for c in case.connections if isinstance(c, FixedConnection)]
        children_of_prismatic_connections = [c.child for c in prismatic_connections]
        handles = [v for v in case.views if type(v) is Handle]
        fixed_connections_with_handle_child = [fc for fc in fixed_connections if fc.child in [h.body for h in handles]]
        drawer_containers = set(children_of_prismatic_connections).intersection(
            set([fc.parent for fc in fixed_connections_with_handle_child]))
        return [Container(b) for b in drawer_containers]
    
    return get_value_for_world_views_of_type_container(case)


def conditions_331345798360792447350644865254855982739(case):
    return len([v for v in case.views if type(v) is Handle]) > 0 and len(
        [v for v in case.views if type(v) is Container]) > 0


def conclusion_331345798360792447350644865254855982739(case):
    def get_value_for_world_views_of_type_drawer(case: World) -> Union[set, list, Drawer]:
        """Get possible value(s) for World.views of types list/set of Drawer"""
        handles = [v for v in case.views if type(v) is Handle]
        containers = [v for v in case.views if type(v) is Container]
        fixed_connections = [c for c in case.connections if
                             isinstance(c, FixedConnection) and c.parent in [cont.body for cont in
                                                                             containers] and c.child in [
                                 h.body for h in handles]]
        prismatic_connections = [c for c in case.connections if
                                 isinstance(c, PrismaticConnection) and c.child in [cont.body for cont in containers]]
        drawer_handle_connections = [fc for fc in fixed_connections if
                                     fc.parent in [pc.child for pc in prismatic_connections]]
        drawers = [Drawer([cont for cont in containers if dc.parent == cont.body][0],
                          [h for h in handles if dc.child == h.body][0]) for dc in drawer_handle_connections]
        return drawers
    
    return get_value_for_world_views_of_type_drawer(case)


def conditions_35528769484583703815352905256802298589(case):
    return len([v for v in case.views if type(v) is Drawer]) > 0


def conclusion_35528769484583703815352905256802298589(case):
    def get_value_for_world_views_of_type_cabinet(case: World) -> Union[set, Cabinet, list]:
        """Get possible value(s) for World.views of types list/set of Cabinet"""
        drawers = [v for v in case.views if type(v) is Drawer]
        prismatic_connections = [c for c in case.connections if
                                 isinstance(c, PrismaticConnection) and c.child in [drawer.container.body for drawer in
                                                                                    drawers]]
        cabinet_container_bodies = [pc.parent for pc in prismatic_connections]
        cabinets = []
        for ccb in cabinet_container_bodies:
            if ccb in [cabinet.container.body for cabinet in cabinets]:
                continue
            cc_prismatic_connections = [pc for pc in prismatic_connections if pc.parent is ccb]
            cabinet_drawer_container_bodies = [pc.child for pc in cc_prismatic_connections]
            cabinet_drawers = [d for d in drawers if d.container.body in cabinet_drawer_container_bodies]
            cabinets.append(Cabinet(Container(ccb), cabinet_drawers))
    
        return cabinets
    
    return get_value_for_world_views_of_type_cabinet(case)


