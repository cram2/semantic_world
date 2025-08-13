from typing_extensions import List, Set, Union
from ..world import World
from ..views import Armchair, Cabinet, Container, Cooktop, Countertop, Door, Drawer, Fridge, Handle, Hotplate, Leg, Oven, Root, Sides, Sink, Sofa, Surface, Table, Wall, Window
from ..connections import FixedConnection, PrismaticConnection, RevoluteConnection


def conditions_90574698325129464513441443063592862114(case) -> bool:
    def has_bodies_named_handle(case: World) -> bool:
        """Get conditions on whether it's possible to conclude a value for World.views  of type Handle."""
        return any("handle" in b.name.name.lower() for b in case.bodies)
    return has_bodies_named_handle(case)


def conclusion_90574698325129464513441443063592862114(case) -> List[Handle]:
    def get_handles(case: World) -> Union[set, list, Handle]:
        """Get possible value(s) for World.views of types list/set of Handle"""
        return [Handle(b) for b in case.bodies if "handle" in b.name.name.lower()]
    
    return get_handles(case)


def conditions_14920098271685635920637692283091167284(case) -> bool:
    def has_handles_and_fixed_and_prismatic_connections(case: World) -> bool:
        """Get conditions on whether it's possible to conclude a value for World.views  of type Container."""
        return (any(v for v in case.views if type(v) is Handle) and
                any(c for c in case.connections if isinstance(c, PrismaticConnection)) and
                any(c for c in case.connections if isinstance(c, FixedConnection)))
    return has_handles_and_fixed_and_prismatic_connections(case)


def conclusion_14920098271685635920637692283091167284(case) -> List[Container]:
    def get_containers(case: World) -> Union[set, Container, list]:
        """Get possible value(s) for World.views of types list/set of Container"""
        prismatic_connections = [c for c in case.connections if isinstance(c, PrismaticConnection)]
        fixed_connections = [c for c in case.connections if isinstance(c, FixedConnection)]
        children_of_prismatic_connections = [c.child for c in prismatic_connections]
        handles = [v for v in case.views if type(v) is Handle]
        fixed_connections_with_handle_child = [fc for fc in fixed_connections if fc.child in [h.body for h in handles]]
        drawer_containers = set(children_of_prismatic_connections).intersection(
            set([fc.parent for fc in fixed_connections_with_handle_child]))
        return [Container(b) for b in drawer_containers]
    
    return get_containers(case)


def conditions_331345798360792447350644865254855982739(case) -> bool:
    def has_handles_and_containers(case: World) -> bool:
        """Get conditions on whether it's possible to conclude a value for World.views  of type Drawer."""
        return any(v for v in case.views if type(v) is Handle) and any(v for v in case.views if type(v) is Container)
    return has_handles_and_containers(case)


def conclusion_331345798360792447350644865254855982739(case) -> List[Drawer]:
    def get_drawers(case: World) -> Union[set, list, Drawer]:
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
    
    return get_drawers(case)


def conditions_35528769484583703815352905256802298589(case) -> bool:
    def has_drawers(case: World) -> bool:
        """Get conditions on whether it's possible to conclude a value for World.views  of type Cabinet."""
        return any(v for v in case.views if type(v) is Drawer)
    return has_drawers(case)


def conclusion_35528769484583703815352905256802298589(case) -> List[Cabinet]:
    def get_cabinets(case: World) -> Union[set, Cabinet, list]:
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
    
    return get_cabinets(case)


def conditions_59112619694893607910753808758642808601(case) -> bool:
    def has_handles_and_revolute_connections(case: World) -> bool:
        """Get conditions on whether it's possible to conclude a value for World.views  of type Door."""
        return (any(v for v in case.views if isinstance(v, Handle)) and
                any(c for c in case.connections if isinstance(c, RevoluteConnection)))
    return has_handles_and_revolute_connections(case)


def conclusion_59112619694893607910753808758642808601(case) -> List[Door]:
    def get_doors(case: World) -> List[Door]:
        """Get possible value(s) for World.views  of type Door."""
        handles = [v for v in case.views if isinstance(v, Handle)]
        handle_bodies = [h.body for h in handles]
        connections_with_handles = [c for c in case.connections if isinstance(c, FixedConnection) and
                                    c.child in handle_bodies]
    
        revolute_connections = [c for c in case.connections if isinstance(c, RevoluteConnection)]
        bodies_connected_to_handles = [c.parent if c.child in handle_bodies else c.child for c in connections_with_handles]
        bodies_that_have_revolute_joints = [b for b in bodies_connected_to_handles for c in revolute_connections
                                            if b == c.child]
        body_handle_connections = [c for c in connections_with_handles if c.parent in bodies_that_have_revolute_joints]
        doors = [Door(c.parent, [h for h in handles if h.body == c.child][0]) for c in body_handle_connections]
        return doors
    return get_doors(case)


def conditions_10840634078579061471470540436169882059(case) -> bool:
    def has_doors_with_fridge_in_their_name(case: World) -> bool:
        """Get conditions on whether it's possible to conclude a value for World.views  of type Fridge."""
        return any(v for v in case.views if isinstance(v, Door) and "fridge" in v.body.name.name.lower())
    return has_doors_with_fridge_in_their_name(case)


def conclusion_10840634078579061471470540436169882059(case) -> List[Fridge]:
    def get_fridges(case: World) -> List[Fridge]:
        """Get possible value(s) for World.views  of type Fridge."""
        # Get fridge-related doors
        fridge_doors = [v for v in case.views if isinstance(v, Door) and "fridge" in v.body.name.name.lower()]
        # Precompute bodies of the fridge doors
        fridge_doors_bodies = [d.body for d in fridge_doors]
        # Filter relevant revolute connections
        fridge_door_connections = [
            c for c in case.connections
            if isinstance(c, RevoluteConnection)
               and c.child in fridge_doors_bodies
               and 'fridge' in c.parent.name.name.lower()
        ]
        return [Fridge(c.parent, fridge_doors[fridge_doors_bodies.index(c.child)]) for c in fridge_door_connections]
    return get_fridges(case)


def conditions_298893002154434174914882697804321727044(case) -> bool:
    def conditions_for_world_views_of_type_root(case: World) -> bool:
        """Get conditions on whether it's possible to conclude a value for World.views  of type Root."""
        return len([Root(r) for r in case.bodies if "root" in r.name.name.lower()]) > 0
    return conditions_for_world_views_of_type_root(case)


def conclusion_298893002154434174914882697804321727044(case) -> List[Root]:
    def world_views_of_type_root(case: World) -> List[Root]:
        """Get possible value(s) for World.views  of type Root."""
        return [Root(r) for r in case.bodies if "root" in r.name.name.lower()]
    return world_views_of_type_root(case)


def conditions_95749960275749164559369305418046530921(case) -> bool:
    def conditions_for_world_views_of_type_table(case: World) -> bool:
        """Get conditions on whether it's possible to conclude a value for World.views  of type Table."""
        return len([r for r in case.views if isinstance(r, Root)]) > 0
    return conditions_for_world_views_of_type_table(case)


def conclusion_95749960275749164559369305418046530921(case) -> List[Table]:
    def world_views_of_type_table(case: World) -> List[Table]:
        """Get possible value(s) for World.views  of type Table."""
        all_root = [r for r in case.views if isinstance(r, Root)]
        root_bodies = [r.body for r in all_root]
        conections_with_root = [
            c for c in case.connections
            if isinstance(c, FixedConnection)
               and c.parent in root_bodies
        ]
        return [Table(b.child) for b in conections_with_root if "table" in b.child.name.name.lower()]
    return world_views_of_type_table(case)


def conditions_56044667384119005486660532922350093576(case) -> bool:
    def conditions_for_world_views_of_type_window(case: World) -> bool:
        """Get conditions on whether it's possible to conclude a value for World.views  of type Window."""
        return len([r for r in case.views if isinstance(r, Root)]) > 0
    return conditions_for_world_views_of_type_window(case)


def conclusion_56044667384119005486660532922350093576(case) -> List[Window]:
    def world_views_of_type_window(case: World) -> List[Window]:
        """Get possible value(s) for World.views  of type Window."""
        all_root = [r for r in case.views if isinstance(r, Root)]
        root_bodies = [r.body for r in all_root]
        conections_with_root = [
            c for c in case.connections
            if isinstance(c, FixedConnection)
               and c.parent in root_bodies
        ]
        return [Window(b.child) for b in conections_with_root if "window" in b.child.name.name.lower()]
    return world_views_of_type_window(case)


def conditions_192414996332569377000417356221731692718(case) -> bool:
    def conditions_for_world_views_of_type_wall(case: World) -> bool:
        """Get conditions on whether it's possible to conclude a value for World.views  of type Wall."""
        return len([r for r in case.views if isinstance(r, Root)]) > 0
    return conditions_for_world_views_of_type_wall(case)


def conclusion_192414996332569377000417356221731692718(case) -> List[Wall]:
    def world_views_of_type_wall(case: World) -> List[Wall]:
        """Get possible value(s) for World.views  of type Wall."""
        all_root = [r for r in case.views if isinstance(r, Root)]
        root_bodies = [r.body for r in all_root]
        conections_with_root = [
            c for c in case.connections
            if isinstance(c, FixedConnection)
               and c.parent in root_bodies
        ]
        return [Wall(b.child) for b in conections_with_root if "wall" in b.child.name.name.lower()]
    return world_views_of_type_wall(case)


def conditions_295145914874381034058732290061016216922(case) -> bool:
    def conditions_for_world_views_of_type_surface(case: World) -> bool:
        """Get conditions on whether it's possible to conclude a value for World.views  of type Surface."""
        return True
    return conditions_for_world_views_of_type_surface(case)


def conclusion_295145914874381034058732290061016216922(case) -> List[Surface]:
    def world_views_of_type_surface(case: World) -> List[Surface]:
        """Get possible value(s) for World.views  of type Surface."""
        return [Surface(b) for b in case.bodies if "surface" in b.name.name.lower()]
    return world_views_of_type_surface(case)


def conditions_76734351718414165115161689080544922137(case) -> bool:
    def conditions_for_world_views_of_type_leg(case: World) -> bool:
        """Get conditions on whether it's possible to conclude a value for World.views  of type Leg."""
        return True
    return conditions_for_world_views_of_type_leg(case)


def conclusion_76734351718414165115161689080544922137(case) -> List[Leg]:
    def world_views_of_type_leg(case: World) -> List[Leg]:
        """Get possible value(s) for World.views  of type Leg."""
        all_surfaces = [Surface(b) for b in case.bodies if "surface" in b.name.name.lower()]
        surfaces_bodies = [r.body for r in all_surfaces]
        conections_with_surface = [
            c for c in case.connections
            if isinstance(c, FixedConnection)
               and c.parent in surfaces_bodies
        ]
    
        return [Leg(b.child) for b in conections_with_surface if "leg" in b.child.name.name.lower()]
    return world_views_of_type_leg(case)



def conditions_276870538550900219151385672580612993791(case) -> bool:
    def conditions_for_world_views_of_type_armchair(case: World) -> bool:
        """Get conditions on whether it's possible to conclude a value for World.views  of type Armchair."""
        return len([r for r in case.views if isinstance(r, Root)]) > 0
    return conditions_for_world_views_of_type_armchair(case)


def conclusion_276870538550900219151385672580612993791(case) -> List[Armchair]:
    def world_views_of_type_armchair(case: World) -> List[Armchair]:
        """Get possible value(s) for World.views  of type Armchair."""
        all_root = [r for r in case.views if isinstance(r, Root)]
        root_bodies = [r.body for r in all_root]
        conections_with_root = [
            c for c in case.connections
            if isinstance(c, FixedConnection)
               and c.parent in root_bodies
        ]
        return [Armchair(b.child) for b in conections_with_root if "armchair" in b.child.name.name.lower()]
    return world_views_of_type_armchair(case)


def conditions_272346104137726532028618426380451371764(case) -> bool:
    def conditions_for_world_views_of_type_sofa(case: World) -> bool:
        """Get conditions on whether it's possible to conclude a value for World.views  of type Sofa."""
        return len([r for r in case.views if isinstance(r, Root)]) > 0
    return conditions_for_world_views_of_type_sofa(case)


def conclusion_272346104137726532028618426380451371764(case) -> List[Sofa]:
    def world_views_of_type_sofa(case: World) -> List[Sofa]:
        """Get possible value(s) for World.views  of type Sofa."""
        all_root = [r for r in case.views if isinstance(r, Root)]
        root_bodies = [r.body for r in all_root]
        conections_with_root = [
            c for c in case.connections
            if isinstance(c, FixedConnection)
               and c.parent in root_bodies
        ]
        return [Sofa(b.child) for b in conections_with_root if "sofa" in b.child.name.name.lower()]
    return world_views_of_type_sofa(case)


def conditions_125114902106458283648997265474531641579(case) -> bool:
    def conditions_for_world_views_of_type_oven(case: World) -> bool:
        """Get conditions on whether it's possible to conclude a value for World.views  of type Oven."""
        return len([r for r in case.views if isinstance(r, Root)]) > 0
    return conditions_for_world_views_of_type_oven(case)


def conclusion_125114902106458283648997265474531641579(case) -> List[Oven]:
    def world_views_of_type_oven(case: World) -> List[Oven]:
        """Get possible value(s) for World.views  of type Oven."""
        return [Oven(b) for b in case.bodies if "oven" in b.name.name.lower()]
    return world_views_of_type_oven(case)


def conditions_39216173054479960152579027281679377283(case) -> bool:
    def conditions_for_world_views_of_type_hotplate(case: World) -> bool:
        """Get conditions on whether it's possible to conclude a value for World.views  of type Hotplate."""
        return True
    return conditions_for_world_views_of_type_hotplate(case)


def conclusion_39216173054479960152579027281679377283(case) -> List[Hotplate]:
    def world_views_of_type_hotplate(case: World) -> List[Hotplate]:
        """Get possible value(s) for World.views  of type Hotplate."""
        return [Hotplate(b) for b in case.bodies if "hotplate" in b.name.name.lower()]
    return world_views_of_type_hotplate(case)


def conditions_56932022228577464889761478858979829689(case) -> bool:
    def conditions_for_world_views_of_type_sides(case: World) -> bool:
        """Get conditions on whether it's possible to conclude a value for World.views  of type Sides."""
        return True
    return conditions_for_world_views_of_type_sides(case)


def conclusion_56932022228577464889761478858979829689(case) -> List[Sides]:
    def world_views_of_type_sides(case: World) -> List[Sides]:
        """Get possible value(s) for World.views  of type Sides."""
        all_root = [r for r in case.views if isinstance(r, Root)]
        kitchenRoot = [b for b in all_root if "kitchen" in b.body.name.name.lower()]
        root_bodies = [r.body for r in kitchenRoot]
        conections_with_root = [
            c for c in case.connections
            if isinstance(c, FixedConnection)
               and c.parent in root_bodies
        ]
        return [Sides(b.child) for b in conections_with_root if "side_" in b.child.name.name.lower()]
    return world_views_of_type_sides(case)


def conditions_304562387165012343799590402190050617477(case) -> bool:
    def conditions_for_world_views_of_type_countertop(case: World) -> bool:
        """Get conditions on whether it's possible to conclude a value for World.views  of type Countertop."""
        return len([r for r in case.views if isinstance(r, Root)]) > 0
    return conditions_for_world_views_of_type_countertop(case)


def conclusion_304562387165012343799590402190050617477(case) -> List[Countertop]:
    def world_views_of_type_countertop(case: World) -> List[Countertop]:
        """Get possible value(s) for World.views  of type Countertop."""
        all_sides = [s for s in case.views if isinstance(s, Sides)]
        side_body = [b.body for b in all_sides]
        conection_with_sides = [
            c for c in case.connections
            if isinstance(c, FixedConnection)
               and c.parent in side_body
        ]
        return [Countertop(r.child) for r in conection_with_sides if "island_countertop" in r.child.name.name.lower()]
    return world_views_of_type_countertop(case)


def conditions_49636263552498546597487129028109645395(case) -> bool:
    def conditions_for_world_views_of_type_cooktop(case: World) -> bool:
        """Get conditions on whether it's possible to conclude a value for World.views  of type Cooktop."""
        return len([r for r in case.views if isinstance(r, Root)]) > 0
    return conditions_for_world_views_of_type_cooktop(case)


def conclusion_49636263552498546597487129028109645395(case) -> List[Cooktop]:
    def world_views_of_type_cooktop(case: World) -> List[Cooktop]:
        """Get possible value(s) for World.views  of type Cooktop."""
        all_counters = [c for c in case.views if isinstance(c, Countertop)]
        counter_bodies = [c.body for c in all_counters]
    
        conections_with_counters = [
            c for c in case.connections
            if isinstance(c, FixedConnection)
               and c.parent in counter_bodies
        ]
    
        return [Cooktop(b.child) for b in conections_with_counters if "cooktop" in b.child.name.name.lower()]
    return world_views_of_type_cooktop(case)


def conditions_180013694937072261831339544475640316362(case) -> bool:
    def conditions_for_world_views_of_type_sink(case: World) -> bool:
        """Get conditions on whether it's possible to conclude a value for World.views  of type Sink."""
        return len([r for r in case.views if isinstance(r, Root)]) > 0
    return conditions_for_world_views_of_type_sink(case)


def conclusion_180013694937072261831339544475640316362(case) -> List[Sink]:
    def world_views_of_type_sink(case: World) -> List[Sink]:
        """Get possible value(s) for World.views  of type Sink."""
        all_counters = [s for s in case.views if isinstance(s, Countertop)]
        countertop_body = [b.body for b in all_counters]
        conection_with_countertops = [
            c for c in case.connections
            if isinstance(c, FixedConnection)
               and c.parent in countertop_body
        ]
        return [Sink(r.child) for r in conection_with_countertops if "sink" in r.child.name.name.lower()]
    return world_views_of_type_sink(case)


