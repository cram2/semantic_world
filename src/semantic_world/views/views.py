from dataclasses import dataclass, field

from semantic_world.world import View, Body


@dataclass(unsafe_hash=True)
class Handle(View):
    body: Body


@dataclass(unsafe_hash=True)
class Container(View):
    body: Body


@dataclass(unsafe_hash=True)
class Drawer(View):
    container: Container
    handle: Handle


@dataclass
class Cabinet(View):
    container: Container
    drawers: list[Drawer] = field(default_factory=list)

    def __hash__(self):
        return hash((self.__class__.__name__, self.container))



######################################


@dataclass
class Cup(View):
    """
    Cup is a body that can contain liquids
    """
    body: Body

@dataclass
class Bottle(View):
    """
    Bottle is a body that can contain liquids
    """
    body: Body

@dataclass
class Food(View):
    body: Body

@dataclass
class Door(View):                   # Door has a Footprint
    """
    Door in a body that has a Handle and can open towards or away from the user.
    """
    handle: Handle
    body: Body

@dataclass
class Refrigerator(View):
    """
    Refrigerator is a Container that can contain Cups and Bottles and other stuff.
    """
    door: Door
    container: Container
    food: Food ###################### this is incide not part of???

@dataclass
class Oven(View):       ################## aber es ist kein Draw???????
    container: Container
    handle: Handle


@dataclass
class Shelf(View):
    body: Body


class Cutlery(View):
    body: Body

@dataclass
class Knife(View):
    """
    Knife is a sharp tool that can cut objects.
    """
    cutlery: Cutlery

@dataclass
class Spoon(View):
    cutlery: Cutlery

@dataclass
class Fork(View):
    cutlery: Cutlery

@dataclass
class Apple(View):
    food: Food
@dataclass
class Cereal(View):
    food: Food

################################

@dataclass(unsafe_hash=True)
class Appliances(View):
    """
    Represents a collection of home appliances.
    """
    body: Body

@dataclass(unsafe_hash=True)
class Areas(View):
    """
    Represents an area view that encapsulates details related to a specific body structure.
    """
    body: Body

@dataclass(unsafe_hash=True)
class Components(View):
    body: Body

@dataclass(unsafe_hash=True)
class Decor(View):
    body: Body

@dataclass(unsafe_hash=True)
class Furniture(View):
    body: Body

@dataclass(unsafe_hash=True)
class Tool(View):
    body: Body

######## Subclasses von Appliances

@dataclass(unsafe_hash=True)
class ElectricalAppliances(Appliances):
    ...

@dataclass(unsafe_hash=True)
class Oven(Appliances):
    ...

@dataclass(unsafe_hash=True)
class Stove(Appliances):
    ...

########## Subclasses von ElectricalAppliances

@dataclass
class AlarmClock(ElectricalAppliances):
    ...

@dataclass
class Blender(ElectricalAppliances):
    ...

@dataclass
class CoffeMaker(ElectricalAppliances):
    ...

@dataclass
class Computer(ElectricalAppliances):
    ...

@dataclass
class Dishwasher(ElectricalAppliances):
    ...

@dataclass
class Keyboard(ElectricalAppliances):           #??????????????????????????????? is it realy??
    ...

@dataclass
class Lamp(ElectricalAppliances):
    ...
@dataclass
class Laptop(ElectricalAppliances):
    ...

@dataclass
class Microwave(ElectricalAppliances):
    ...

@dataclass
class Monitor(ElectricalAppliances):
    ...

@dataclass
class Mouse(ElectricalAppliances):                # ????????? again is it really??
    ...

@dataclass
class Printer(ElectricalAppliances):
    ...

@dataclass
class Refrigerator(ElectricalAppliances):
    ...

@dataclass
class Speakers(ElectricalAppliances):
    ...

@dataclass
class Television(ElectricalAppliances):
    ...

@dataclass
class Toaster(ElectricalAppliances):
    ...

#################### subclasses von areas

@dataclass
class Bathroom(Areas):
    ...

@dataclass
class Bedroom(Areas):
    ...

@dataclass
class Hallway(Areas):
    ...

@dataclass
class Kitchen(Areas):
    ...

@dataclass
class LivingRoom(Areas):
    ...

@dataclass
class Office(Areas):
    ...

@dataclass
class OutdoorSpace(Areas):
    ...

#################### subclasses von Components

@dataclass
class Armrest(Components):
    ...

@dataclass
class Backrest(Components):
    ...

@dataclass
class Base(Components):
    ...

@dataclass
class Compartment(Components):
    ...

@dataclass
class Cooktop(Components):
    ...

@dataclass
class Cushion(Components):
    ...

@dataclass
class DesignatedHandle(Components):
    ...

@dataclass
class DesignedSpade(Components):
    ...

@dataclass
class Door(Components):
    ...

@dataclass
class Drawer(Components):
    ...

@dataclass
class Frame(Components):
    ...

@dataclass
class Hinge(Components):
    ...

@dataclass
class Hotplate(Components):
    ...

@dataclass
class Latch(Components):
    ...

@dataclass
class Leg(Components):
    ...

@dataclass
class Lid(Components):
    ...

@dataclass
class Panel(Components):
    ...

@dataclass
class Rack(Components):
    ...

@dataclass
class Sink(Components):
    ...

@dataclass
class Tap(Components):
    ...

@dataclass
class TopSurface(Components):
    ...

@dataclass
class Upholstery(Components):
    ...

################################## subclasses to Compartment
@dataclass
class FreezerCompartment(Compartment):
    ...

############################### subclasses to Cooktop
@dataclass
class ElectricCooktop(Cooktop):
    ...

@dataclass
class GasCooktop(Cooktop):
    ...

############################## subclasses to ElectricCooktop
@dataclass
class CeramicCooktop(ElectricCooktop):
    ...

@dataclass
class CoilCooktop(ElectricCooktop):
    ...

@dataclass
class InductionCooktop(ElectricCooktop):
    ...

############################### subclasses to TopSurface
@dataclass
class Countertop(TopSurface):
    ...

@dataclass
class TableTop(TopSurface):
    ...

@dataclass
class WorkingSurface(TopSurface):
    ...

############################### subclasses to Decor
@dataclass
class Blanket(Decor):
    ...

@dataclass
class Pillows(Decor):
    ...

@dataclass
class WallMounted(Decor):
    ...

############################### subclasses to Blanket
@dataclass
class ThrowBlanket(Blanket):
    ...

############################### subclasses to Pillows
@dataclass
class Cushion(Pillows):
    ...

@dataclass
class ThrowPillow(Pillows):
    ...

############################### subclasses to WallMounted
@dataclass
class Mirror(WallMounted):
    ...

@dataclass
class Pictures(WallMounted):
    ...

############################### subclasses to Furniture
@dataclass
class Cupboard(Furniture):
    ...

@dataclass
class SittingFurniture(Furniture):
    ...

@dataclass
class Storage(Furniture):
    ...

@dataclass
class Table(Furniture):
    ...

############################### subclasses to Cupboard
@dataclass
class Cabinet(Cupboard):
    ...

@dataclass
class Wardrobe(Cupboard):
    ...

############################### subclasses to Cabinet
@dataclass
class BathroomCabinet(Cabinet):
    ...

@dataclass
class ChinaCabinet(Cabinet):
    ...

@dataclass
class DisplayCabinet(Cabinet):
    ...

@dataclass
class KitchenCabinet(Cabinet):
    ...

@dataclass
class MedicineCabinet(Cabinet):
    ...

############################### subclasses to Wardrobe
@dataclass
class Dresser(Wardrobe):
    ...

############################### subclasses to SittingFurniture
@dataclass
class DesignatedChair(SittingFurniture):
    ...

@dataclass
class Sofa(SittingFurniture):
    ...

############################### subclasses to DesignatedChair
@dataclass
class Armchair(DesignatedChair):
    ...

@dataclass
class BeanBagChair(DesignatedChair):
    ...

@dataclass
class ChaiseLounge(DesignatedChair):
    ...

@dataclass
class DiningChair(DesignatedChair):
    ...

@dataclass
class OfficeChair(DesignatedChair):
    ...

@dataclass
class Ottoman(DesignatedChair):
    ...

@dataclass
class Stool(DesignatedChair):
    ...

@dataclass
class Stools(DesignatedChair):                  ############# doesn't make sense
    ...

@dataclass
class StorageBench(DesignatedChair):
    ...

############################### subclasses to Stools
@dataclass
class BarStools(Stools):
    ...

############################### subclasses to Storage
@dataclass
class Bookshelf(Storage):
    ...

@dataclass
class Chest(Storage):
    ...

@dataclass
class Cupboard(Storage):                                    ###### already exists!!!!
    ...

@dataclass
class LaundryHamper(Storage):
    ...

@dataclass
class Shelf(Storage):
    ...

@dataclass
class Sideboard(Storage):
    ...

@dataclass
class StorageBench(Storage):
    ...

@dataclass
class TowelRack(Storage):
    ...

@dataclass
class WineRack(Storage):
    ...

############################### subclasses to Shelf
@dataclass
class BookShelf(Shelf):
    ...

############################### subclasses to Table
@dataclass
class CoffeeTable(Table):
    ...

@dataclass
class ConsoleTable(Table):
    ...

@dataclass
class DinningTable(Table):
    ...

@dataclass
class SideTable(Table):
    ...

@dataclass
class TVStand(Table):
    ...

@dataclass
class VanityTable(Table):
    ...

############################### subclasses to SideTable
@dataclass
class BedsideTable(SideTable):
    ...

@dataclass
class EndTable(SideTable):
    ...

############################### subclasses to Tool
@dataclass
class CleaningSupplies(Tool):
    ...

@dataclass
class CuttingTool(Tool):
    ...

@dataclass
class DishRack(Tool):
    ...

@dataclass
class Tableware(Tool):
    ...

@dataclass
class TrashCan(Tool):
    ...

############################### subclasses to CleaningSupplies
@dataclass
class Brush(CleaningSupplies):
    ...

@dataclass
class Soap(CleaningSupplies):
    ...

@dataclass
class Sponge(CleaningSupplies):
    ...

############################### subclasses to CuttingTool
@dataclass
class Knife(CuttingTool):
    ...

############################### subclasses to Knife
@dataclass
class KitchenKnife(Knife):
    ...

############################### subclasses to KitchenKnife
@dataclass
class KitchenKnife(KitchenKnife):
    ...

@dataclass
class KitchenKnife(KitchenKnife):
    ...

############################### subclasses to Tableware
@dataclass
class Crockery(Tableware):
    ...

@dataclass
class Cutlery(Tableware):
    ...

############################### subclasses to Crockery
@dataclass
class Bowl(Crockery):
    ...

@dataclass
class Cup(Crockery):
    ...

@dataclass
class Glass(Crockery):
    ...

@dataclass
class Pan(Crockery):
    ...

@dataclass
class Plate(Crockery):
    ...

@dataclass
class Pot(Crockery):
    ...

############################### subclasses to Bowl
@dataclass
class PastaBowl(Bowl):
    ...

@dataclass
class SaladBowl(Bowl):
    ...

############################### subclasses to Glass
@dataclass
class WaterGlass(Glass):
    ...

@dataclass
class WineGlass(Glass):
    ...

############################### subclasses to Plate
@dataclass
class BreakfastPlate(Plate):
    ...

@dataclass
class DinnerPlate(Plate):
    ...

############################### subclasses to Pot
@dataclass
class SoupPot(Pot):
    ...

############################### subclasses to Cutlery
@dataclass
class Fork(Cutlery):
    ...

@dataclass
class Spatula(Cutlery):
    ...

@dataclass
class Spoon(Cutlery):
    ...

############################### subclasses to Fork
@dataclass
class DessertFork(Fork):
    ...

@dataclass
class TableFork(Fork):
    ...

############################### subclasses to Spoon
@dataclass
class TableSpoon(Spoon):
    ...

@dataclass
class TeaSpoon(Spoon):
    ...