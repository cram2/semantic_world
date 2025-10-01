from dataclasses import dataclass, field
from typing import Optional


# base stubs
class View: ...

@dataclass(eq=False)
class Entity(View):
    """
    Entity
    """

@dataclass(eq=False)
class Object(Entity):
    """
    Object
    """

@dataclass(eq=False)
class Socialobject(Object):
    """
    SocialObject
    """

@dataclass(eq=False)
class Concept(Socialobject):
    """
    Concept
    """

@dataclass(eq=False)
class Eventtype(Concept):
    """
    EventType
    """

@dataclass(eq=False)
class Task(Eventtype):
    """
    task
    """

@dataclass(eq=False)
class Parameter(Concept):
    """
    Parameter
    """

@dataclass(eq=False)
class MixingMotion(Parameter):
    """
    mixing motion
    """

@dataclass(eq=False)
class Physicalobject(Object):
    """
    PhysicalObject
    """

@dataclass(eq=False)
class Tool(Physicalobject):
    """
    Tool
    """

@dataclass(eq=False)
class Cutlery(Tool):
    """
    Cutlery
    """

@dataclass(eq=False)
class Peelingtool(Cutlery):
    """
    PeelingTool
    """

@dataclass(eq=False)
class Peeler(Peelingtool):
    """
    Peeler
    """

@dataclass(eq=False)
class Cuttingtool(Cutlery):
    """
    CuttingTool
    """

@dataclass(eq=False)
class Knife(Cuttingtool):
    """
    Knife
    """

@dataclass(eq=False)
class Food(Physicalobject):
    """
    Food
    """

@dataclass(eq=False)
class Foodpart(Food):
    """
    FoodPart
    """

@dataclass(eq=False)
class Physicalquality(Entity):
    """
    PhysicalQuality
    """

@dataclass(eq=False)
class Disposition(Physicalquality):
    """
    Disposition
    """
@dataclass(eq=False)
class Cuttingtask(Task):
    """
    CuttingTask
    """

@dataclass(eq=False)
class Preparationtask(Task):
    """
    PreparationTask
    """

@dataclass(eq=False)
class Foodpreparationtask(Preparationtask):
    """
    FoodPreparationTask
    """

@dataclass(eq=False)
class Peeling(Foodpreparationtask):
    """
    Peeling
    """

@dataclass(eq=False)
class Coreremovaltool(Cutlery):
    """
    CoreRemovalTool
    """

@dataclass(eq=False)
class Peelability(Disposition):
    """
    Peelability
    """
    peelable: bool = True
    affords_task: Optional[Peeling] = None
    affords_trigger: Optional[Cutlery] = None


@dataclass(eq=False)
class Cuttability(Disposition):
    """
    Cuttability
    """
    cuttable: bool = True
    affords_task: Optional[Cuttingtask] = None
    affords_trigger: Optional[Cuttingtool] = None


@dataclass(eq=False)
class Coreremovability(Disposition):
    """
    CoreRemovability
    """
    coreremovable: bool = True
    affords_task: Optional[Foodpreparationtask] = None
    affords_trigger: Optional[Cutlery] = None

@dataclass(eq=False)
class Edibility(Disposition):
    """
    Edibility
    """

@dataclass(eq=False)
class Shape(Physicalquality):
    """
    Shape
    """

@dataclass(eq=False)
class Liquids(Food):
    """
    liquids
    """

@dataclass(eq=False)
class Alcohol(Liquids):
    """
    alcohol
    """

@dataclass(eq=False)
class Feature(Object):
    """
    Feature
    """

@dataclass(eq=False)
class Consistency(Feature):
    """
    Consistency
    """

@dataclass(eq=False)
class Liquid(Consistency):
    """
    Liquid
    """

@dataclass(eq=False)
class DrinkingWater(Liquids):
    """
    drinking water
    """

@dataclass(eq=False)
class PlantFruitFoodProduct(Food):
    """
    plant fruit food product
    """

@dataclass(eq=False)
class Honey(Liquids):
    """
    honey
    """

@dataclass(eq=False)
class Viscuous(Consistency):
    """
    Viscuous
    """

@dataclass(eq=False)
class BeerBeverage(Liquids):
    """
    beer beverage
    """

@dataclass(eq=False)
class BlackOrWhitePepper(Food):
    """
    black or white pepper
    """

@dataclass(eq=False)
class Powdery(Consistency):
    """
    Powdery
    """

@dataclass(eq=False)
class BakingSoda(Food):
    """
    baking soda
    """

@dataclass(eq=False)
class Mixtures(Food):
    """
    mixtures
    """

@dataclass(eq=False)
class SoupLiquid(Liquids, Mixtures):
    """
    soup (liquid)
    """

@dataclass(eq=False)
class BerryFruit(PlantFruitFoodProduct):
    """
    berry fruit
    """

@dataclass(eq=False)
class PepoFruit(BerryFruit):
    """
    pepo fruit
    """


@dataclass(eq=False)
class Edible(Edibility):
    """
    Edible
    """

@dataclass(eq=False)
class Partoutside(Foodpart):
    """
    PartOutside
    """

@dataclass(eq=False)
class Skinorpeel(Partoutside):
    """
    SkinOrPeel
    """

@dataclass(eq=False)
class Shell(Partoutside):
    """
    Shell
    """
    eatable: bool = field(default=True)


@dataclass(eq=False)
class Peel(Skinorpeel):
    """
    Peel
    """
    eatable: bool = field(default=True)

@dataclass(eq=False)
class Inedible(Edibility):
    """
    Inedible
    """

@dataclass(eq=False)
class Nutcracker(Peelingtool):
    """
    Nutcracker
    """


@dataclass(eq=False)
class Shouldbeavoided(Inedible):
    """
    ShouldBeAvoided
    """

@dataclass(eq=False)
class Stem(Partoutside):
    """
    Stem
    """
    eatable: bool = field(default=True)

@dataclass(eq=False)
class Strawberry(PlantFruitFoodProduct):
    """
    strawberry
    """
    cuttable: Cuttability = field(default_factory=lambda: Cuttability(
    cuttable=True,
    affords_task=Cuttingtask(),
    affords_trigger=Knife()
))
    stem: Stem = field(default_factory=lambda : Stem(eatable=False))


@dataclass(eq=False)
class CucumberWhole(PepoFruit):
    """
    cucumber (whole)
    """
    peelable: Peelability = field(default_factory=lambda: Peelability(
    peelable=True,
    affords_task=Peeling(),
    affords_trigger=Peeler()
))
    cuttable: Cuttability = field(default_factory=lambda: Cuttability(
    cuttable=True,
    affords_task=Cuttingtask(),
    affords_trigger=Knife()
))
    peel: Peel = field(default_factory=lambda : Peel(eatable=True))
    stem: Stem = field(default_factory=lambda : Stem(eatable=False))

@dataclass(eq=False)
class NutFruit(PlantFruitFoodProduct):
    """
    nut fruit
    """
    peelable: Peelability = field(default_factory=lambda: Peelability(
    peelable=True,
    affords_task=Peeling(),
    affords_trigger=Nutcracker()
))
    cuttable: Cuttability = field(default_factory=lambda: Cuttability(
    cuttable=True,
    affords_task=Cuttingtask(),
    affords_trigger=Knife()
))
    shell: Shell = field(default_factory=lambda : Shell(eatable=False))

@dataclass(eq=False)
class Coconut(NutFruit):
    """
    coconut
    """


@dataclass(eq=False)
class Partinside(Foodpart):
    """
    PartInside
    """


@dataclass(eq=False)
class Core(Partinside):
    """
    Core
    """
    eatable: bool = field(default=True)


@dataclass(eq=False)
class Pineapple(BerryFruit):
    """
    pineapple
    """
    core_removable: Coreremovability = field(default_factory=lambda: Coreremovability(
    coreremovable=True,
    affords_task=Corecutting(),
    affords_trigger=Knife()
))
    peelable: Peelability = field(default_factory=lambda: Peelability(
    peelable=True,
    affords_task=Peeling(),
    affords_trigger=Knife()
))
    cuttable: Cuttability = field(default_factory=lambda: Cuttability(
    cuttable=True,
    affords_task=Cuttingtask(),
    affords_trigger=Knife()
))
    core: Core = field(default_factory=lambda : Core(eatable=True))
    peel: Peel = field(default_factory=lambda : Peel(eatable=False))

@dataclass(eq=False)
class Coreremoving(Foodpreparationtask):
    """
    CoreRemoving
    """

@dataclass(eq=False)
class Corecutting(Coreremoving):
    """
    CoreCutting
    """



@dataclass(eq=False)
class Mustbeavoided(Inedible):
    """
    MustBeAvoided
    """

@dataclass(eq=False)
class StoneFruit(PlantFruitFoodProduct):
    """
    stone_fruit
    """
    core_removable: Coreremovability = field(default_factory=lambda: Coreremovability(
    coreremovable=True,
    affords_task=Corecutting(),
    affords_trigger=Knife()
))
    cuttable: Cuttability = field(default_factory=lambda: Cuttability(
    cuttable=True,
    affords_task=Cuttingtask(),
    affords_trigger=Knife()
))
    core: Core = field(default_factory=lambda : Core(eatable=False))

@dataclass(eq=False)
class Cherry(StoneFruit):
    """
    cherry
    """

@dataclass(eq=False)
class Pumpkin(PepoFruit):
    """
    pumpkin
    """
    core_removable: Coreremovability = field(default_factory=lambda: Coreremovability(
    coreremovable=True,
    affords_task=Corescraping(),
    affords_trigger=Spoon()
))
    peelable: Peelability = field(default_factory=lambda: Peelability(
    peelable=True,
    affords_task=Peeling(),
    affords_trigger=Knife()
))
    cuttable: Cuttability = field(default_factory=lambda: Cuttability(
    cuttable=True,
    affords_task=Cuttingtask(),
    affords_trigger=Knife()
))
    peel: Peel = field(default_factory=lambda : Peel(eatable=False))

@dataclass(eq=False)
class Scraping(Foodpreparationtask):
    """
    Scraping
    """

@dataclass(eq=False)
class Corescraping(Coreremoving, Scraping):
    """
    CoreScraping
    """


@dataclass(eq=False)
class Pepper(BerryFruit):
    """
    pepper
    """
    cuttable: Cuttability = field(default_factory=lambda: Cuttability(
    cuttable=True,
    affords_task=Cuttingtask(),
    affords_trigger=Knife()
))
    stem: Stem = field(default_factory=lambda : Stem(eatable=False))

@dataclass(eq=False)
class Almond(NutFruit):
    """
    almond
    """

@dataclass(eq=False)
class Squash(PepoFruit):
    """
    squash
    """
    core_removable: Coreremovability = field(default_factory=lambda: Coreremovability(
    coreremovable=True,
    affords_task=Corescraping(),
    affords_trigger=Spoon()
))
    peelable: Peelability = field(default_factory=lambda: Peelability(
    peelable=True,
    affords_task=Peeling(),
    affords_trigger=Knife()
))
    cuttable: Cuttability = field(default_factory=lambda: Cuttability(
    cuttable=True,
    affords_task=Cuttingtask(),
    affords_trigger=Knife()
))
    peel: Peel = field(default_factory=lambda : Peel(eatable=False))

@dataclass(eq=False)
class Avocado(BerryFruit):
    """
    avocado
    """
    core_removable: Coreremovability = field(default_factory=lambda: Coreremovability(
    coreremovable=True,
    affords_task=Corecutting(),
    affords_trigger=Spoon()
))
    peelable: Peelability = field(default_factory=lambda: Peelability(
    peelable=True,
    affords_task=Peeling(),
    affords_trigger=Knife()
))
    cuttable: Cuttability = field(default_factory=lambda: Cuttability(
    cuttable=True,
    affords_task=Cuttingtask(),
    affords_trigger=Knife()
))
    core: Core = field(default_factory=lambda : Core(eatable=False))
    peel: Peel = field(default_factory=lambda : Peel(eatable=False))


@dataclass(eq=False)
class CitrusFruitWholeRaw(PlantFruitFoodProduct):
    """
    citrus fruit (whole, raw)
    """
    peelable: Peelability = field(default_factory=lambda: Peelability(
    peelable=True,
    affords_task=Peeling(),
    affords_trigger=Hand()
))
    cuttable: Cuttability = field(default_factory=lambda: Cuttability(
    cuttable=True,
    affords_task=Cuttingtask(),
    affords_trigger=Knife()
))
    peel: Peel = field(default_factory=lambda : Peel(eatable=False))

@dataclass(eq=False)
class Lime(CitrusFruitWholeRaw):
    """
    lime
    """

@dataclass(eq=False)
class FoodMixture(Mixtures):
    """
    food mixture
    """

@dataclass(eq=False)
class Moist(Consistency):
    """
    Moist
    """

@dataclass(eq=False)
class Banana(BerryFruit):
    """
    banana
    """
    peelable: Peelability = field(default_factory=lambda: Peelability(
    peelable=True,
    affords_task=Peeling(),
    affords_trigger=Hand()
))
    cuttable: Cuttability = field(default_factory=lambda: Cuttability(
    cuttable=True,
    affords_task=Cuttingtask(),
    affords_trigger=Knife()
))
    peel: Peel = field(default_factory=lambda : Peel(eatable=False))

@dataclass(eq=False)
class Hand(Peelingtool):
    """
    Hand
    """

@dataclass(eq=False)
class Rice(Food):
    """
    rice
    """

@dataclass(eq=False)
class Dry(Consistency):
    """
    Dry
    """

@dataclass(eq=False)
class KiwiFruit(BerryFruit):
    """
    kiwi fruit
    """
    peelable: Peelability = field(default_factory=lambda: Peelability(
    peelable=True,
    affords_task=Peeling(),
    affords_trigger=Knife()
))
    cuttable: Cuttability = field(default_factory=lambda: Cuttability(
    cuttable=True,
    affords_task=Cuttingtask(),
    affords_trigger=Knife()
))
    peel: Peel = field(default_factory=lambda : Peel(eatable=True))

@dataclass(eq=False)
class CoffeeLiquidDrink(Liquids):
    """
    coffee (liquid drink)
    """

@dataclass(eq=False)
class SugarGranulated(Food):
    """
    sugar (granulated)
    """

@dataclass(eq=False)
class Batter(Liquids, Mixtures):
    """
    batter
    """

@dataclass(eq=False)
class WineBeverage(Liquids):
    """
    wine beverage
    """

@dataclass(eq=False)
class Bean(PlantFruitFoodProduct):
    """
    bean
    """
    cuttable: Cuttability = field(default_factory=lambda: Cuttability(
    cuttable=True,
    affords_task=Cuttingtask(),
    affords_trigger=Knife()
))

@dataclass(eq=False)
class Lemon(CitrusFruitWholeRaw):
    """
    lemon
    """

@dataclass(eq=False)
class Champagne(Liquids):
    """
    champagne
    """

@dataclass(eq=False)
class Vinegar(Liquids):
    """
    vinegar
    """

@dataclass(eq=False)
class PomeFruit(BerryFruit):
    """
    pome fruit
    """

@dataclass(eq=False)
class Apple(PomeFruit):
    """
    apple
    """
    core_removable: Coreremovability = field(default_factory=lambda: Coreremovability(
    coreremovable=True,
    affords_task=Corecutting(),
    affords_trigger=Cuttingtool()
))
    core_removable: Coreremovability = field(default_factory=lambda: Coreremovability(
        coreremovable=True,
        affords_task=Corescraping(),
        affords_trigger=Spoon()
    ))

    peelable: Peelability = field(default_factory=lambda: Peelability(
    peelable=True,
    affords_task=Peeling(),
    affords_trigger=Peeler()
))
    cuttable: Cuttability = field(default_factory=lambda: Cuttability(
    cuttable=True,
    affords_task=Cuttingtask(),
    affords_trigger=Cuttingtool()
))
    core: Core = field(default_factory=lambda : Core(eatable=False))
    peel: Peel = field(default_factory=lambda : Peel(eatable=True))

@dataclass(eq=False)
class Butter(Food):
    """
    butter
    """

@dataclass(eq=False)
class Powder(Food):
    """
    powder
    """

@dataclass(eq=False)
class BakingPowder(Powder, Food):
    """
    baking powder
    """

@dataclass(eq=False)
class WhiteWheatFlour(Food):
    """
    white wheat flour
    """

@dataclass(eq=False)
class Syrup(Liquids):
    """
    syrup
    """

@dataclass(eq=False)
class Pasta(Food):
    """
    pasta
    """

@dataclass(eq=False)
class CitronWholeRaw(CitrusFruitWholeRaw):
    """
    citron (whole, raw)
    """

@dataclass(eq=False)
class KumquatWholeRaw(CitrusFruitWholeRaw):
    """
    kumquat (whole, raw)
    """

@dataclass(eq=False)
class Chocolate(Food):
    """
    chocolate
    """

@dataclass(eq=False)
class Orange(CitrusFruitWholeRaw):
    """
    orange
    """
    peelable: Peelability = field(default_factory=lambda: Peelability(
    peelable=True,
    affords_task=Peeling(),
    affords_trigger=Hand()
))
    cuttable: Cuttability = field(default_factory=lambda: Cuttability(
    cuttable=True,
    affords_task=Cuttingtask(),
    affords_trigger=Knife()
))
    peel: Peel = field(default_factory=lambda : Peel(eatable=False))

@dataclass(eq=False)
class Tomato(BerryFruit):
    """
    tomato
    """
    peelable: Peelability = field(default_factory=lambda: Peelability(
    peelable=True,
    affords_task=Peeling(),
    affords_trigger=None
))
    cuttable: Cuttability = field(default_factory=lambda: Cuttability(
    cuttable=True,
    affords_task=Cuttingtask(),
    affords_trigger=Knife()
))
    peel: Peel = field(default_factory=lambda : Peel(eatable=True))
    stem: Stem = field(default_factory=lambda : Stem(eatable=False))

@dataclass(eq=False)
class TableSalt(Food):
    """
    table salt
    """

@dataclass(eq=False)
class BakersYeast(Food):
    """
    bakers yeast
    """

@dataclass(eq=False)
class Oil(Liquids):
    """
    oil
    """

@dataclass(eq=False)
class Sauce(Liquids, Mixtures):
    """
    sauce
    """

@dataclass(eq=False)
class TeaFoodProduct(Liquids, Mixtures):
    """
    tea food product
    """

@dataclass(eq=False)
class Dressing(Liquids, Mixtures):
    """
    dressing
    """

@dataclass(eq=False)
class Peach(StoneFruit):
    """
    peach
    """

@dataclass(eq=False)
class JuiceBeverage(Liquids):
    """
    juice beverage
    """

@dataclass(eq=False)
class ChickenEgg(Food):
    """
    chicken egg
    """

@dataclass(eq=False)
class Olive(StoneFruit):
    """
    olive
    """

@dataclass(eq=False)
class Broth(Liquids, Mixtures):
    """
    broth
    """

@dataclass(eq=False)
class Milk(Liquids):
    """
    milk
    """

@dataclass(eq=False)
class Quantity(Socialobject, Entity):
    """
    Quantity
    """

@dataclass(eq=False)
class Resourceunit(Quantity):
    """
    ResourceUnit
    """

@dataclass(eq=False)
class Countingunit(Resourceunit):
    """
    CountingUnit
    """

@dataclass(eq=False)
class Degree(Quantity):
    """
    Degree
    """

@dataclass(eq=False)
class Second(Quantity):
    """
    Second
    """

@dataclass(eq=False)
class Kitchenknife(Coreremovaltool, Knife):
    """
    KitchenKnife
    """

@dataclass(eq=False)
class Crockery(Tool):
    """
    Crockery
    """

@dataclass(eq=False)
class Bowl(Crockery):
    """
    bowl
    """

@dataclass(eq=False)
class Bread(Food):
    """
    Bread
    """
    cuttable: Cuttability = field(default_factory=lambda: Cuttability(
    cuttable=True,
    affords_task=Cuttingtask(),
    affords_trigger=Knife()
))

@dataclass(eq=False)
class Cup(Crockery):
    """
    Cup
    """

@dataclass(eq=False)
class Cutting(Cuttingtask):
    """
    Cutting
    """

@dataclass(eq=False)
class Slice(Shape):
    """
    Slice
    """

@dataclass(eq=False)
class Cuttingposition(Parameter):
    """
    CuttingPosition
    """

@dataclass(eq=False)
class Slicingposition(Cuttingposition):
    """
    SlicingPosition
    """

@dataclass(eq=False)
class Dicing(Cuttingtask):
    """
    Dicing
    """

@dataclass(eq=False)
class Stripe(Shape):
    """
    Stripe
    """

@dataclass(eq=False)
class Cube(Shape):
    """
    Cube
    """

@dataclass(eq=False)
class Julienning(Cuttingtask):
    """
    Julienning
    """

@dataclass(eq=False)
class Pan(Crockery):
    """
    Pan
    """

@dataclass(eq=False)
class Pot(Crockery):
    """
    Pot
    """

@dataclass(eq=False)
class Slicing(Cuttingtask):
    """
    Slicing
    """

@dataclass(eq=False)
class Mixingtool(Tool):
    """
    MixingTool
    """

@dataclass(eq=False)
class Spoon(Cutlery, Mixingtool, Coreremovaltool):
    """
    Spoon
    """

@dataclass(eq=False)
class Mixingtask(Task):
    """
    MixingTask
    """

@dataclass(eq=False)
class Adding(Mixingtask):
    """
    Adding
    """

@dataclass(eq=False)
class SpiralMotion(MixingMotion):
    """
    spiral motion
    """

@dataclass(eq=False)
class Admixing(Adding):
    """
    Admixing
    """

@dataclass(eq=False)
class Mixing(Mixingtask):
    """
    Mixing
    """

@dataclass(eq=False)
class Aggregating(Mixing):
    """
    Aggregating
    """

@dataclass(eq=False)
class Amalgamating(Mixing):
    """
    Amalgamating
    """

@dataclass(eq=False)
class Applecutter(Cuttingtool):
    """
    AppleCutter
    """

@dataclass(eq=False)
class Pickplacetask(Task):
    """
    PickPlaceTask
    """

@dataclass(eq=False)
class Arranging(Pickplacetask):
    """
    Arranging
    """

@dataclass(eq=False)
class Cookcooltask(Task):
    """
    CookCoolTask
    """

@dataclass(eq=False)
class Baking(Cookcooltask):
    """
    Baking
    """

@dataclass(eq=False)
class Balancing(Pickplacetask):
    """
    Balancing
    """

@dataclass(eq=False)
class Beating(Mixingtask):
    """
    Beating
    """

@dataclass(eq=False)
class Whisk(Mixingtool, Tool):
    """
    whisk
    """

@dataclass(eq=False)
class Leaveinhotwater(Cookcooltask):
    """
    LeaveInHotWater
    """

@dataclass(eq=False)
class Blanching(Leaveinhotwater):
    """
    Blanching
    """

@dataclass(eq=False)
class Blending(Mixing):
    """
    Blending
    """

@dataclass(eq=False)
class Cooking(Cookcooltask):
    """
    Cooking
    """

@dataclass(eq=False)
class Boiling(Cooking):
    """
    Boiling
    """

@dataclass(eq=False)
class Brewing(Leaveinhotwater):
    """
    Brewing
    """

@dataclass(eq=False)
class Frying(Cookcooltask):
    """
    Frying
    """

@dataclass(eq=False)
class Browning(Baking, Frying):
    """
    Browning
    """

@dataclass(eq=False)
class Caramelizing(Frying):
    """
    Caramelizing
    """

@dataclass(eq=False)
class Carving(Cutting):
    """
    Carving
    """

@dataclass(eq=False)
class Pouringtask(Task):
    """
    PouringTask
    """

@dataclass(eq=False)
class Pouring(Pouringtask):
    """
    Pouring
    """

@dataclass(eq=False)
class Cascading(Pouring):
    """
    Cascading
    """

@dataclass(eq=False)
class Changing(Arranging):
    """
    Changing
    """

@dataclass(eq=False)
class Cooling(Cookcooltask):
    """
    Cooling
    """

@dataclass(eq=False)
class Chilling(Cooling):
    """
    Chilling
    """

@dataclass(eq=False)
class Chopping(Dicing):
    """
    Chopping
    """

@dataclass(eq=False)
class Shutting(Pickplacetask):
    """
    Shutting
    """

@dataclass(eq=False)
class Closing(Shutting):
    """
    Closing
    """

@dataclass(eq=False)
class Coalescing(Mixing):
    """
    Coalescing
    """

@dataclass(eq=False)
class Picking(Pickplacetask):
    """
    Picking
    """

@dataclass(eq=False)
class Collecting(Picking):
    """
    Collecting
    """

@dataclass(eq=False)
class Combining(Mixing):
    """
    Combining
    """

@dataclass(eq=False)
class Commingling(Mixing):
    """
    Commingling
    """

@dataclass(eq=False)
class Commixing(Mixing):
    """
    Commixing
    """

@dataclass(eq=False)
class Compounding(Mixing):
    """
    Compounding
    """

@dataclass(eq=False)
class Concocting(Mixing):
    """
    Concocting
    """

@dataclass(eq=False)
class Conflating(Mixing):
    """
    Conflating
    """

@dataclass(eq=False)
class Container(Physicalobject):
    """
    Container
    """

@dataclass(eq=False)
class Event(Entity):
    """
    Event
    """

@dataclass(eq=False)
class Action(Event):
    """
    Action
    """

@dataclass(eq=False)
class Cookcoolaction(Action):
    """
    CookCoolAction
    """

@dataclass(eq=False)
class Sprinkling(Pouringtask):
    """
    Sprinkling
    """

@dataclass(eq=False)
class Crumbling(Sprinkling):
    """
    Crumbling
    """

@dataclass(eq=False)
class Cubing(Dicing):
    """
    Cubing
    """

@dataclass(eq=False)
class Cuttingaction(Action):
    """
    CuttingAction
    """

@dataclass(eq=False)
class Heating(Cookcooltask):
    """
    Heating
    """

@dataclass(eq=False)
class Defrosting(Heating):
    """
    Defrosting
    """

@dataclass(eq=False)
class Disposing(Arranging):
    """
    Disposing
    """

@dataclass(eq=False)
class Halving(Cuttingtask):
    """
    Halving
    """

@dataclass(eq=False)
class Dividing(Halving):
    """
    Dividing
    """

@dataclass(eq=False)
class Pouringthrough(Pouringtask):
    """
    PouringThrough
    """

@dataclass(eq=False)
class Draining(Pouringthrough):
    """
    Draining
    """

@dataclass(eq=False)
class Sieve(Tool):
    """
    Sieve
    """

@dataclass(eq=False)
class Eighth(Countingunit):
    """
    Eighth
    """

@dataclass(eq=False)
class Electrictool(Tool):
    """
    ElectricTool
    """

@dataclass(eq=False)
class Mixer(Electrictool, Mixingtool):
    """
    Mixer
    """

@dataclass(eq=False)
class Electricmixer(Mixer):
    """
    ElectricMixer
    """

@dataclass(eq=False)
class Filleting(Foodpreparationtask):
    """
    Filleting
    """

@dataclass(eq=False)
class Filling(Preparationtask):
    """
    Filling
    """

@dataclass(eq=False)
class Shapechanging(Preparationtask):
    """
    ShapeChanging
    """

@dataclass(eq=False)
class Flattening(Shapechanging):
    """
    Flattening
    """

@dataclass(eq=False)
class Fleshorpulp(Partinside):
    """
    FleshOrPulp
    """

@dataclass(eq=False)
class Flowing(Pouring):
    """
    Flowing
    """

@dataclass(eq=False)
class Folding(Mixingtask):
    """
    Folding
    """

@dataclass(eq=False)
class OrbitalMotion(MixingMotion):
    """
    orbital motion
    """

@dataclass(eq=False)
class Fork(Cutlery):
    """
    Fork
    """

@dataclass(eq=False)
class Freezing(Cooling):
    """
    Freezing
    """

@dataclass(eq=False)
class Fusing(Mixing):
    """
    Fusing
    """

@dataclass(eq=False)
class Gathering(Picking):
    """
    Gathering
    """

@dataclass(eq=False)
class Grilling(Cookcooltask):
    """
    Grilling
    """

@dataclass(eq=False)
class Grouping(Mixingtask):
    """
    Grouping
    """

@dataclass(eq=False)
class Halve(Countingunit):
    """
    Halve
    """

@dataclass(eq=False)
class Halvingposition(Cuttingposition):
    """
    HalvingPosition
    """

@dataclass(eq=False)
class Handmixer(Mixer):
    """
    Handmixer
    """

@dataclass(eq=False)
class Placing(Pickplacetask):
    """
    Placing
    """

@dataclass(eq=False)
class Inserting(Placing):
    """
    Inserting
    """

@dataclass(eq=False)
class Integrating(Adding):
    """
    Integrating
    """

@dataclass(eq=False)
class Intermixing(Mixing):
    """
    Intermixing
    """

@dataclass(eq=False)
class Kneading(Preparationtask):
    """
    Kneading
    """

@dataclass(eq=False)
class Laying(Placing):
    """
    Laying
    """

@dataclass(eq=False)
class Melding(Mixing):
    """
    Melding
    """

@dataclass(eq=False)
class Melting(Heating):
    """
    Melting
    """

@dataclass(eq=False)
class Merging(Mixing):
    """
    Merging
    """

@dataclass(eq=False)
class Microwaving(Heating):
    """
    Microwaving
    """

@dataclass(eq=False)
class Mincing(Dicing):
    """
    Mincing
    """

@dataclass(eq=False)
class Mingling(Mixing):
    """
    Mingling
    """

@dataclass(eq=False)
class Mixingaction(Action):
    """
    MixingAction
    """

@dataclass(eq=False)
class Mixingduration(Parameter):
    """
    MixingDuration
    """

@dataclass(eq=False)
class Mug(Crockery):
    """
    Mug
    """

@dataclass(eq=False)
class Opening(Pickplacetask):
    """
    Opening
    """

@dataclass(eq=False)
class Oval(Shape):
    """
    Oval
    """

@dataclass(eq=False)
class Overcooking(Cooking):
    """
    Overcooking
    """

@dataclass(eq=False)
class Paring(Cutting):
    """
    Paring
    """

@dataclass(eq=False)
class Paringknife(Knife):
    """
    ParingKnife
    """

@dataclass(eq=False)
class Pastabowl(Bowl):
    """
    PastaBowl
    """

@dataclass(eq=False)
class Pickplaceaction(Action):
    """
    PickPlaceAction
    """

@dataclass(eq=False)
class Piece(Countingunit):
    """
    Piece
    """

@dataclass(eq=False)
class Piling(Arranging):
    """
    Piling
    """

@dataclass(eq=False)
class Pit(Partinside):
    """
    Pit
    """

@dataclass(eq=False)
class Positioning(Arranging):
    """
    Positioning
    """

@dataclass(eq=False)
class Pouringangle(Parameter):
    """
    PouringAngle
    """

@dataclass(eq=False)
class Pouringduration(Parameter):
    """
    PouringDuration
    """

@dataclass(eq=False)
class Pouringaction(Action):
    """
    PouringAction
    """

@dataclass(eq=False)
class Preparationaction(Action):
    """
    PreparationAction
    """

@dataclass(eq=False)
class Putting(Placing):
    """
    Putting
    """

@dataclass(eq=False)
class Quarter(Countingunit):
    """
    Quarter
    """

@dataclass(eq=False)
class Quartercoreremoving(Corecutting):
    """
    QuarterCoreRemoving
    """

@dataclass(eq=False)
class Quartering(Cuttingtask):
    """
    Quartering
    """

@dataclass(eq=False)
class Reaching(Picking):
    """
    Reaching
    """

@dataclass(eq=False)
class Refrigerating(Cooling):
    """
    Refrigerating
    """

@dataclass(eq=False)
class Reheating(Heating):
    """
    Reheating
    """

@dataclass(eq=False)
class Removing(Preparationtask):
    """
    Removing
    """

@dataclass(eq=False)
class Roasting(Frying):
    """
    Roasting
    """

@dataclass(eq=False)
class Rolling(Shapechanging):
    """
    Rolling
    """

@dataclass(eq=False)
class Round(Shape):
    """
    Round
    """

@dataclass(eq=False)
class Saladbowl(Bowl):
    """
    SaladBowl
    """

@dataclass(eq=False)
class Sawing(Cutting):
    """
    Sawing
    """

@dataclass(eq=False)
class Seeds(Partinside):
    """
    Seeds
    """

@dataclass(eq=False)
class Semicircle(Shape):
    """
    Semicircle
    """

@dataclass(eq=False)
class Semioval(Shape):
    """
    Semioval
    """

@dataclass(eq=False)
class Setting(Placing):
    """
    Setting
    """

@dataclass(eq=False)
class Shaker(Container):
    """
    Shaker
    """

@dataclass(eq=False)
class Simmering(Cooking):
    """
    Simmering
    """

@dataclass(eq=False)
class Sixteenth(Countingunit):
    """
    Sixteenth
    """

@dataclass(eq=False)
class Slivering(Slicing):
    """
    Slivering
    """

@dataclass(eq=False)
class Snipping(Slicing):
    """
    Snipping
    """

@dataclass(eq=False)
class Soaking(Preparationtask):
    """
    Soaking
    """

@dataclass(eq=False)
class Spilling(Pouringtask):
    """
    Spilling
    """

@dataclass(eq=False)
class Splashing(Pouring):
    """
    Splashing
    """

@dataclass(eq=False)
class Steaming(Cooking):
    """
    Steaming
    """

@dataclass(eq=False)
class Stemremoving(Foodpreparationtask):
    """
    StemRemoving
    """

@dataclass(eq=False)
class Stemremovability(Disposition):
    """
    StemRemovability
    """

@dataclass(eq=False)
class Sticking(Arranging):
    """
    Sticking
    """

@dataclass(eq=False)
class Stirring(Mixing):
    """
    Stirring
    """

@dataclass(eq=False)
class Streaming(Pouring):
    """
    Streaming
    """

@dataclass(eq=False)
class Taking(Picking):
    """
    Taking
    """

@dataclass(eq=False)
class Thawing(Heating):
    """
    Thawing
    """

@dataclass(eq=False)
class Throwing(Pickplacetask):
    """
    Throwing
    """

@dataclass(eq=False)
class Tilting(Arranging):
    """
    Tilting
    """

@dataclass(eq=False)
class Toasting(Cookcooltask):
    """
    Toasting
    """

@dataclass(eq=False)
class Tossing(Pickplacetask):
    """
    Tossing
    """

@dataclass(eq=False)
class Transferring(Arranging):
    """
    Transferring
    """

@dataclass(eq=False)
class Unifying(Mixing):
    """
    Unifying
    """

@dataclass(eq=False)
class Using(Pickplacetask):
    """
    Using
    """

@dataclass(eq=False)
class Warming(Heating):
    """
    Warming
    """

@dataclass(eq=False)
class WhirlstormMotion(MixingMotion):
    """
    whirlstorm motion
    """

@dataclass(eq=False)
class Whisking(Mixingtask):
    """
    Whisking
    """

@dataclass(eq=False)
class Woodenspoon(Spoon):
    """
    WoodenSpoon
    """

@dataclass(eq=False)
class CircularMotion(MixingMotion):
    """
    circular motion
    """

@dataclass(eq=False)
class Cream(Liquids):
    """
    cream
    """

@dataclass(eq=False)
class Drink(Liquids):
    """
    drink
    """

@dataclass(eq=False)
class EllipticalMotion(MixingMotion):
    """
    elliptical motion
    """

@dataclass(eq=False)
class FoldingMotion(MixingMotion):
    """
    folding motion
    """

@dataclass(eq=False)
class Grease(Liquids):
    """
    grease
    """

@dataclass(eq=False)
class Marinade(Mixtures):
    """
    marinade
    """

@dataclass(eq=False)
class Meat(Food):
    """
    meat
    """

@dataclass(eq=False)
class Puree(Liquids):
    """
    puree
    """

@dataclass(eq=False)
class Smoothie(Liquids, Mixtures):
    """
    smoothie
    """

@dataclass(eq=False)
class _10seconds(Pouringduration):
    """
    10Seconds
    """

@dataclass(eq=False)
class _2seconds(Pouringduration):
    """
    2Seconds
    """

@dataclass(eq=False)
class _45degree(Pouringangle):
    """
    45Degree
    """

@dataclass(eq=False)
class _90degree(Pouringangle):
    """
    90Degree
    """

@dataclass(eq=False)
class Breadknife(Knife):
    """
    BreadKnife
    """

