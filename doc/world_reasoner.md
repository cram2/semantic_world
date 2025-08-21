# World Reasoner

The world reasoner {py:class}`semantic_world.reasoner.WorldReasoner` is a class that uses [Ripple Down Rules](https://github.com/AbdelrhmanBassiouny/ripple_down_rules/tree/main)
to classify concepts and attributes of the world. This is done using a rule based classifier that benefits from incremental
rule addition through querying the system and answering the prompts that pop up using python code.

The benefit of that is the rules of the reasoner are based on the world datastructures and are updates as the datastructures
are updated. Thus, the rules become a part of the semantic world repository and are update, migrated, and versioned with it.

## How to use:

There are two ways in which the reasoner can be used, classification mode, and fitting mode, both of which are explained
bellow.

### A: Classification Mode

In classification mode, the reasoner is used as is with it's latest knowledge or rule trees to classify concepts about the
world.

For example lets say the reasoner now has rules that enable it find specific types of views like the Drawer and the Cabinet.
The way to use the reasoner is like the following example:

```python
from semantic_world.reasoner import WorldReasoner
from semantic_world.adapters.urdf import URDFParser

def create_kitchen_world(kitchen_path: str = '<PATH TO THE URDF FOLDER>/kitchen-small.urdf'):
    return URDFParser(kitchen_path).parse()

kitchen_world = create_kitchen_world()
reasoner = WorldReasoner(kitchen_world)
found_concepts = reasoner.reason()

# 1st method, access the views directly from the reasoning result
new_views = found_concepts['views']
print(new_views)

# Or 2nd method, access all the views from the world.views, but this will include all views not just the new ones.
all_views = kitchen_world.views
print(all_views)
```

Similarly, for any other world attribute that the reasoner can infer values for, just replace the 'views' with the 
appropriate attribute name.

### B: Fitting Mode

In fitting mode, the reasoner can be used to improve and enlarge it's rule tree or even to widen it's application to even
more attributes of the world.

For example, let's say you want to improve an existing rule that classifies Drawers, you can do that as follows:

```python
from semantic_world.reasoner import WorldReasoner
from semantic_world.adapters.urdf import URDFParser
from semantic_world.views.views import Drawer

def create_kitchen_world(kitchen_path: str = '<PATH TO THE URDF FOLDER>/kitchen-small.urdf'):
    return URDFParser(kitchen_path).parse()

kitchen_world = create_kitchen_world()
reasoner = WorldReasoner(kitchen_world)

reasoner.fit_attribute("views", [Drawer], update_existing_views=True, world_factory=create_kitchen_world)
```

Then you will be prompted to write a rule for Drawer, and you can see the currently detected drawers shown in the Ipyton
shell. Maybe you see a mistake and not all the currently detected drawers are actual drawers, so you want to filter the
results. To start writing your rule, just type `%edit` in the Ipython terminal as shown the image bellow, or if using
the GUI just press the `Edit` button.

```{figure} images/write_edit_in_ipython.png
---
width: 800px
---
Open the Template File in Editor from the Ipython Shell.
```

Now, a template file with some imports and an empty function is openned for you to write your rule inside the body of
the function as shown bellow:

```python
from dataclasses import dataclass, field
from posixpath import dirname
from typing_extensions import Any, Callable, ClassVar, Dict, List, Optional, Type, Union
from ripple_down_rules.rdr import GeneralRDR
from ripple_down_rules.datastructures.dataclasses import CaseQuery
from semantic_world.world_entity import View
from semantic_world.reasoner import WorldReasoner
from semantic_world.world import World
from semantic_world.views.views import Drawer

def world_views_of_type_drawer(case: World) -> List[Drawer]:
    """Get possible value(s) for World.views  of type Drawer."""
    # Write code here
    pass
```

You can write a filter on the current views of type Drawer as follows:

```python
from dataclasses import dataclass, field
from posixpath import dirname
from typing_extensions import Any, Callable, ClassVar, Dict, List, Optional, Type, Union
from ripple_down_rules.rdr import GeneralRDR
from ripple_down_rules.datastructures.dataclasses import CaseQuery
from semantic_world.world_entity import View
from semantic_world.reasoner import WorldReasoner
from semantic_world.world import World
from semantic_world.views.views import Drawer

def world_views_of_type_drawer(case: World) -> List[Drawer]:
    """Get possible value(s) for World.views  of type Drawer."""
    known_drawers =  [v for v in case.views if isinstance(v, Drawer)]
    good_drawers = [d for d in known_drawers if d.name.name != "bad_drawer"]
    return good_drawers
```

So the above is the generated template, and I just filled in the body of the function with my rule logic. After that
you write `%load` in the Ipython and the function you just wrote will be available to you to test it out in the Ipython
shell as shown bellow (in the GUI just pres the Load button):

```{figure} images/load_rule_and_test_it.png
---
width: 1600px
---
Load the written Rule into the Ipython Shell.
```

Then if you want to change the rule, just edit the already open template file and do `load` again. Once you are happy
with your rule results just return the function output as follows (in the GUI just press the Accept button):

```{figure} images/accept_rule.png
---
width: 600px
---
Accept the Rule in Ipython.
```

If you also want to contribute to the semantic world package, then it's better to do that in the `test_views/test_views.py`
test file. Since there is already rules for Drawer, there would already be a test method for that. All you need to do is
set the `update_existing_views` to `True` like this:

```python
    def test_drawer_view(self):
        self.fit_rules_for_a_view_in_apartment(Drawer, scenario=self.test_drawer_view, update_existing_views=True)
```
then run the test from the terminal using `pytest` as follows:
```bash
cd semantic_world/test/test_views && pytest -s -k "test_drawer_view"
```
Then answer the prompt with the rule as described before. Now the rules for the Drawer view has been updated, Nice Work!

You could also create a new test method if your world is not the apartment or if you want to add a specific test for a
specific context, more tests are always welcome :D. Just make sure you set the scenario to be the new test method name,
and set the world factory to the method that creates your world (if it doesn't exist create one and put it in the test 
file).

In addition you can fit the reasoner on a totaly new concept/attribute of the world instead of `views`, maybe `regions`
or `predicates` , ...etc. What's great is that inside your rules you can use the views that were classified already by
the views rules, and vice verse, you can add views rules that use outputs from rules on other attributes as well.
