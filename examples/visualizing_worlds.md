---
jupyter:
  jupytext:
    text_representation:
      extension: .md
      format_name: markdown
      format_version: '1.3'
      jupytext_version: 1.17.3
  kernelspec:
    display_name: Python 3
    language: python
    name: python3
---

(visualizing-worlds)=
# Visualizing Worlds

This tutorial explains you how to visualize a world.
There are two recommended ways of doing it.
One light weight way through RVIZ2 and a more heavy weight way through simulation with multiverse.
Let's load a world first to get started.

```python
import logging
import os

from semantic_world.adapters.urdf import URDFParser 
from semantic_world.utils import get_semantic_world_directory_root

logging.disable(logging.CRITICAL)
apartment = os.path.join(get_semantic_world_directory_root(os.getcwd()), "resources", "urdf", "apartment.urdf")
world = URDFParser.from_file(apartment).parse()

```

For the RVIZ2 way, ROS2 is needed. A caveat of this approach is that you have to manage the lifecycle of a ROS2 node yourself.
We recommend to put the spinning into sperate threads and just shutdown the thread when exiting the system.

```python
from semantic_world.adapters.viz_marker import VizMarkerPublisher
import threading
import rclpy
rclpy.init()

node = rclpy.create_node("semantic_world")
thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
thread.start()

viz = VizMarkerPublisher(world=world, node=node)
```

When you want to stop visualizing, you have to stop the visualizer and afterwards clean up ROS2.

```python
node.destroy_node()
rclpy.shutdown()
```

The multiverse way relies on multiverse and is WIP. Do it faster giang.
