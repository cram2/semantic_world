import inspect
import os
from subprocess import call

import semantic_world.geometry
from semantic_world.world import *


def generate_world_class_diagram(sources: str, output_dir: str):

    command = ["pyreverse",
               "--source-roots", f"{sources}/semantic_world",
               "--output-directory", output_dir,
               "--project", "semantic_world",
               "--output", "puml",
               "--only-classnames",
               "semantic_world.world",]
    raw_command = " ".join(command)
    print(raw_command)
    call(command)


def main():
    """
    Main function to generate the class diagrams
    """
    sources = os.path.abspath(os.path.join(inspect.getfile(semantic_world), "..", ".."))
    output_dir = os.path.abspath(os.path.join(__file__, "..", "..", "doc", "images"))
    generate_world_class_diagram(sources, output_dir)


if __name__ == '__main__':
    main()
