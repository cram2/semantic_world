# Welcome to the Semantic World Package

The semantic world is a Python package for querying and manipulating robot simulation data.  
It originates from PyCRAM's abstract world and unifies the functionality needed by multiple packages.

# User Installation


You can install the package directly from PyPI:

```bash
pip install -U semantic_world
```

# Contributing

If you are interested in contributing, you can check out the source code from GitHub:

```bash
git clone https://github.com/cram2/semantic_world.git
```

### Development Dependencies

```bash
sudo apt install -y graphviz graphviz-dev
pip install -r requirements.txt
```


# Tests
The tests can be run with `pytest` directly in PyCharm or from the terminal after installing Semantic World as a python package.

```bash
pip install -e .
pytest test/
```

# Documentation

You can read the official documentation [here](https://cram2.github.io/semantic_world/intro.html)!