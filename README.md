# Welcome to the Semantic World Package

The semantic world is a Python package for querying and manipulating robot simulation data.  
It originates from PyCRAM's abstract world and unifies the functionality needed by multiple packages.

# Installation


You can install the package directly from PyPI:

```bash
pip install semantic_world
```

To install and set up the package locally along with its dependencies, follow these steps:

```bash
git clone https://github.com/cram2/semantic_world.git
cd semantic_world

python -m pip install --upgrade pip
pip install virtualenv
python -m virtualenv venv
source venv/bin/activate

sudo apt-get update
sudo apt install graphviz graphviz-dev

pip install -r requirements.txt
pip install pytest mypy flake8 black isort
pip install .
```

# Running Tests

```bash
source venv/bin/activate
pytest -v test/
```

Read the docs [here](https://cram2.github.io/semantic_world/intro.html)!