# -*- coding: utf-8 -*-
#
# Configuration file for the Sphinx documentation builder.
#
# This file does only contain a selection of the most common options. For a
# full list see the documentation:
# http://www.sphinx-doc.org/en/master/config

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join("..", "src")))

# -- Project information -----------------------------------------------------

project = 'semantic_world'
copyright = '2025, AICOR Institute for Artificial Intelligence'
author = 'AICOR Institute for Artificial Intelligence'

# The short X.Y version
version = ''
# The full version, including alpha/beta/rc tags
release = ''

# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.viewcode',
    'sphinx.ext.napoleon',
    'autoapi.extension',
    'nbsphinx',
    'sphinxcontrib.plantuml',
    'sphinxcontrib.programoutput',
    'myst_parser',
]

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store', '**.ipynb_checkpoints']

# The suffix(es) of source filenames.
source_suffix = {
    '.rst': None,
    '.md': 'myst_parser',
    '.ipynb': 'nbsphinx',
}

# -- AutoAPI configuration --------------------------------------------------

autoapi_type = 'python'
autoapi_dirs = ['../src']
autoapi_root = 'api'
autoapi_keep_files = True
autoapi_generate_api_docs = True

# -- NBSphinx configuration -------------------------------------------------

# Execute notebooks when building docs
nbsphinx_execute = 'always'
nbsphinx_allow_errors = False

# Timeout for notebook execution (in seconds)
nbsphinx_timeout = 60

# -- PlantUML configuration ------------------------------------------------

plantuml = 'java -jar plantuml.jar'
plantuml_output_format = 'png'

# -- Options for HTML output ------------------------------------------------

# The theme to use for HTML and HTML Help pages.
html_theme = 'sphinx_book_theme'

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']

# Theme options
html_theme_options = {
    'repository_url': 'https://github.com/cram2/semantic_world',
    'use_repository_button': True,
    'use_issues_button': True,
    'use_edit_page_button': True,
    'path_to_docs': 'doc',
}

html_title = 'Semantic World Documentation'