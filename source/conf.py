# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'MotoROS2'
copyright = '2023, Ted Miller (Yaskawa America Inc), G.A. vd. Hoorn (Delft University of Technology)'
author = 'Ted Miller (Yaskawa America Inc), G.A. vd. Hoorn (Delft University of Technology)'
release = "0.1.0"


# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    # "myst_parser",
    "sphinx.ext.extlinks",
    "sphinx_copybutton",
    "sphinx_inline_tabs",  # https://sphinx-inline-tabs.readthedocs.io/en/
    "sphinx_design",  # https://sphinx-design.readthedocs.io/en/
    "sphinx_favicon", # https://pypi.org/project/sphinx-favicon/
]

templates_path = ['_templates']
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]


# -- Options for myst ----------------------------------------------------
#

# myst_enable_extensions = ["deflist"]
# myst_gfm_only = True

# myst_enable_extensions = [
#     # "amsmath",
#     # "attrs_inline",
#     # "colon_fence",
#     # "deflist",
#     # "dollarmath",
#     # "fieldlist",
#     "html_admonition",
#     # "html_image",
#     # "linkify",
#     # "replacements",
#     # "smartquotes",
#     # "strikethrough",
#     # "substitution",
#     # "tasklist",
# ]


# -- Options for extlinks ----------------------------------------------------
#

extlinks = {
    "github": ("https://github.com/yaskawa-global/motoros2/issues/%s", "%s"),
}


# -- Options for favicons -----------------------------------------------------
#

favicons = [
    {"href": "favicon-16x16.png"},
    {"href": "favicon-32x32.png"},
]


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'furo'
html_static_path = ['_static']
html_logo = "_static/yaskawa-y-logo.png"
html_css_files = ["css/custom.css"]

html_title = "MotoROS2 documentation"

pygments_style = "sphinx"
pygments_dark_style = "monokai"

html_theme_options = {
    "light_css_variables": {
        # "color-brand-primary": "#1976d2",
        # "color-brand-content": "#1976d2",
        # "color-admonition-background": "orange",
        # "color-sidebar-background": "#1976d2",
    },
    "source_repository": "https://github.com/yaskawa-global/motoros2/",
    "source_branch": "main",
    "source_directory": "doc/",
}
