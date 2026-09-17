"""Sphinx configuration for the cubemarsPyCAN documentation.

Built from the repository root with::

    python -m sphinx -b html -W --keep-going -n docs docs/_build/html

``docs/`` also holds the seven hand-written Markdown guides, which this configuration
reads in place through MyST. ``tests/test_examples_and_docs.py`` asserts exact set
equality over ``docs/*.md``, so no new ``.md`` file may be added to this directory - which
is why the landing page is ``index.rst`` and the API pages live in ``api/``.
"""

from __future__ import annotations

from importlib.metadata import version as _dist_version

project = "cubemarsPyCAN"
author = "Hector Azpurua"
# `project_copyright`, not `copyright`: the latter shadows a builtin and ruff A001 would
# fail the `ruff check .` step in CI on an otherwise green docs build.
project_copyright = "2026, VeRLab, Universidade Federal de Minas Gerais"
release = _dist_version("cubemarspycan")
version = release

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.autosummary",
    "sphinx.ext.intersphinx",
    "sphinx.ext.viewcode",
    "sphinx_copybutton",
    "myst_parser",
]

templates_path: list[str] = []
exclude_patterns = ["_build", ".DS_Store", "Thumbs.db"]
root_doc = "index"

# -- autodoc ---------------------------------------------------------------------------

autodoc_default_options = {
    "members": True,
    "show-inheritance": True,
    # registry.SPECS has a ~47,000-character repr and cli.DEFAULT_URL is computed from
    # platform.system() at import, so its value would differ between a Linux and a macOS
    # build. Neither belongs in a :value: field.
    "no-value": True,
}
autodoc_member_order = "bysource"
autodoc_typehints = "signature"
autodoc_typehints_format = "short"
# Renders `policy: SafetyPolicy = DEFAULT_POLICY` as the name rather than a 600-character
# dataclass repr, and OriginMode.TEMPORARY rather than <OriginMode.TEMPORARY: 0>.
autodoc_preserve_defaults = True
autodoc_class_signature = "mixed"
# Off on purpose. MitMotor and ServoMotor override accepts/on_frame/update; inheriting the
# base text would paper over exactly the documentation gaps this effort exists to close.
autodoc_inherit_docstrings = False
# python-can exposes these at can.BusABC, but their __module__ is can.bus, and only the
# short path is in the published inventory. Every module naming them has
# `from __future__ import annotations`, so the annotations are strings and Sphinx can
# rewrite them before evaluation.
autodoc_type_aliases = {
    "can.BusABC": "can.BusABC",
    "can.Listener": "can.Listener",
    "can.Message": "can.Message",
    "can.Notifier": "can.Notifier",
}

add_module_names = False
python_use_unqualified_type_names = True
python_maximum_signature_line_length = 88
toc_object_entries_show_parents = "hide"

# -- cross references ------------------------------------------------------------------

nitpicky = True
nitpick_ignore = [
    # TypeVars. Sphinx renders these as py:class cross-references, but autodata would
    # register them as py:data, which a py:class reference still would not match. There
    # is no correct target: they are type-system plumbing, not API.
    ("py:class", "cubemarspycan.latch.S"),
    ("py:class", "cubemarspycan.motor.base.StateT"),
    ("py:class", "cubemarspycan.spec.T"),
    # The Generic[...] class signature emits these as py:obj rather than py:class.
    ("py:obj", "cubemarspycan.latch.S"),
    ("py:obj", "cubemarspycan.motor.base.StateT"),
    ("py:obj", "cubemarspycan.spec.T"),
    # autodoc_type_aliases fixes these in signatures, but AttributeDocumenter passes the
    # alias map to get_type_hints() as a localns, where it has no effect.
    ("py:class", "can.bus.BusABC"),
    ("py:class", "can.listener.Listener"),
    ("py:class", "can.message.Message"),
    ("py:class", "can.notifier.Notifier"),
]

intersphinx_mapping = {
    "python": ("https://docs.python.org/3", None),
    "can": ("https://python-can.readthedocs.io/en/stable/", None),
}
intersphinx_timeout = 30

# -- MyST ------------------------------------------------------------------------------

# GFM pipe tables and the `> **Note:**` blockquotes need no extension: tables are core
# MyST, and those blockquotes are blockquotes with a bold lead-in, which is what they are.
# colon_fence is enabled so a future guide can use :::{note} without a syntax change.
myst_enable_extensions = ["colon_fence", "deflist", "fieldlist", "linkify"]
myst_heading_anchors = 3

# -- HTML ------------------------------------------------------------------------------

html_theme = "furo"
html_title = f"cubemarsPyCAN {release}"
html_baseurl = "https://verlab.github.io/cubemarsPyCAN/"
# [] not ["_static"]: a declared directory that does not exist is itself a warning, and
# warnings are errors here.
html_static_path: list[str] = []
