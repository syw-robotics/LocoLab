# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Installation script for the 'locolab' python package."""

import os
from pathlib import Path

import toml
from setuptools import find_packages, setup

# Obtain the extension data from the extension.toml file
EXTENSION_PATH = os.path.dirname(os.path.realpath(__file__))
# Read the extension.toml file
EXTENSION_TOML_DATA = toml.load(os.path.join(EXTENSION_PATH, "config", "extension.toml"))

# Minimum dependencies required prior to installation
INSTALL_REQUIRES = [
    "psutil",
]


def _template_package_data() -> list[str]:
    """Non-Python files copied into generated external projects."""
    template_pkg = Path(EXTENSION_PATH) / "locolab" / "template"
    root = template_pkg / "external_project"
    if not root.is_dir():
        return []
    return [str(path.relative_to(template_pkg)) for path in root.rglob("*") if path.is_file()]


# Installation operation
setup(
    name="locolab",
    packages=find_packages(),
    author=EXTENSION_TOML_DATA["package"]["author"],
    maintainer=EXTENSION_TOML_DATA["package"]["maintainer"],
    url=EXTENSION_TOML_DATA["package"]["repository"],
    version=EXTENSION_TOML_DATA["package"]["version"],
    description=EXTENSION_TOML_DATA["package"]["description"],
    keywords=EXTENSION_TOML_DATA["package"]["keywords"],
    install_requires=INSTALL_REQUIRES,
    license="Apache License 2.0",
    include_package_data=True,
    package_data={"locolab.template": _template_package_data()},
    python_requires=">=3.10",
    entry_points={
        "locolab.tasks": [
            "locolab = locolab.tasks",
        ],
    },
    classifiers=[
        "Natural Language :: English",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Isaac Sim :: 4.5.0",
        "Isaac Sim :: 5.0.0",
        "Isaac Sim :: 5.1.0",
    ],
    zip_safe=False,
)
