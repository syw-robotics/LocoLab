# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Merge URDF fixed joints (Isaac importer 3.0 semantics) with an optional keep-list."""

from __future__ import annotations

import importlib.util
import logging
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from xml.etree.ElementTree import Element

_logger = logging.getLogger(__name__)

_isaac_urdf: Any = None


def _urdf_utils() -> Any:
    """Load Isaac's URDF helpers without importing the URDF package.

    ``isaacsim.asset.importer.urdf`` registers ``physxSchemaFallback`` while its package
    initializes. OvPhysX then fails when it registers the real ``physxSchema`` plugin for
    the same types. ``urdf_utils`` itself has no package-relative imports, so load that
    file directly.
    """
    global _isaac_urdf
    if _isaac_urdf is not None:
        return _isaac_urdf

    spec = importlib.util.find_spec("isaacsim")
    search_locations = () if spec is None else spec.submodule_search_locations or ()
    relative = Path("asset") / "importer" / "urdf" / "impl" / "urdf_utils.py"
    path = next((Path(root) / relative for root in search_locations if (Path(root) / relative).is_file()), None)
    if path is None:
        raise ModuleNotFoundError("isaacsim.asset.importer.urdf.impl.urdf_utils")

    module_spec = importlib.util.spec_from_file_location("_locolab_isaac_urdf_utils", path)
    if module_spec is None or module_spec.loader is None:
        raise ModuleNotFoundError(str(path))
    module = importlib.util.module_from_spec(module_spec)
    module_spec.loader.exec_module(module)
    _isaac_urdf = module
    return module


def write_merged_urdf(source: Path, output: Path, *, keep_fixed: re.Pattern[str]) -> Path:
    """Write *output* with every mergeable fixed joint collapsed into its parent link."""
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(str(source), parser=parser)
    root = tree.getroot()

    while True:
        batch = _mergeable_fixed_joints(root, keep_fixed)
        if not batch:
            break
        if not _merge_joint(root, batch[0], source):
            root.remove(batch[0])

    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(output), xml_declaration=True, encoding="UTF-8")
    return output


def _mergeable_fixed_joints(root: Element, keep_fixed: re.Pattern[str]) -> list[Element]:
    return [
        joint
        for joint in root.findall("joint")
        if joint.get("type") == "fixed" and not keep_fixed.fullmatch(joint.get("name") or "")
    ]


def _merge_joint(root: Element, joint: Element, source: Path) -> bool:
    name = joint.get("name", "<unnamed>")
    parent_elem = joint.find("parent")
    child_elem = joint.find("child")
    if parent_elem is None or child_elem is None:
        _logger.warning("Fixed joint '%s' in %s missing parent/child; skipping merge.", name, source)
        return False

    parent_name = parent_elem.get("link")
    child_name = child_elem.get("link")
    if not parent_name or not child_name:
        _logger.warning("Fixed joint '%s' in %s missing link name; skipping merge.", name, source)
        return False
    if parent_name == child_name:
        _logger.error("Fixed joint '%s' in %s is a self-loop; skipping merge.", name, source)
        return False

    urdf = _urdf_utils()
    parent_link = urdf._find_link(root, parent_name)
    child_link = urdf._find_link(root, child_name)
    if parent_link is None or child_link is None:
        _logger.error(
            "Fixed joint '%s' in %s references missing link (parent=%s, child=%s); skipping merge.",
            name,
            source,
            parent_name,
            child_name,
        )
        return False

    t_joint = urdf._parse_origin(joint.find("origin"))
    for visual in child_link.findall("visual"):
        urdf._compose_origin(visual, t_joint)
        parent_link.append(visual)
    for collision in child_link.findall("collision"):
        urdf._compose_origin(collision, t_joint)
        parent_link.append(collision)
    urdf._merge_inertial(parent_link, child_link, t_joint)

    for other in root.findall("joint"):
        if other is joint:
            continue
        other_parent = other.find("parent")
        if other_parent is not None and other_parent.get("link") == child_name:
            other_parent.set("link", parent_name)
            t_other = urdf._parse_origin(other.find("origin"))
            urdf._set_origin(other, t_joint @ t_other)

    root.remove(joint)
    root.remove(child_link)
    return True
