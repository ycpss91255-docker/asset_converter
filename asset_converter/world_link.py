"""World link handling (Gazebo-compatible rule).

* URDF has a link named ``world``  -> fixed base (converter output kept as is).
* URDF has no ``world`` link       -> floating base: the synthetic
  ``root_joint`` inserted by urdf-usd-converter is removed.

The decision is made by matching the link *name* ``world`` in the URDF, not by
the converter's ghost-link detection. A ``world`` link carrying inertial,
visual or collision elements is rejected.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

from pxr import Usd, UsdPhysics

WORLD_LINK_NAME = "world"
ROOT_JOINT_NAME = "root_joint"
_FORBIDDEN_CHILDREN = ("inertial", "visual", "collision")


class WorldLinkError(ValueError):
    """The URDF violates the world link rules."""


def urdf_has_world_link(urdf_path: str | Path) -> bool:
    """Return True if the URDF declares a link named ``world``.

    Raises WorldLinkError if that link has inertial/visual/collision elements.
    """
    try:
        root = ET.parse(str(urdf_path)).getroot()
    except ET.ParseError as e:
        raise WorldLinkError(f"cannot parse URDF {urdf_path}: {e}") from e

    found = False
    for link in root.iter("link"):
        if link.get("name") != WORLD_LINK_NAME:
            continue
        found = True
        bad = [c for c in _FORBIDDEN_CHILDREN if link.find(c) is not None]
        if bad:
            raise WorldLinkError(
                f"'{WORLD_LINK_NAME}' link must be empty, found: {', '.join(bad)}"
            )
    return found


def find_synthetic_root_joint(stage: Usd.Stage) -> Usd.Prim | None:
    """Locate the converter's synthetic root joint, or None.

    It is a PhysicsFixedJoint named ``root_joint`` whose body0 is the
    defaultPrim (an Xform without RigidBodyAPI). A real URDF joint named
    ``root_joint`` would have a link as body0, so it is not matched.
    """
    default = stage.GetDefaultPrim()
    if not default:
        return None
    for prim in stage.Traverse():
        if prim.GetName() != ROOT_JOINT_NAME:
            continue
        joint = UsdPhysics.FixedJoint(prim)
        if not joint:
            continue
        if list(joint.GetBody0Rel().GetTargets()) == [default.GetPath()]:
            return prim
    return None


def remove_synthetic_root_joint(stage: Usd.Stage) -> bool:
    """Remove ``root_joint`` from every layer that defines it.

    Edits the layers in place (does not save them). Returns True if removed.
    """
    prim = find_synthetic_root_joint(stage)
    if prim is None:
        return False
    for spec in list(prim.GetPrimStack()):
        parent = spec.nameParent
        if parent is not None:
            del parent.nameChildren[spec.name]
    return True


def apply_world_link_rule(urdf_path: str | Path, usd_path: str | Path) -> bool:
    """Apply the world link rule to converter output, saving edited layers.

    Returns True if ``root_joint`` was removed (floating base).
    """
    has_world = urdf_has_world_link(urdf_path)
    stage = Usd.Stage.Open(str(usd_path))
    if stage is None:
        raise WorldLinkError(f"cannot open USD {usd_path}")
    if has_world:
        return False

    prim = find_synthetic_root_joint(stage)
    layers = {s.layer for s in prim.GetPrimStack()} if prim else set()
    removed = remove_synthetic_root_joint(stage)
    for layer in layers:
        layer.Save()
    return removed
