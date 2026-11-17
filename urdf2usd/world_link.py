"""World link handling (Gazebo-compatible rule).

* URDF has a link named ``world``  -> fixed base (converter output kept as is).
* URDF has no ``world`` link       -> floating base.

The decision is made by matching the link *name* ``world`` in the URDF, not by
the converter's ghost-link detection. A ``world`` link carrying inertial,
visual or collision elements is rejected.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

WORLD_LINK_NAME = "world"
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
