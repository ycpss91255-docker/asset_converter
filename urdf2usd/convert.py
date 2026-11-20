"""URDF -> USD conversion wrapper.

Calls urdf-usd-converter with an explicit ``package://`` map, then applies the
world link rule and the URDF-derived drives.

The converter only auto-resolves ``package://`` when the URDF sits inside the
package tree, and when a mesh cannot be resolved it logs a warning and emits an
empty Xform (silent degradation, which also drops collision). This wrapper
runs preflight.check_urdf first (PreflightError propagates unchanged), then
compares the meshes referenced by the URDF with the Mesh prims in the output
and raises ConvertError when some are missing (second guard).
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Mapping

from pxr import Tf, Usd, UsdGeom
from urdf_usd_converter import Converter

from urdf2usd.drive import apply_drives
from urdf2usd.preflight import check_urdf
from urdf2usd.world_link import apply_world_link_rule


class ConvertError(ValueError):
    """Conversion failed or silently lost geometry."""


def _ros_packages(
    packages: Mapping[str, str | Path] | None, urdf_dir: Path
) -> list[dict[str, str]]:
    """Absolute package paths (relative ones are relative to the URDF dir)."""
    result = []
    for name, path in (packages or {}).items():
        p = Path(path)
        if not p.is_absolute():
            p = urdf_dir / p
        result.append({"name": name, "path": str(p.resolve())})
    return result


def _count_urdf_meshes(urdf_path: Path) -> dict[str, int]:
    try:
        root = ET.parse(str(urdf_path)).getroot()
    except ET.ParseError as e:
        raise ConvertError(f"cannot parse URDF {urdf_path}: {e}") from e
    counts = {"visual": 0, "collision": 0}
    for kind in counts:
        for elem in root.iter(kind):
            if elem.find("geometry/mesh") is not None:
                counts[kind] += 1
    return counts


def _check_meshes(urdf_path: Path, usd_path: Path) -> None:
    expected = _count_urdf_meshes(urdf_path)
    stage = Usd.Stage.Open(str(usd_path))
    found = {"visual": 0, "collision": 0}
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            purpose = UsdGeom.Imageable(prim).GetPurposeAttr().Get()
            found["collision" if purpose == UsdGeom.Tokens.guide else "visual"] += 1
    missing = {k: expected[k] - found[k] for k in expected if found[k] < expected[k]}
    if missing:
        raise ConvertError(
            f"{urdf_path}: mesh(es) missing in converter output "
            f"({', '.join(f'{n} {k}' for k, n in missing.items())}); "
            "unresolved package:// or missing mesh file? pass `packages`"
        )


def convert_urdf(
    urdf_path: str | Path,
    out_dir: str | Path,
    packages: Mapping[str, str | Path] | None = None,
) -> Path:
    """Convert a URDF and post-process it. Returns the root usda path."""
    urdf = Path(urdf_path).resolve()
    out = Path(out_dir).resolve()
    # Raises PreflightError (unwrapped) before the converter can silently
    # degrade; relative package paths resolve against the URDF dir.
    check_urdf(urdf, packages)
    converter = Converter(ros_packages=_ros_packages(packages, urdf.parent))
    try:
        asset = converter.convert(str(urdf), str(out))
    except (ValueError, Tf.ErrorException) as e:
        raise ConvertError(f"cannot convert {urdf}: {e}") from e
    usd_path = Path(asset.path)
    if not usd_path.is_absolute():
        usd_path = out / usd_path
    _check_meshes(urdf, usd_path)
    apply_world_link_rule(urdf, usd_path)
    apply_drives(usd_path)
    return usd_path
