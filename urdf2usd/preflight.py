"""Pre-flight validation of a URDF's mesh references.

Collects *all* problems before conversion instead of failing on the first:

* mesh file missing
* unsupported mesh extension (supported: stl, obj, dae; case-insensitive)
* unsupported URI scheme (supported: ``package://``, ``file://``, relative
  and absolute paths)
* ``package://`` name not present in the package map
* OBJ whose ``mtllib`` file is missing

Pure stdlib; no pxr required.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Mapping
from urllib.parse import unquote, urlparse

SUPPORTED_MESH_EXTENSIONS = (".stl", ".obj", ".dae")


class PreflightError(ValueError):
    """The URDF failed pre-flight checks; ``problems`` lists every issue."""

    def __init__(self, problems: list[str]):
        self.problems = list(problems)
        super().__init__(
            f"{len(self.problems)} pre-flight problem(s):\n"
            + "\n".join(f"  - {p}" for p in self.problems)
        )


def _resolve(
    uri: str, base: Path, packages: Mapping[str, str | Path]
) -> tuple[Path | None, str | None]:
    """Return (path, None) or (None, problem)."""
    if "://" not in uri:
        p = Path(uri)
        return (p if p.is_absolute() else base / p), None
    parsed = urlparse(uri)
    scheme = parsed.scheme.lower()
    if scheme == "file":
        return Path(unquote(parsed.netloc + parsed.path)), None
    if scheme == "package":
        name = parsed.netloc
        if name not in packages:
            return None, f"package '{name}' not in package map: {uri}"
        return Path(packages[name]) / unquote(parsed.path).lstrip("/"), None
    return None, f"unsupported URI scheme '{scheme}': {uri}"


def _mtllibs(obj: Path) -> list[str]:
    names: list[str] = []
    with obj.open("r", errors="replace") as f:
        for line in f:
            if line.startswith("mtllib"):
                names.extend(line.split()[1:])
    return names


def check_urdf(
    urdf_path: str | Path, packages: Mapping[str, str | Path] | None = None
) -> None:
    """Raise PreflightError listing every mesh problem in the URDF."""
    urdf_path = Path(urdf_path)
    try:
        root = ET.parse(str(urdf_path)).getroot()
    except (ET.ParseError, OSError) as e:
        raise PreflightError([f"cannot parse URDF {urdf_path}: {e}"]) from e

    packages = packages or {}
    problems: list[str] = []
    for mesh in root.iter("mesh"):
        uri = mesh.get("filename")
        if not uri:
            problems.append("mesh element without filename")
            continue
        path, problem = _resolve(uri, urdf_path.parent, packages)
        if problem:
            problems.append(problem)
            continue
        ext = path.suffix.lower()
        if ext not in SUPPORTED_MESH_EXTENSIONS:
            problems.append(f"unsupported mesh extension '{ext.lstrip('.')}': {uri}")
        if not path.is_file():
            problems.append(f"mesh file not found: {uri} ({path})")
            continue
        if ext == ".obj":
            for mtl in _mtllibs(path):
                if not (path.parent / mtl).is_file():
                    problems.append(f"mtllib '{mtl}' missing for OBJ: {uri}")
    if problems:
        raise PreflightError(problems)
