"""UsdPhysics.DriveAPI from URDF-derived values (design: urdf2usd#21).

For every actuated joint in the converter output, apply ``DriveAPI``
(``angular`` for revolute/continuous, ``linear`` for prismatic):

* ``maxForce``  <- ``urdf:limit:effort`` (only if > 0)
* ``damping``   <- ``newton:damping`` (default 0)
* ``stiffness`` -> 0 (no position control is assumed without explicit input)

Units: probed against urdf-usd-converter 0.4.0, ``newton:damping`` is already
in USD units (per degree for angular joints, i.e. URDF damping * pi/180), so
values are copied verbatim and no unit conversion happens in this module.

Skipped: fixed joints (incl. the synthetic ``root_joint``), joints of other
types, and mimic joints (``newton:mimicJoint``), which must follow their
leader rather than be driven independently.

Joints are identified by their URDF name: the converter sanitizes prim names
(``w-1`` -> ``tn__w1_h3``) and keeps the original in ``displayName``.
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass
from pathlib import Path
from typing import Literal, Mapping

from pxr import Sdf, Tf, Usd, UsdPhysics

DAMPING_ATTR = "newton:damping"
EFFORT_ATTR = "urdf:limit:effort"
MIMIC_REL = "newton:mimicJoint"


class DriveError(ValueError):
    """Invalid drive input or output."""


@dataclass(frozen=True)
class DriveParams:
    drive_type: Literal["force", "acceleration"] = "force"
    stiffness: float = 0.0
    damping: float = 0.0
    max_force: float | None = None  # None -> not written (stays inf)


def urdf_joint_name(prim: Usd.Prim) -> str:
    return prim.GetDisplayName() or prim.GetName()


def drive_axis(prim: Usd.Prim) -> Literal["angular", "linear"] | None:
    """DriveAPI instance for a joint prim, or None if it gets no drive."""
    if prim.HasRelationship(MIMIC_REL):
        return None
    if prim.IsA(UsdPhysics.RevoluteJoint):
        return "angular"
    if prim.IsA(UsdPhysics.PrismaticJoint):
        return "linear"
    return None


def _validate(params: DriveParams, name: str) -> DriveParams:
    if params.drive_type not in ("force", "acceleration"):
        raise DriveError(f"{name}: bad drive type {params.drive_type!r}")
    for field in ("stiffness", "damping"):
        if getattr(params, field) < 0:
            raise DriveError(f"{name}: {field} must be >= 0")
    if params.max_force is not None and params.max_force < 0:
        raise DriveError(f"{name}: max_force must be >= 0")
    return params


def defaults_from_urdf_attrs(prim: Usd.Prim) -> DriveParams:
    damping = prim.GetAttribute(DAMPING_ATTR).Get()
    effort = prim.GetAttribute(EFFORT_ATTR).Get()
    if effort is not None and effort < 0:
        raise DriveError(f"{urdf_joint_name(prim)}: negative effort {effort}")
    return _validate(
        DriveParams(
            damping=float(damping) if damping is not None else 0.0,
            max_force=float(effort) if effort is not None and effort > 0 else None,
        ),
        urdf_joint_name(prim),
    )


def _defining_target(prim: Usd.Prim) -> Usd.EditTarget:
    """Edit target for the layer that defines the joint (e.g. Physics payload).

    That layer is not in the stage's local layer stack, so the target is
    built from the prim index node that contributes it.
    """
    for spec in prim.GetPrimStack():
        if spec.specifier == Sdf.SpecifierDef:
            layer = spec.layer
            break
    else:
        raise DriveError(f"{prim.GetPath()}: no defining spec")
    stage = prim.GetStage()
    if layer in stage.GetLayerStack():
        return Usd.EditTarget(layer)
    for node in prim.GetPrimIndex().rootNode.children:
        if layer in node.layerStack.layers:
            return Usd.EditTarget(layer, node)
    raise DriveError(f"{prim.GetPath()}: cannot target layer {layer.identifier}")


def apply_drive(prim: Usd.Prim, params: DriveParams) -> None:
    """Write DriveAPI on one joint, into the layer that defines it. Idempotent."""
    axis = drive_axis(prim)
    if axis is None:
        raise DriveError(f"{urdf_joint_name(prim)}: joint gets no drive")
    _validate(params, urdf_joint_name(prim))
    with Usd.EditContext(prim.GetStage(), _defining_target(prim)):
        drive = UsdPhysics.DriveAPI.Apply(prim, axis)
        drive.CreateTypeAttr().Set(params.drive_type)
        drive.CreateStiffnessAttr().Set(params.stiffness)
        drive.CreateDampingAttr().Set(params.damping)
        if params.max_force is not None:
            drive.CreateMaxForceAttr().Set(params.max_force)


def apply_drives(
    usd_path: str | Path,
    overrides: Mapping[str, DriveParams | Mapping[str, object]] | None = None,
) -> list[str]:
    """Apply drives to every actuated joint, save edited layers.

    ``overrides`` maps URDF joint names to a DriveParams or a dict of
    DriveParams fields, which win over URDF-derived values. Returns the URDF
    names of the driven joints.
    """
    try:
        stage = Usd.Stage.Open(str(usd_path))
    except Tf.ErrorException as e:
        raise DriveError(f"cannot open USD {usd_path}") from e
    joints = {}
    for prim in stage.Traverse():
        if drive_axis(prim) is not None:
            joints[urdf_joint_name(prim)] = prim
    overrides = dict(overrides or {})
    for name in overrides:
        if name not in joints:
            raise DriveError(f"override for unknown or non-actuated joint: {name}")

    fields = {f.name for f in dataclasses.fields(DriveParams)}
    layers = set()
    for name, prim in joints.items():
        params = defaults_from_urdf_attrs(prim)
        ov = overrides.get(name)
        if isinstance(ov, DriveParams):
            params = ov
        elif ov:
            bad = set(ov) - fields
            if bad:
                raise DriveError(f"{name}: unknown override field(s) {sorted(bad)}")
            params = dataclasses.replace(params, **ov)
        apply_drive(prim, params)
        layers.update(s.layer for s in prim.GetPrimStack())
    for layer in layers:
        if layer.dirty:
            layer.Save()
    return list(joints)
