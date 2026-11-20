"""Regression tests pinning urdf-usd-converter 0.4.0 behaviour for OBJ meshes
and sensor frame links (D455-style), plus how apply_world_link_rule and
apply_drives behave on that output. Test-only: no production code involved
beyond the two post-processing entry points."""

import subprocess

import pytest
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade

from asset_converter.drive import apply_drives
from asset_converter.world_link import apply_world_link_rule

INERTIAL = ('<inertial><mass value="1"/><inertia ixx="1" iyy="1" izz="1" '
            'ixy="0" ixz="0" iyz="0"/></inertial>')
HALF_PI = "1.5707963267948966"


def _convert(urdf, out):
    subprocess.run(["urdf_usd_converter", str(urdf), str(out)], check=True,
                   capture_output=True, text=True)
    return Usd.Stage.Open(str(out / f"{urdf.stem}.usda"))


def _prims(stage, parent_name):
    return [p for p in stage.Traverse() if p.GetPath().GetParentPath().name == parent_name]


# --- OBJ meshes ----------------------------------------------------------

@pytest.fixture
def obj_dir(tmp_path):
    (tmp_path / "m.obj").write_text(
        "mtllib m.mtl\nv 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\n"
        "usemtl red\nf 1 2 3\nusemtl blue\nf 1 3 4\n")
    (tmp_path / "m.mtl").write_text(
        "newmtl red\nKd 1 0 0\nnewmtl blue\nKd 0 0 1\n")
    (tmp_path / "n.obj").write_text(
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 2 3\nf 1 3 4\n")
    return tmp_path


def _obj_urdf(obj_dir, body):
    p = obj_dir / "o.urdf"
    p.write_text(f'<robot name="o">{body}</robot>')
    return p


def _mesh(f):
    return f'<geometry><mesh filename="{f}"/></geometry>'


def test_obj_with_mtl_visual_and_collision(obj_dir):
    urdf = _obj_urdf(obj_dir, f'<link name="base_link">{INERTIAL}'
                     f'<visual>{_mesh("m.obj")}</visual>'
                     f'<collision>{_mesh("m.obj")}</collision></link>')
    stage = _convert(urdf, obj_dir / "out")
    base = "/o/Geometry/base_link"

    # visual: one GeomSubset per material, each bound to its Material
    visual = stage.GetPrimAtPath(f"{base}/m")
    assert visual.IsA(UsdGeom.Mesh)
    assert not visual.HasAPI(UsdPhysics.CollisionAPI)
    subsets = [c for c in visual.GetChildren() if c.IsA(UsdGeom.Subset)]
    assert len(subsets) == 2
    bound = {UsdShade.MaterialBindingAPI(s).ComputeBoundMaterial()[0].GetPath().name
             for s in subsets}
    assert bound == {"red", "blue"}
    assert stage.GetPrimAtPath("/o/Materials/red").IsA(UsdShade.Material)
    assert stage.GetPrimAtPath("/o/Materials/blue").IsA(UsdShade.Material)

    # collision: guide purpose, convexHull approximation, not rendered
    coll = [p for p in _prims(stage, "base_link")
            if p.IsA(UsdGeom.Mesh) and p.HasAPI(UsdPhysics.CollisionAPI)]
    assert len(coll) == 1
    assert coll[0].HasAPI(UsdPhysics.MeshCollisionAPI)
    assert UsdGeom.Imageable(coll[0]).GetPurposeAttr().Get() == UsdGeom.Tokens.guide
    assert UsdPhysics.MeshCollisionAPI(coll[0]).GetApproximationAttr().Get() == "convexHull"


def test_obj_without_mtl_converts(obj_dir):
    urdf = _obj_urdf(obj_dir, f'<link name="base_link">{INERTIAL}'
                     f'<visual>{_mesh("n.obj")}</visual></link>')
    stage = _convert(urdf, obj_dir / "out")  # check=True: no converter error
    mesh = stage.GetPrimAtPath("/o/Geometry/base_link/n")
    assert mesh.IsA(UsdGeom.Mesh)
    assert len(UsdGeom.Mesh(mesh).GetFaceVertexCountsAttr().Get()) == 2
    assert not [c for c in mesh.GetChildren() if c.IsA(UsdGeom.Subset)]


# --- D455-style sensor frames -------------------------------------------

D455 = f'''<robot name="d">
<link name="base_link">{INERTIAL}<collision><geometry><box size="1 1 1"/></geometry></collision></link>
<link name="camera_link">{INERTIAL}<visual><geometry><box size=".1 .1 .1"/></geometry></visual></link>
<link name="camera_color_optical_frame"/>
<link name="camera_depth_optical_frame"/>
<joint name="cam_mount" type="fixed"><parent link="base_link"/><child link="camera_link"/><origin xyz="1 2 3" rpy="0 0 {HALF_PI}"/></joint>
<joint name="color_optical_joint" type="fixed"><parent link="camera_link"/><child link="camera_color_optical_frame"/><origin xyz="0 0.059 0" rpy="-{HALF_PI} 0 -{HALF_PI}"/></joint>
<joint name="depth_optical_joint" type="fixed"><parent link="camera_link"/><child link="camera_depth_optical_frame"/><origin xyz="0 0 0" rpy="-{HALF_PI} 0 -{HALF_PI}"/></joint>
</robot>'''


@pytest.fixture
def d455(tmp_path):
    urdf = tmp_path / "d.urdf"
    urdf.write_text(D455)
    return urdf, tmp_path / "out"


def _world(prim):
    return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)


def _close(a, b):
    return Gf.IsClose(Gf.Vec3d(*a), Gf.Vec3d(*b), 1e-6)


def test_frame_links_become_xforms_with_urdf_names(d455):
    stage = _convert(*d455)
    cam = stage.GetPrimAtPath("/d/Geometry/base_link/camera_link")
    assert cam.HasAPI(UsdPhysics.RigidBodyAPI)
    for name in ("camera_color_optical_frame", "camera_depth_optical_frame"):
        frame = stage.GetPrimAtPath(f"/d/Geometry/base_link/camera_link/{name}")
        assert frame.GetTypeName() == "Xform"
        assert not frame.HasAPI(UsdPhysics.RigidBodyAPI)
        assert not frame.HasAPI(UsdPhysics.MassAPI)


def test_frame_world_pose(d455):
    stage = _convert(*d455)
    base = "/d/Geometry/base_link/camera_link"
    cam = _world(stage.GetPrimAtPath(base))
    assert _close(cam.ExtractTranslation(), (1, 2, 3))
    # camera_link yaw +90deg: its +x (forward) points along world +y
    assert _close(cam.TransformDir(Gf.Vec3d(1, 0, 0)), (0, 1, 0))

    color = _world(stage.GetPrimAtPath(f"{base}/camera_color_optical_frame"))
    depth = _world(stage.GetPrimAtPath(f"{base}/camera_depth_optical_frame"))
    # origin offset of 0.059 along camera_link +y (= world -x after the yaw)
    assert _close(color.ExtractTranslation(), (1 - 0.059, 2, 3))
    assert _close(depth.ExtractTranslation(), (1, 2, 3))
    # optical convention: z forward, x right, y down relative to camera_link
    for m in (color, depth):
        assert _close(m.TransformDir(Gf.Vec3d(0, 0, 1)), (0, 1, 0))   # z = cam x
        assert _close(m.TransformDir(Gf.Vec3d(1, 0, 0)), (1, 0, 0))   # x = -cam y
        assert _close(m.TransformDir(Gf.Vec3d(0, 1, 0)), (0, 0, -1))  # y = -cam z


def test_no_physics_joint_for_massless_frame_joints(d455):
    stage = _convert(*d455)
    joints = {p.GetDisplayName() or p.GetName()
              for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)}
    assert "cam_mount" in joints  # massful link keeps its fixed joint
    assert not joints & {"color_optical_joint", "depth_optical_joint"}


def test_world_link_rule_and_drives_on_frames(d455):
    urdf, out = d455
    _convert(urdf, out)
    usd = out / "d.usda"
    assert apply_world_link_rule(urdf, usd) is True  # no 'world': floating
    assert apply_drives(usd) == []                    # only fixed joints
    stage = Usd.Stage.Open(str(usd))
    assert not stage.GetPrimAtPath("/d/Physics/root_joint")
    assert stage.GetPrimAtPath("/d/Physics/cam_mount")
    frame = stage.GetPrimAtPath(
        "/d/Geometry/base_link/camera_link/camera_color_optical_frame")
    assert frame.GetTypeName() == "Xform"
    assert not any(p.HasAPI(UsdPhysics.DriveAPI, n)
                   for p in stage.Traverse() for n in ("angular", "linear"))


def test_world_link_present_keeps_root_joint(tmp_path):
    urdf = tmp_path / "d.urdf"
    urdf.write_text(D455.replace(
        '<link name="base_link">',
        '<link name="world"/><joint name="wj" type="fixed"><parent link="world"/>'
        '<child link="base_link"/></joint><link name="base_link">', 1))
    out = tmp_path / "out"
    _convert(urdf, out)
    assert apply_world_link_rule(urdf, out / "d.usda") is False
    assert apply_drives(out / "d.usda") == []
