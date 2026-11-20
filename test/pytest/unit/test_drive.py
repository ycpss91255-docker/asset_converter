import math
import subprocess
import textwrap

import pytest
from pxr import Usd, UsdPhysics

from asset_converter.drive import (
    DriveError,
    DriveParams,
    apply_drive,
    apply_drives,
    defaults_from_urdf_attrs,
    drive_axis,
)
from asset_converter.world_link import apply_world_link_rule


def _make_usd(tmp_path, extra=""):
    extra = textwrap.indent(extra, " " * 16).lstrip()
    p = tmp_path / "r.usda"
    p.write_text(textwrap.dedent(f"""\
        #usda 1.0
        (defaultPrim = "r")
        def Xform "r"
        {{
            def Xform "base_link" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {{}}
            def Xform "wheel" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {{}}
            def Scope "Physics"
            {{
                def PhysicsFixedJoint "root_joint"
                {{
                    rel physics:body0 = </r>
                    rel physics:body1 = </r/base_link>
                }}
                def PhysicsRevoluteJoint "wj"
                {{
                    rel physics:body0 = </r/base_link>
                    rel physics:body1 = </r/wheel>
                    double newton:damping = 0.5
                    double urdf:limit:effort = 10
                }}
                def PhysicsPrismaticJoint "sj"
                {{
                    rel physics:body0 = </r/base_link>
                    rel physics:body1 = </r/wheel>
                    double newton:damping = 3
                    double urdf:limit:effort = 7
                }}
                def PhysicsFixedJoint "fx"
                {{
                    rel physics:body0 = </r/base_link>
                    rel physics:body1 = </r/wheel>
                }}
                def PhysicsSphericalJoint "sp"
                {{
                    rel physics:body0 = </r/base_link>
                    rel physics:body1 = </r/wheel>
                }}
                {extra}
            }}
        }}
        """))
    return p


def _drive(stage, path, axis):
    return UsdPhysics.DriveAPI(stage.GetPrimAtPath(path), axis)


def test_axis_per_joint_type(tmp_path):
    st = Usd.Stage.Open(str(_make_usd(tmp_path)))
    assert drive_axis(st.GetPrimAtPath("/r/Physics/wj")) == "angular"
    assert drive_axis(st.GetPrimAtPath("/r/Physics/sj")) == "linear"
    assert drive_axis(st.GetPrimAtPath("/r/Physics/fx")) is None
    assert drive_axis(st.GetPrimAtPath("/r/Physics/sp")) is None


def test_defaults_from_attrs(tmp_path):
    st = Usd.Stage.Open(str(_make_usd(tmp_path)))
    p = defaults_from_urdf_attrs(st.GetPrimAtPath("/r/Physics/wj"))
    assert p == DriveParams(stiffness=0.0, damping=0.5, max_force=10.0)
    assert p.drive_type == "force"


def test_defaults_missing_or_zero_effort_not_written(tmp_path):
    extra = ('def PhysicsRevoluteJoint "a"\n{\n double urdf:limit:effort = 0\n}\n'
             'def PhysicsRevoluteJoint "b"\n{\n}')
    st = Usd.Stage.Open(str(_make_usd(tmp_path, extra)))
    for n in "ab":
        p = defaults_from_urdf_attrs(st.GetPrimAtPath(f"/r/Physics/{n}"))
        assert p.max_force is None and p.damping == 0.0


def test_negative_values_rejected(tmp_path):
    extra = 'def PhysicsRevoluteJoint "neg"\n{\n double newton:damping = -1\n}'
    st = Usd.Stage.Open(str(_make_usd(tmp_path, extra)))
    with pytest.raises(DriveError):
        defaults_from_urdf_attrs(st.GetPrimAtPath("/r/Physics/neg"))
    with pytest.raises(DriveError):
        apply_drive(st.GetPrimAtPath("/r/Physics/wj"), DriveParams(max_force=-1))


def test_apply_drives_values_no_unit_conversion(tmp_path):
    usd = _make_usd(tmp_path)
    assert apply_drives(usd) == ["wj", "sj"]
    st = Usd.Stage.Open(str(usd))
    w = _drive(st, "/r/Physics/wj", "angular")
    assert "PhysicsDriveAPI:angular" in st.GetPrimAtPath("/r/Physics/wj").GetAppliedSchemas()
    # newton:damping is already per degree: copied verbatim
    assert w.GetDampingAttr().Get() == pytest.approx(0.5)
    assert w.GetStiffnessAttr().Get() == 0.0
    assert w.GetMaxForceAttr().Get() == pytest.approx(10.0)
    assert w.GetTypeAttr().Get() == "force"
    s = _drive(st, "/r/Physics/sj", "linear")
    assert "PhysicsDriveAPI:linear" in st.GetPrimAtPath("/r/Physics/sj").GetAppliedSchemas()
    assert s.GetDampingAttr().Get() == pytest.approx(3.0)
    assert s.GetMaxForceAttr().Get() == pytest.approx(7.0)


def test_fixed_root_and_other_joints_skipped(tmp_path):
    usd = _make_usd(tmp_path)
    apply_drives(usd)
    st = Usd.Stage.Open(str(usd))
    for n in ("root_joint", "fx", "sp"):
        assert st.GetPrimAtPath(f"/r/Physics/{n}").GetAppliedSchemas() == []


def test_mimic_joint_skipped(tmp_path):
    extra = ('def PhysicsRevoluteJoint "mm"\n{\n'
             ' rel newton:mimicJoint = </r/Physics/wj>\n}')
    usd = _make_usd(tmp_path, extra)
    assert "mm" not in apply_drives(usd)
    st = Usd.Stage.Open(str(usd))
    assert st.GetPrimAtPath("/r/Physics/mm").GetAppliedSchemas() == []


def test_idempotent(tmp_path):
    usd = _make_usd(tmp_path)
    apply_drives(usd)
    first = usd.read_text()
    assert apply_drives(usd) == ["wj", "sj"]
    assert usd.read_text() == first


def test_overrides_win_and_others_untouched(tmp_path):
    usd = _make_usd(tmp_path)
    apply_drives(usd, {"wj": {"stiffness": 100.0, "max_force": 99.0}})
    st = Usd.Stage.Open(str(usd))
    w = _drive(st, "/r/Physics/wj", "angular")
    assert w.GetStiffnessAttr().Get() == 100.0
    assert w.GetMaxForceAttr().Get() == 99.0
    assert w.GetDampingAttr().Get() == pytest.approx(0.5)  # urdf value kept
    assert _drive(st, "/r/Physics/sj", "linear").GetStiffnessAttr().Get() == 0.0


def test_overrides_accept_driveparams(tmp_path):
    usd = _make_usd(tmp_path)
    apply_drives(usd, {"sj": DriveParams(stiffness=5.0, damping=1.0)})
    st = Usd.Stage.Open(str(usd))
    assert _drive(st, "/r/Physics/sj", "linear").GetDampingAttr().Get() == 1.0


@pytest.mark.parametrize("ov", [
    {"nope": {"stiffness": 1.0}},          # unknown joint
    {"wj": {"bogus": 1.0}},                # unknown field
    {"fx": {"stiffness": 1.0}},            # joint that gets no drive
])
def test_bad_overrides_rejected(tmp_path, ov):
    with pytest.raises(DriveError):
        apply_drives(_make_usd(tmp_path), ov)


def test_cannot_open(tmp_path):
    with pytest.raises(DriveError):
        apply_drives(tmp_path / "missing.usda")


# --- real urdf-usd-converter 0.4.0: these assertions are the probe results
# (design #21 open questions 1, 2, 3, 4, 7) -------------------------------

LINK = ('<link name="{n}"><inertial><mass value="1"/><inertia ixx="1" iyy="1" '
        'izz="1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry>'
        '<box size="1 1 1"/></geometry></collision></link>')
URDF = ('<robot name="r">'
        + "".join(LINK.format(n=n) for n in
                  ("base_link", "wheel-a", "arm", "slide", "mim", "mount")) + '''
<joint name="w-1" type="continuous"><parent link="base_link"/><child link="wheel-a"/><axis xyz="0 1 0"/><dynamics damping="2.0" friction="0.5"/><limit effort="10" velocity="3"/></joint>
<joint name="arm j" type="revolute"><parent link="base_link"/><child link="arm"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="5" velocity="2"/><dynamics damping="1.0"/></joint>
<joint name="s" type="prismatic"><parent link="base_link"/><child link="slide"/><axis xyz="1 0 0"/><limit lower="0" upper="1" effort="7" velocity="1"/><dynamics damping="3.0"/></joint>
<joint name="m" type="revolute"><parent link="base_link"/><child link="mim"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="5" velocity="2"/><mimic joint="arm j" multiplier="2" offset="0.1"/></joint>
<joint name="fx" type="fixed"><parent link="base_link"/><child link="mount"/></joint>
</robot>''')


@pytest.fixture
def converted(tmp_path):
    urdf = tmp_path / "r.urdf"
    urdf.write_text(URDF)
    out = tmp_path / "out"
    subprocess.run(["urdf_usd_converter", str(urdf), str(out)], check=True,
                   capture_output=True, text=True)
    return urdf, out


def _joint(stage, urdf_name):
    for p in stage.Traverse():
        if p.IsA(UsdPhysics.Joint) and (p.GetDisplayName() or p.GetName()) == urdf_name:
            return p
    raise AssertionError(urdf_name)


def test_probe_python_has_tomllib():  # Q7
    import tomllib  # noqa: F401


def test_probe_physics_and_no_physx_schema():  # Q3
    assert UsdPhysics.DriveAPI
    with pytest.raises(ImportError):
        from pxr import PhysxSchema  # noqa: F401


def test_probe_damping_is_already_usd_units(converted):  # Q1
    st = Usd.Stage.Open(str(converted[1] / "r.usda"))
    # URDF damping 2.0 Nm*s/rad -> converter already wrote Nm*s/deg
    assert _joint(st, "w-1").GetAttribute("newton:damping").Get() == \
        pytest.approx(2.0 * math.pi / 180)
    assert _joint(st, "arm j").GetAttribute("newton:damping").Get() == \
        pytest.approx(1.0 * math.pi / 180)
    # prismatic is untouched (SI)
    assert _joint(st, "s").GetAttribute("newton:damping").Get() == pytest.approx(3.0)
    # effort is a plain force/torque
    assert _joint(st, "w-1").GetAttribute("urdf:limit:effort").Get() == 10.0


def test_probe_joint_names_are_sanitized_displayname_keeps_urdf(converted):  # Q4
    st = Usd.Stage.Open(str(converted[1] / "r.usda"))
    w = _joint(st, "w-1")
    assert w.GetName() != "w-1"
    assert _joint(st, "arm j").GetName() != "arm j"
    assert _joint(st, "s").GetName() == "s"


def test_probe_mimic_representation(converted):  # Q2
    st = Usd.Stage.Open(str(converted[1] / "r.usda"))
    m = _joint(st, "m")
    assert m.GetRelationship("newton:mimicJoint").GetTargets() == \
        [_joint(st, "arm j").GetPath()]
    assert m.GetAttribute("newton:mimicCoef1").Get() == 2.0
    assert not _joint(st, "arm j").HasRelationship("newton:mimicJoint")


def test_real_converter_drives(converted):
    urdf, out = converted
    usd = out / "r.usda"
    assert apply_drives(usd) == ["w-1", "arm j", "s"]
    st = Usd.Stage.Open(str(usd))
    w = UsdPhysics.DriveAPI(_joint(st, "w-1"), "angular")
    assert w.GetDampingAttr().Get() == pytest.approx(2.0 * math.pi / 180)
    assert w.GetMaxForceAttr().Get() == 10.0
    assert UsdPhysics.DriveAPI(_joint(st, "s"), "linear").GetMaxForceAttr().Get() == 7.0
    # joint count == drive count (+ skipped: fixed, root_joint, mimic)
    joints = [p for p in st.Traverse() if p.IsA(UsdPhysics.Joint)]
    driven = [p for p in joints if any(s.startswith("PhysicsDriveAPI")
                                       for s in p.GetAppliedSchemas())]
    assert len(driven) == 3 and len(joints) == 6
    # written into the Physics payload layer, not the root layer
    assert "DriveAPI" in (out / "Payload" / "Physics.usda").read_text()
    assert "DriveAPI" not in usd.read_text()


def test_real_converter_world_link_order_independent(converted, tmp_path):
    urdf, out = converted
    apply_drives(out / "r.usda")
    apply_world_link_rule(urdf, out / "r.usda")
    a = (out / "Payload" / "Physics.usda").read_text()

    out2 = tmp_path / "out2"
    subprocess.run(["urdf_usd_converter", str(urdf), str(out2)], check=True,
                   capture_output=True, text=True)
    apply_world_link_rule(urdf, out2 / "r.usda")
    apply_drives(out2 / "r.usda")
    assert (out2 / "Payload" / "Physics.usda").read_text() == a
    st = Usd.Stage.Open(str(out2 / "r.usda"))
    assert not st.GetPrimAtPath("/r/Physics/root_joint")


def test_real_converter_override_by_urdf_name(converted):
    _, out = converted
    apply_drives(out / "r.usda", {"arm j": {"stiffness": 7.0}})
    st = Usd.Stage.Open(str(out / "r.usda"))
    assert UsdPhysics.DriveAPI(_joint(st, "arm j"), "angular").GetStiffnessAttr().Get() == 7.0
