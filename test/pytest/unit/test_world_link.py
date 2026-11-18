import subprocess
import textwrap

import pytest
from pxr import Usd

from urdf2usd.world_link import (
    apply_world_link_rule,
    WorldLinkError,
    find_synthetic_root_joint,
    remove_synthetic_root_joint,
    urdf_has_world_link,
)

LINK = '<link name="{n}"><inertial><mass value="1"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="1 1 1"/></geometry></collision></link>'


def _urdf(tmp_path, body, name="r.urdf"):
    p = tmp_path / name
    p.write_text(f'<robot name="r">{body}</robot>')
    return p


MOBILE = LINK.format(n="base_link") + LINK.format(n="wheel") + (
    '<joint name="w" type="continuous"><parent link="base_link"/>'
    '<child link="wheel"/><axis xyz="0 1 0"/></joint>'
)


def test_no_world_link(tmp_path):
    assert urdf_has_world_link(_urdf(tmp_path, MOBILE)) is False


def test_world_link_empty(tmp_path):
    body = '<link name="world"/>' + MOBILE
    assert urdf_has_world_link(_urdf(tmp_path, body)) is True


@pytest.mark.parametrize("child", [
    '<inertial><mass value="1"/></inertial>',
    '<visual><geometry><box size="1 1 1"/></geometry></visual>',
    '<collision><geometry><box size="1 1 1"/></geometry></collision>',
])
def test_world_link_with_content_rejected(tmp_path, child):
    p = _urdf(tmp_path, f'<link name="world">{child}</link>' + MOBILE)
    with pytest.raises(WorldLinkError):
        urdf_has_world_link(p)


def test_name_match_is_exact(tmp_path):
    # 'World' / 'world_link' are not the world link
    p = _urdf(tmp_path, LINK.format(n="World") + LINK.format(n="world_link"))
    assert urdf_has_world_link(p) is False


def test_malformed_urdf_rejected(tmp_path):
    p = tmp_path / "bad.urdf"
    p.write_text("<robot")
    with pytest.raises(WorldLinkError):
        urdf_has_world_link(p)


def _make_usd(tmp_path, root_joint_body0="</r>"):
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
                    rel physics:body0 = {root_joint_body0}
                    rel physics:body1 = </r/base_link>
                }}
                def PhysicsRevoluteJoint "w"
                {{
                    rel physics:body0 = </r/base_link>
                    rel physics:body1 = </r/wheel>
                }}
            }}
        }}
        """))
    return p


def test_find_synthetic_root_joint(tmp_path):
    stage = Usd.Stage.Open(str(_make_usd(tmp_path)))
    assert find_synthetic_root_joint(stage).GetPath() == "/r/Physics/root_joint"


def test_real_joint_named_root_joint_is_not_synthetic(tmp_path):
    # body0 is a link, so this is a URDF joint, not the synthetic one
    stage = Usd.Stage.Open(str(_make_usd(tmp_path, "</r/base_link>")))
    assert find_synthetic_root_joint(stage) is None
    assert remove_synthetic_root_joint(stage) is False
    assert stage.GetPrimAtPath("/r/Physics/root_joint")


def test_remove_synthetic_root_joint(tmp_path):
    stage = Usd.Stage.Open(str(_make_usd(tmp_path)))
    assert remove_synthetic_root_joint(stage) is True
    assert not stage.GetPrimAtPath("/r/Physics/root_joint")
    assert stage.GetPrimAtPath("/r/Physics/w")  # real joints kept
    assert remove_synthetic_root_joint(stage) is False  # idempotent


def test_apply_removes_root_joint_when_no_world(tmp_path):
    urdf, usd = _urdf(tmp_path, MOBILE), _make_usd(tmp_path)
    assert apply_world_link_rule(urdf, usd) is True
    assert not Usd.Stage.Open(str(usd)).GetPrimAtPath("/r/Physics/root_joint")


def test_apply_keeps_joint_when_world_link_present(tmp_path):
    urdf = _urdf(tmp_path, '<link name="world"/>' + MOBILE)
    usd = _make_usd(tmp_path)
    assert apply_world_link_rule(urdf, usd) is False
    assert Usd.Stage.Open(str(usd)).GetPrimAtPath("/r/Physics/root_joint")


def test_apply_is_idempotent(tmp_path):
    urdf, usd = _urdf(tmp_path, MOBILE), _make_usd(tmp_path)
    assert apply_world_link_rule(urdf, usd) is True
    assert apply_world_link_rule(urdf, usd) is False


def test_apply_rejects_non_empty_world_link(tmp_path):
    urdf = _urdf(tmp_path, '<link name="world"><visual><geometry>'
                 '<box size="1 1 1"/></geometry></visual></link>' + MOBILE)
    with pytest.raises(WorldLinkError):
        apply_world_link_rule(urdf, _make_usd(tmp_path))


@pytest.mark.parametrize("world", [False, True])
def test_with_real_converter(tmp_path, world):
    urdf = _urdf(tmp_path, ('<link name="world"/><joint name="wj" type="fixed">'
                            '<parent link="world"/><child link="base_link"/></joint>'
                            if world else "") + MOBILE)
    out = tmp_path / "out"
    subprocess.run(["urdf_usd_converter", str(urdf), str(out)], check=True,
                   capture_output=True, text=True)
    usd = out / "r.usda"
    assert apply_world_link_rule(urdf, usd) is (not world)
    assert find_synthetic_root_joint(Usd.Stage.Open(str(usd))) is None
    # layered output: the Physics sublayer file was actually rewritten
    assert ("root_joint" in (out / "Payload" / "Physics.usda").read_text()) is False
