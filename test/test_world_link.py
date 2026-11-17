import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from urdf2usd.world_link import WorldLinkError, urdf_has_world_link  # noqa: E402

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
