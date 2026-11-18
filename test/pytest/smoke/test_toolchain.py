"""Toolchain smoke test: USD bindings and the official converter work."""

import subprocess

from pxr import Usd

URDF = """<robot name="smoke">
  <link name="base_link">
    <inertial><mass value="1"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial>
    <collision><geometry><box size="1 1 1"/></geometry></collision>
  </link>
</robot>"""


def test_pxr_can_create_a_stage():
    stage = Usd.Stage.CreateInMemory()
    assert stage.DefinePrim("/a", "Xform")


def test_converter_produces_usd(tmp_path):
    urdf = tmp_path / "smoke.urdf"
    urdf.write_text(URDF)
    out = tmp_path / "out"
    subprocess.run(["urdf_usd_converter", str(urdf), str(out)],
                   check=True, capture_output=True, text=True)
    stage = Usd.Stage.Open(str(out / "smoke.usda"))
    assert stage.GetDefaultPrim().GetName() == "smoke"
