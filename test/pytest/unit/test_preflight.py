import pytest

from urdf2usd.preflight import PreflightError, check_urdf

OBJ = "o t\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"
STL = "solid t\nendsolid t\n"


def _urdf(tmp_path, *filenames, name="r.urdf"):
    links = "".join(
        f'<link name="l{i}"><visual><geometry><mesh filename="{f}"/></geometry>'
        f"</visual></link>"
        for i, f in enumerate(filenames)
    )
    p = tmp_path / name
    p.write_text(f'<robot name="r">{links}</robot>')
    return p


def _problems(urdf, **kw):
    with pytest.raises(PreflightError) as e:
        check_urdf(urdf, **kw)
    return e.value.problems


def test_is_value_error():
    assert issubclass(PreflightError, ValueError)


def test_ok_relative_absolute_file_and_package(tmp_path):
    (tmp_path / "a.stl").write_text(STL)
    (tmp_path / "b.OBJ").write_text(OBJ)
    pkg = tmp_path / "pkg_root"
    pkg.mkdir()
    (pkg / "c.dae").write_text("<COLLADA/>")
    urdf = _urdf(
        tmp_path,
        "a.stl",
        str(tmp_path / "b.OBJ"),
        f"file://{tmp_path}/a.stl",
        "package://mypkg/c.dae",
    )
    check_urdf(urdf, packages={"mypkg": pkg})


def test_no_meshes_ok(tmp_path):
    p = tmp_path / "r.urdf"
    p.write_text('<robot name="r"><link name="a"/></robot>')
    check_urdf(p)


def test_missing_mesh(tmp_path):
    probs = _problems(_urdf(tmp_path, "nope.stl"))
    assert len(probs) == 1 and "nope.stl" in probs[0]


def test_unsupported_extension(tmp_path):
    (tmp_path / "a.fbx").write_text("x")
    probs = _problems(_urdf(tmp_path, "a.fbx"))
    assert len(probs) == 1 and "fbx" in probs[0]


def test_unsupported_scheme(tmp_path):
    probs = _problems(_urdf(tmp_path, "http://example.com/a.stl"))
    assert len(probs) == 1 and "http" in probs[0]


def test_unknown_package(tmp_path):
    probs = _problems(_urdf(tmp_path, "package://ghost/a.stl"))
    assert len(probs) == 1 and "ghost" in probs[0]


def test_package_given_but_file_missing(tmp_path):
    probs = _problems(_urdf(tmp_path, "package://p/a.stl"), packages={"p": tmp_path})
    assert len(probs) == 1 and "a.stl" in probs[0]


def test_obj_missing_mtl(tmp_path):
    (tmp_path / "a.obj").write_text("mtllib a.mtl\n" + OBJ)
    probs = _problems(_urdf(tmp_path, "a.obj"))
    assert len(probs) == 1 and "a.mtl" in probs[0]


def test_obj_with_mtl_ok(tmp_path):
    (tmp_path / "a.obj").write_text("mtllib a.mtl\n" + OBJ)
    (tmp_path / "a.mtl").write_text("newmtl m\n")
    check_urdf(_urdf(tmp_path, "a.obj"))


def test_lists_all_problems(tmp_path):
    (tmp_path / "a.fbx").write_text("x")
    (tmp_path / "b.obj").write_text("mtllib b.mtl\n" + OBJ)
    urdf = _urdf(
        tmp_path, "gone.stl", "a.fbx", "ftp://h/a.stl", "package://x/a.stl", "b.obj"
    )
    with pytest.raises(PreflightError) as e:
        check_urdf(urdf)
    assert len(e.value.problems) == 5
    for s in ("gone.stl", "fbx", "ftp", "x", "b.mtl"):
        assert s in str(e.value)


def test_collision_meshes_checked(tmp_path):
    p = tmp_path / "r.urdf"
    p.write_text(
        '<robot name="r"><link name="a"><collision><geometry>'
        '<mesh filename="c.stl"/></geometry></collision></link></robot>'
    )
    probs = _problems(p)
    assert "c.stl" in probs[0]


def test_unparsable_urdf(tmp_path):
    p = tmp_path / "bad.urdf"
    p.write_text("<robot")
    assert _problems(p)
