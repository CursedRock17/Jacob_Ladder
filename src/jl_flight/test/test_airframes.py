"""Airframes: lookup, registration, and the simulated camera matching its model."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from jl_flight.airframes import AIRFRAMES, X500_RGBD, Airframe, get_airframe, register
from jl_flight.perception import CameraMount

REPO = Path(__file__).resolve().parents[3]


def test_lookup_by_name_and_environment(monkeypatch: pytest.MonkeyPatch) -> None:
    assert get_airframe("x500_rgbd") is X500_RGBD
    monkeypatch.setenv("JL_AIRFRAME", "x500_rgbd")
    assert get_airframe() is X500_RGBD
    monkeypatch.delenv("JL_AIRFRAME")
    assert get_airframe().name == "generic"


def test_unknown_names_list_the_choices() -> None:
    with pytest.raises(
        KeyError, match="known: generic, realsense_drone, x500_dual_cam, x500_rgbd"
    ):
        get_airframe("nope")


def test_register_refuses_duplicates() -> None:
    with pytest.raises(ValueError, match="already exists"):
        register(Airframe("generic"))
    mine = register(Airframe("test_frame_xyz"))
    assert get_airframe("test_frame_xyz") is mine
    del AIRFRAMES["test_frame_xyz"]


def test_x500_rgbd_mount_matches_the_gazebo_model() -> None:
    sdf = ET.parse(REPO / "gazebo/models/x500_rgbd/model.sdf").getroot()
    link = sdf.find(".//link[@name='rgbd_camera_link']")
    assert link is not None
    lx, ly, lz = (float(v) for v in link.findtext("pose", "").split()[:3])
    sx, sy, sz = (float(v) for v in link.findtext("sensor/pose", "").split()[:3])
    mount = X500_RGBD.camera("front").mount
    # Gazebo's FLU (x forward, y left, z up) -> body FRD
    assert mount.forward == pytest.approx(lx + sx)
    assert mount.right == pytest.approx(-(ly + sy))
    assert mount.down == pytest.approx(-(lz + sz))


def test_x500_dual_cam_cameras_sit_at_the_origin_as_the_model_places_them() -> None:
    # Its cameras' offsets are on their fixed joints, which do not move the links.
    sdf = ET.parse(REPO / "gazebo/models/x500_dual_cam/model.sdf").getroot()
    for link in ("front_camera_link", "down_camera_link"):
        assert sdf.find(f".//link[@name='{link}']/pose") is None
    cameras = get_airframe("x500_dual_cam").cameras
    assert cameras["front"].mount == CameraMount()
    assert cameras["down"].mount == CameraMount(facing="down")


def test_the_real_drone_reads_the_camera_vio_publishes() -> None:
    # oak_d_visual_odometry republishes the D435i's colour stream for everyone else.
    params = (
        REPO / "src/oak_d_visual_odometry/config/realsense_params.yaml"
    ).read_text()
    front = get_airframe("realsense_drone").camera("front")
    assert f"rgb_topic: {front.image_topic}" in params
    assert f"rgb_camera_info_topic: {front.info_topic}" in params


def test_the_real_drone_uses_the_lever_arm_vio_was_measured_with() -> None:
    params = (REPO / "src/oak_d_visual_odometry/config/cuvslam_params.yaml").read_text()
    mount = get_airframe("realsense_drone").camera("front").mount
    assert f"t_body_cam: [{mount.forward}, {mount.right}, {mount.down}]" in params
