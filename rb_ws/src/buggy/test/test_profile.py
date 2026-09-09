"""The vehicle profile is the only source of limits and never loosens them."""

import math
from pathlib import Path

import pytest
import yaml

from racing.profile import ProfileError, VehicleProfile, default_profile_path

CONFIG = Path(__file__).resolve().parents[1] / "config" / "vehicle_sc.yaml"


def test_default_profile_loads_and_reports_provenance():
    profile = VehicleProfile.load(str(CONFIG))
    provenance = profile.provenance()
    assert provenance["wheelbase_m"] == "verified_source"
    assert provenance["software_steering_clip_deg"] == "verified_source"
    assert provenance["physical_wheel_angle_deg"] == "unverified"
    assert profile.value("physical_wheel_angle_deg") is None
    assert default_profile_path().endswith("vehicle_sc.yaml")


def test_ceiling_follows_the_software_clip_and_cap_is_stricter():
    profile = VehicleProfile.load(str(CONFIG))
    ceiling = math.tan(math.radians(20.0)) / 1.104
    assert profile.curvature_ceiling() == pytest.approx(ceiling)
    assert profile.planning_curvature_cap() == pytest.approx(0.25)
    assert profile.planning_curvature_cap() <= profile.curvature_ceiling()
    assert profile.command_limit_deg() == (20.0, "software_steering_clip_deg")


def test_policy_cap_can_never_exceed_the_ceiling(tmp_path):
    raw = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    raw["fields"]["planning_curvature_cap_1_per_m"]["value"] = 5.0
    path = tmp_path / "loose.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    profile = VehicleProfile.load(str(path))
    assert profile.planning_curvature_cap() == pytest.approx(profile.curvature_ceiling())


def test_known_physical_limit_tightens_the_ceiling(tmp_path):
    raw = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    raw["fields"]["physical_wheel_angle_deg"] = {"value": 15.0, "status": "measured", "note": "test"}
    path = tmp_path / "measured.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    profile = VehicleProfile.load(str(path))
    assert profile.command_limit_deg() == (15.0, "physical_wheel_angle_deg")
    assert profile.curvature_ceiling() == pytest.approx(math.tan(math.radians(15.0)) / 1.104)


def test_hardware_profile_refuses_unverified_fields():
    profile = VehicleProfile.load(str(CONFIG))
    with pytest.raises(ProfileError):
        profile.require_measured(["wheelbase_m", "actuator_slew_dps"])
    with pytest.raises(ProfileError):
        profile.value("actuator_slew_dps", required=True)


@pytest.mark.parametrize("mutation", [
    lambda f: f["wheelbase_m"].__setitem__("status", "guessed"),
    lambda f: f["wheelbase_m"].__setitem__("value", None),
    lambda f: f["actuator_slew_dps"].__setitem__("value", 30.0),
    lambda f: f.pop("width_m"),
])
def test_malformed_profiles_are_rejected(tmp_path, mutation):
    raw = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    mutation(raw["fields"])
    path = tmp_path / "bad.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    with pytest.raises(ProfileError):
        VehicleProfile.load(str(path))
