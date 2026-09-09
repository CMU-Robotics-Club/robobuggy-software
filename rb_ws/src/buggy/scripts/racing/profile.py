"""Vehicle profile: geometric and actuator limits with explicit provenance (D1, D9).

The profile file (config/vehicle_sc.yaml) is the only place limits come from. Each
field has a status; a hardware profile refuses to start when a needed field is
still `unverified`, and simulation results never change a field's status.
"""

import math
import os
from dataclasses import dataclass

import yaml

STATUSES = ("verified_source", "measured", "assumed", "policy", "unverified")
KNOWN = ("verified_source", "measured", "assumed", "policy")


class ProfileError(ValueError):
    """The profile is malformed or a required value is unknown."""


@dataclass(frozen=True)
class ProfileField:
    name: str
    value: float
    status: str
    note: str = ""

    @property
    def known(self):
        return self.value is not None and self.status in KNOWN


def default_profile_path():
    """config/vehicle_sc.yaml under $RBROOT, else relative to this source tree."""
    root = os.environ.get("RBROOT")
    if root:
        candidate = os.path.join(root, "src", "buggy", "config", "vehicle_sc.yaml")
        if os.path.exists(candidate):
            return candidate
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, "..", "..", "config", "vehicle_sc.yaml"))


class VehicleProfile:
    def __init__(self, fields, command_coordinate="", name="", path=""):
        self.fields = dict(fields)
        self.command_coordinate = command_coordinate
        self.name = name
        self.path = path

    @classmethod
    def load(cls, path=None):
        path = path or default_profile_path()
        with open(path, "r", encoding="utf-8") as handle:
            raw = yaml.safe_load(handle) or {}
        fields = {}
        for key, spec in (raw.get("fields") or {}).items():
            if not isinstance(spec, dict) or "status" not in spec:
                raise ProfileError(f"{path}: field {key} needs value and status")
            status = str(spec["status"])
            if status not in STATUSES:
                raise ProfileError(f"{path}: field {key} has unknown status {status}")
            value = spec.get("value")
            if status == "unverified" and value is not None:
                raise ProfileError(f"{path}: field {key} is unverified but carries a value")
            if status != "unverified":
                if value is None or not math.isfinite(float(value)):
                    raise ProfileError(f"{path}: field {key} ({status}) needs a finite value")
                value = float(value)
            fields[key] = ProfileField(key, value, status, str(spec.get("note", "")))
        profile = cls(fields, str(raw.get("command_coordinate", "")), str(raw.get("vehicle", "")), path)
        for required in ("wheelbase_m", "width_m", "length_m", "software_steering_clip_deg",
                         "planning_curvature_cap_1_per_m"):
            profile.value(required, required=True)
        return profile

    def field(self, name):
        try:
            return self.fields[name]
        except KeyError as exc:
            raise ProfileError(f"{self.path}: missing field {name}") from exc

    def value(self, name, required=False):
        """The field's value, or None when unverified (raises if required)."""
        item = self.field(name)
        if not item.known:
            if required:
                raise ProfileError(f"{self.path}: {name} is {item.status}; measure it before use")
            return None
        return item.value

    def require_measured(self, names):
        """Hardware profiles: every listed field must be measured or verified from source."""
        bad = [n for n in names if self.field(n).status not in ("measured", "verified_source")]
        if bad:
            raise ProfileError(f"{self.path}: not measured on hardware: {', '.join(bad)}")

    # ------------------------------------------------------------------ derived limits
    def wheelbase(self):
        return self.value("wheelbase_m", required=True)

    def half_width(self):
        return 0.5 * self.value("width_m", required=True)

    def half_length(self):
        return 0.5 * self.value("length_m", required=True)

    def steering_limit_deg(self):
        """Smallest known steering angle limit and where it came from."""
        candidates = [(self.value("software_steering_clip_deg", required=True), "software_steering_clip_deg")]
        for name in ("physical_wheel_angle_deg", "actuator_command_limit_deg"):
            v = self.value(name)
            if v is not None:
                candidates.append((v, name))
        return min(candidates)

    def curvature_ceiling(self):
        """Geometric ceiling from the steering limit: tan(delta_max) / wheelbase."""
        limit_deg, _ = self.steering_limit_deg()
        return math.tan(math.radians(limit_deg)) / self.wheelbase()

    def planning_curvature_cap(self):
        """The cap planners must use: the policy value, never above the ceiling."""
        cap = self.value("planning_curvature_cap_1_per_m", required=True)
        return min(cap, self.curvature_ceiling())

    def command_limit_deg(self):
        """Largest composed steering command allowed after offset correction."""
        return self.steering_limit_deg()

    def slew_limit_dps(self):
        return self.value("actuator_slew_dps")

    def max_offset_correction_deg(self):
        return self.value("max_offset_correction_deg")

    def provenance(self):
        """{field: status} for logging at start-up."""
        return {name: item.status for name, item in self.fields.items()}
