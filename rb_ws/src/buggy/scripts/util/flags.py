"""Feature flags reader for use within ROS nodes.

Feature flags are pushed into every node as the parameter ff_<name>,
flags["vision"] is a pass-through read of the parameter ff_vision for example.

Example usage from inside a node:
>>> flags = Flags(self)
>>> vision_enabled = flags["vision"]
"""

import os

import yaml
from rcl_interfaces.msg import ParameterDescriptor


class Flags:
    LEDGER = os.path.join(os.environ.get("RBROOT", "/rb_ws"), "flags.yml")

    def __init__(self, node):
        self._node = node
        self._retired_flags = []

        with open(self.LEDGER) as f:
            ledger = yaml.safe_load(f)

        for name, meta in ledger.items():
            node.declare_parameter(
                "ff_" + name,
                bool(meta["default"]),
                ParameterDescriptor(description=meta["description"], read_only=True),
            )

            if meta["state"] == "stable":
                self._retired_flags.append(name)
            
        self._names = list(ledger)

    def __getitem__(self, name):
        if name not in self._names:
            raise KeyError("unknown feature flag: " + name)

        if name in self._retired_flags:
            self._node.get_logger().warn(f"Stable feature flag {name} accessed")
        
        return bool(self._node.get_parameter("ff_" + name).value)

    def as_dict(self):
        return {name: self[name] for name in self._names}