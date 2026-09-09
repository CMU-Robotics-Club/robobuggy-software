# Committed simulator scenarios (DECISIONS.md D12)

Each file is a complete parameter file for one launch. Seeds are fixed so the
measurement noise and the fake sensors repeat. Run one with the scenario runner
inside the container (from `/rb_ws`):

```bash
S=src/buggy/config/scenarios
R=src/buggy/scripts/debug/run_sim_scenario.sh
bash $R --launch sim_2d_single.xml --config $S/single_reference.yaml --metrics sim_metrics.py --duration 90 --out /tmp/single_reference.json -- use_frenet:=true
bash $R --launch sim_2d_double.xml --config $S/double_pass.yaml     --metrics pass_metrics.py --duration 100 --out /tmp/double_pass.json
bash $R --launch sim_2d_double.xml --config $S/traffic.yaml         --metrics pass_metrics.py --duration 100 --out /tmp/traffic.json -- use_perception_sim:=true
bash $R --launch sim_2d_double.xml --config $S/right_corridor.yaml  --metrics pass_metrics.py --duration 100 --out /tmp/right_corridor.json -- use_perception_sim:=true
bash $R --launch sim_2d_double.xml --config $S/blocked.yaml         --metrics pass_metrics.py --duration 60  --out /tmp/blocked.json -- use_perception_sim:=true
```

| scenario | what it exercises | pass criterion (metrics exit 0) |
| --- | --- | --- |
| `single_reference.yaml` | envelope mode on the reference line, no opponents | no collision, envelope used > 95 % of the run |
| `double_pass.yaml` | SC 25 m behind NAND, 13 vs 10 m/s, NAND known only through the raw radio | pass completed, no collision, envelope used |
| `traffic.yaml` | NAND plus two ghost buggies seen by fake lidar and camera | no collision with NAND or ghosts |
| `right_corridor.yaml` | wide assumed right corridor, opponent left of the line: the pass must go right | pass on the right, no collision |
| `blocked.yaml` | two stationary ghosts side by side: no eligible plan exists | no collision; the controller reports `reference_fallback`; no ineligible plan ever steers |

The metrics scripts exit non-zero on a collision, on missing input, or when an
envelope-mode controller was found steering on a plan that was not control
eligible. They are regression gates, not proof of race performance: the
simulator is a kinematic bicycle with no tyre model.
