# UAV Trajectory Tracking

Controller comparison study for multi-UAV precision trajectory tracking under real-world fault conditions, built on PyBullet-Drones.

The central question is whether a lightweight supervisory layer can meaningfully improve tracking recovery over a standard PID controller when faults occur — without replacing PID or adding a fully learned black-box controller. Three controllers are compared across identical fault scenarios and randomized seeds so the results are directly attributable to the controller design, not to lucky conditions.

![Python](https://img.shields.io/badge/Python-3776AB?style=flat&logo=python&logoColor=white)
![PyBullet](https://img.shields.io/badge/PyBullet-306998?style=flat&logoColor=white)

## Controllers

**Open-loop** - baseline with no feedback correction, used to establish a performance floor.

**PID** - standard closed-loop controller. Tracks position error and applies corrections, but has no awareness of whether an error comes from a real fault or normal tracking lag. Under fault conditions, it keeps pushing harder regardless.

**Agentic supervisor (PID + supervisory layer)** - the main contribution. A slow, selective supervisor runs at 5Hz on top of PID and operates in four priority modes:

- Mode 3 - connectivity rescue: compresses formation scale and increases trajectory smoothing when inter-agent connectivity drops below threshold
- Mode 2 - anti-windup: freezes the PID integrator when actuator saturation is detected, preventing integrator windup from amplifying fault response
- Mode 1 - drift correction: applies bounded reference shifts to re-anchor the swarm center of mass when persistent drift is detected and sensors are trusted
- Mode 0 - off: supervisor is inactive, PID runs normally

The supervisor distinguishes between sensor-corrupted and trusted agents, freezes reference shifts during severe communication loss, and uses persistence checks before triggering any mode change to avoid false positives from momentary signal noise.

## Fault Injection

Three fault types injected in controlled, time-windowed scenarios:

- Wind disturbance - persistent external force applied to individual agents
- Sensor corruption - position and velocity observations corrupted for a subset of agents
- Communication degradation - inter-agent connectivity rate reduced, dropping below thresholds that trigger connectivity rescue mode

Each fault type is evaluated independently across 20 randomized seeds with per-seed CSV telemetry output and plots for direct cross-controller comparison.

## Evaluation

- 20 randomized seeds per scenario
- Per-seed CSV outputs for trajectory error, supervisor mode activations, and recovery metrics
- Batch plot generation for cross-controller performance comparison
- Separate quick metrics script for summary statistics across seeds

## Structure

```
src/
  controllers/        open-loop, PID, agentic supervisor, PID-agentic hybrid
  faults/             fault injection logic
  disturbances.py     wind and external disturbance models
  trajectories.py     trajectory definitions
  metrics.py          tracking error and recovery metrics
  eval/               per-seed evaluation runner
configs/              scenario configuration files
outputs/              CSV telemetry and figures
```

