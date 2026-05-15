# UAV Precision Trajectory Tracking

Controller comparison study for single and multi-UAV trajectory tracking under fault conditions, built on PyBullet-Drones.

The experiment compares three controllers — open-loop, PID, and an agentic controller — across 20 randomized seeds with fault injection across wind disturbance, sensor corruption, and communication degradation. The goal is to isolate how each controller behaves when tracking error comes from a real fault rather than normal lag, and to quantify the performance gap between them with repeatable, measurable results.

![Python](https://img.shields.io/badge/Python-3776AB?style=flat&logo=python&logoColor=white)
![PyBullet](https://img.shields.io/badge/PyBullet-306998?style=flat&logoColor=white)
