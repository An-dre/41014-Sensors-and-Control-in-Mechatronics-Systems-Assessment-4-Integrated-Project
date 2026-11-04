# 41014 Sensors and Control in Mechatronics Systems – Assessment 4: Integrated Project

This is a group project. This team integrates sensing, estimation and control on MujoCo.

**Chosen track:** Track A — TurtleBot3 visual waypoint navigation and ArUco docking

## Project 100

| Member | Part |
|---|---|
| Gabriel | Part A – Modelling |
| Dinel | Part B – Sensing |
| Abdul | Part C – Estimation |
| Andre | Part D – Control |

## Integration

- See [PIPELINE.md](PIPELINE.md) for setup, commands, the code walkthrough and limitations.


## Quick start

From this folder (Python 3.10):

```powershell
python -m pip install -r requirements.txt
python tb3_project/run_pipeline.py
python tb3_project/run_pipeline.py --viewer
```

The default run saves results in `tb3_project/results_pipeline/` and uses the camera's nominal intrinsics until a calibration file is supplied. Ground truth is used for evaluation only.
