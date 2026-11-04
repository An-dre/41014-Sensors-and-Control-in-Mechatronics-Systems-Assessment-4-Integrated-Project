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

## Generative AI Statement

| Tool | Used for |
|---|---|
| **Claude (Anthropic)** | Explaining concepts, debugging code, diagnosing simulator errors, and formatting the GitHub project pages |
| **OpenAI Codex** | Tests and documentation, and integrating the parts into the shared pipeline |

All AI suggestions were reviewed by the team before use. We ran and tested the code ourselves, checked the results against the simulator, and made our own decisions on parameters and design choices. Every team member can explain the code and results for their part. All experiments were run by the team, and all reported numbers come from our own runs.

