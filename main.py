"""Run the whole Track A system: camera and LiDAR -> Part B sensing -> Part C EKF -> LiDAR A* planner -> Part D LQR.

From the repository root:
    python main.py              one docking run from the west start (about 4 minutes)
    python main.py --viewer     the same run in the MuJoCo viewer
    python main.py --help       every option: other starts, repeated trials, disturbances, noise settings

The integration code is in part_e_integration/pipeline.py. Results go to
part_e_integration/results_pipeline/ unless --output names another folder.
"""

from part_e_integration.pipeline import main

if __name__ == "__main__":
    main()
