"""Connect Part B's sensing functions to the MuJoCo scene used by the pipeline."""

from pathlib import Path

import numpy as np

from part_b_sensing import sensing


class Sensors:
    def __init__(self, camera_position, marker_size, calibration=None,
                 lidar_noise_scale=1.0, encoder_noise_scale=1.0):
        # Part B's constants must describe this scene, or every range would be scaled wrong.
        if not np.isclose(marker_size, sensing.MARKER_SIZE):
            raise ValueError(f"Scene marker is {marker_size:.3f} m but sensing.MARKER_SIZE is {sensing.MARKER_SIZE} m.")
        if not np.allclose(camera_position, sensing.CAM_POS_IN_BASE):
            raise ValueError(f"Scene camera is at {camera_position} but sensing.CAM_POS_IN_BASE is "
                             f"{sensing.CAM_POS_IN_BASE}.")
        self.K = sensing.camera_matrix()
        self.distortion = np.zeros(5)
        self.calibrated = calibration is not None
        self.lidar_noise_scale = lidar_noise_scale
        self.encoder_noise_scale = encoder_noise_scale
        self.invalid_camera = 0
        for reason in sensing.REJECTS:
            sensing.REJECTS[reason] = 0
        if calibration is not None:
            with np.load(Path(calibration)) as saved:
                self.K = saved["K"].copy()
                self.distortion = saved["dist"].copy()

    def encoders(self, angles):
        return np.array(sensing.noisy_encoders(*angles, noise_scale=self.encoder_noise_scale))

    def odometry(self, now, previous, dt):
        return sensing.odometry(now, previous, dt)

    def lidar(self, ranges):
        return sensing.noisy_lidar(ranges, noise_scale=self.lidar_noise_scale)

    def marker(self, image):
        seen, translation, _ = sensing.detect_marker(image, self.K, self.distortion)
        if not seen:
            return None
        if not np.all(np.isfinite(translation)):
            self.invalid_camera += 1
            return None
        measurement = np.array(sensing.range_bearing(sensing.cam_to_base(translation)))
        if not np.all(np.isfinite(measurement)) or measurement[0] <= 0:
            self.invalid_camera += 1
            return None
        return measurement
