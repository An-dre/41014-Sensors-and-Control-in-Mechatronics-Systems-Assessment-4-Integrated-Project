"""Connect Dinel's unchanged sensing functions to the current MuJoCo scene."""

from pathlib import Path

import numpy as np

import sensing


class Sensors:
    def __init__(self, camera_position, marker_size, calibration=None):
        self.camera_position = np.asarray(camera_position, dtype=float)
        self.marker_scale = marker_size / sensing.MARKER_SIZE
        self.K = sensing.camera_matrix()
        self.distortion = np.zeros(5)
        self.calibrated = calibration is not None
        self.invalid_camera = 0
        for reason in sensing.REJECTS:
            sensing.REJECTS[reason] = 0
        if calibration is not None:
            with np.load(Path(calibration)) as saved:
                self.K = saved["K"].copy()
                self.distortion = saved["dist"].copy()

    def encoders(self, angles):
        return np.array(sensing.noisy_encoders(*angles))

    def odometry(self, now, previous, dt):
        return sensing.odometry(now, previous, dt)

    def lidar(self, ranges):
        return sensing.noisy_lidar(ranges)

    def marker(self, image):
        seen, translation, _ = sensing.detect_marker(image, self.K, self.distortion)
        if not seen:
            return None
        if not np.all(np.isfinite(translation)):
            self.invalid_camera += 1
            return None
        # PnP translation scales linearly with the supplied physical marker size.
        # These two adjustments are scene settings, not edits to B's algorithms.
        point = sensing.cam_to_base(translation * self.marker_scale)
        point += self.camera_position - sensing.CAM_POS_IN_BASE
        measurement = np.array(sensing.range_bearing(point))
        if not np.all(np.isfinite(measurement)) or measurement[0] <= 0:
            self.invalid_camera += 1
            return None
        return measurement
