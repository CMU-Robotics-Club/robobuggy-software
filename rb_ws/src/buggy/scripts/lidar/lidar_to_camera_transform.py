import numpy as np

from util.constants import Constants

# Simple demo to show transforming points in lidar frame to camera frame

def transform_point(pt: np.ndarray) -> np.ndarray:
    pt_h = np.append(pt, 1.0)
    pt_cam_h = Constants.T_LIDAR_TO_CAMERA @ pt_h
    return pt_cam_h[:3]

