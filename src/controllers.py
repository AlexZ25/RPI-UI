import numpy as np
import math


class StanleyController:
    def __init__(self, k_e = -3.0, k_v = 1.2, wheelbase = 0.13, ff_gain=-2, 
                 max_steer_rad=math.radians(30)):
        self.k_e = k_e
        self.k_v = k_v
        self.wheelbase = wheelbase
        self.ff_gain = ff_gain
        self.max_steer_rad = max_steer_rad
        self.control_sppd_mps = 0.0

    def compute_steering(self, estimate):
        heading_error_rad = math.radians(estimate.yaw_deg)
        cross_track_m = estimate.offset_m
        curvature_m = estimate.curvature

        stanley_term = math.atan(
            (self.k_e * cross_track_m) / (self.k_v + self.control_sppd_mps)
        )

        delta_ff = math.atan(self.wheelbase * curvature_m) * self.ff_gain
        steer_rad = heading_error_rad*0.9 + stanley_term + delta_ff
        steer_norm = max(-1.0, min(1.0, steer_rad / self.max_steer_rad))

        return steer_norm