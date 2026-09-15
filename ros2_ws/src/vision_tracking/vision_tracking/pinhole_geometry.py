#!/usr/bin/env python3
"""
3D Pinhole Distance Estimation & Turn-Point Geometry Module.
Ported and restored from reference commit cb6b1ae (ArduPilot SITL / vehicle_yaw_search.py)
for the PX4 + ROS 2 Offboard architecture.

Separates geometric perception / camera ray projection from control laws and autopilot transport.
"""

import math
from dataclasses import dataclass, asdict
from typing import Optional


@dataclass
class PinholeCameraConfig:
    """Camera intrinsic and extrinsic calibration parameters."""
    camera_pitch_rad: float = 0.65        # Downward pitch of camera relative to body (rad, ~37.24 deg)
    fx: float = 133.55                    # Horizontal focal length in 416x416 reference frame
    fy: float = 178.07                    # Vertical focal length in 416x416 reference frame
    cx: float = 208.0                     # Principal point X in 416x416 reference frame
    cy: float = 208.0                     # Principal point Y in 416x416 reference frame
    person_height_m: float = 0.90         # Estimated center of torso height above ground (m)
    min_relative_height_m: float = 1.50   # Minimum vertical standoff for projection stability (m)
    target_distance_m: float = 4.50       # Nominal ground follow distance (m)
    tree_clearance_margin_m: float = 1.80 # Tree clearance buffer for turn-point navigation (m)


@dataclass
class TargetGeometry:
    """Target tracking geometry output decoupled from identity."""
    target_id: int = -1
    target_valid: bool = False
    pixel_error_x: float = 0.0
    pixel_error_y: float = 0.0
    distance_dx: float = 0.0              # Forward ground distance (m)
    distance_dy: float = 0.0              # Lateral ground distance (m)
    ground_distance: float = 0.0          # Euclidean ground distance (m)
    target_dx: float = 0.0                # Target advance distance including tree clearance (m)
    target_dy: float = 0.0                # Target advance lateral distance (m)
    target_dist: float = 0.0              # Total turn-point travel distance (m)
    distance_confidence: float = 0.0      # Metric confidence [0.0, 1.0]
    distance_valid: bool = False          # True if geometry projection is reliable

    def to_dict(self):
        return asdict(self)


def estimate_pinhole_geometry(
    error_x: float,
    error_y: float,
    current_altitude: float,
    vehicle_pitch: float = 0.0,
    config: Optional[PinholeCameraConfig] = None,
    target_id: int = -1
) -> TargetGeometry:
    """
    Compute real-world ground distance (dx, dy) and turn-point trajectory vector (target_dx, target_dy)
    using 3D Pinhole camera geometry with vehicle altitude and pitch angle compensation.

    Restores exact mathematical formulation from cb6b1ae:vehicle_yaw_search.py:
        h_rel = max(1.5, current_alt - 0.90)
        alpha_y = atan2(error_y, fy)
        alpha_x = atan2(error_x, fx)
        theta_dep = max(0.12, min(1.48, camera_pitch_rad - vehicle_pitch + alpha_y))
        dx = h_rel / tan(theta_dep)
        dy = dx * tan(alpha_x)
        target_dx = dx + tree_clearance_margin
        total_dist = sqrt(target_dx^2 + dy^2)
    """
    if config is None:
        config = PinholeCameraConfig()

    # Relative height of camera above person center of mass
    h_rel = max(config.min_relative_height_m, current_altitude - config.person_height_m)

    # Angular ray deviation from camera optical axis (using pinhole focal lengths)
    alpha_y = math.atan2(float(error_y), config.fy)
    alpha_x = math.atan2(float(error_x), config.fx)

    # Depression angle relative to ground plane
    # Camera is tilted down by camera_pitch_rad. Vehicle nose-up pitch (>0) reduces depression angle.
    theta_dep = config.camera_pitch_rad - float(vehicle_pitch) + alpha_y
    # Bound depression angle to avoid singularity near horizontal (tan->0) or vertical (tan->inf)
    theta_dep_clamped = max(0.12, min(1.48, theta_dep))

    # Ground distances
    dx = h_rel / math.tan(theta_dep_clamped)
    dy = dx * math.tan(alpha_x)
    ground_dist = math.hypot(dx, dy)

    # Target turn-point vector with tree clearance margin (+1.8m)
    target_dx = dx + config.tree_clearance_margin_m
    target_dy = dy
    target_dist = math.hypot(target_dx, target_dy)

    # Validity checks:
    # 1. Drone must be airborne (altitude >= 0.5m)
    # 2. Depression angle must be well within valid forward-looking quadrant (0.12 < theta_dep < 1.48)
    distance_valid = bool(current_altitude >= 0.5 and 0.12 < theta_dep < 1.48)

    # Metric confidence based on pixel centering and attitude stability
    conf_x = max(0.0, 1.0 - min(1.0, abs(error_x) / (config.cx * 0.95)) * 0.35)
    conf_pitch = max(0.0, 1.0 - min(1.0, abs(vehicle_pitch) / 0.35) * 0.30)
    confidence = (conf_x * conf_pitch) if distance_valid else 0.0

    return TargetGeometry(
        target_id=target_id,
        target_valid=True,
        pixel_error_x=round(float(error_x), 2),
        pixel_error_y=round(float(error_y), 2),
        distance_dx=round(float(dx), 3),
        distance_dy=round(float(dy), 3),
        ground_distance=round(float(ground_dist), 3),
        target_dx=round(float(max(2.5, min(18.0, target_dx))), 3),
        target_dy=round(float(max(-8.0, min(8.0, target_dy))), 3),
        target_dist=round(float(max(3.0, min(18.0, target_dist))), 3),
        distance_confidence=round(float(confidence), 3),
        distance_valid=distance_valid
    )
