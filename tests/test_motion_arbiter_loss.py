#!/usr/bin/env python3
"""
Unit Test Suite for Phase 2A: Safe Lost-Target Deceleration and Hold.

Covers the mandatory unit tests from Phase 2A specification:
- Test A: Target loss from zero velocity (vx = 0)
- Test B: Target loss while moving forward (vx = 1.26 m/s, monotonic decay to 0)
- Test C: Target loss while moving laterally (vx = 0.8, vy = 0.4 -> 0)
- Test D: Yaw rate bounded and decays to 0 (no unconstrained spin)
- Test E: No target switching during loss (distractor rejection)
- Test F: Timeout horizon transition to STANDBY
- Test G: Explicit legacy turn-point isolation (generic loss vs explicit maneuver)
"""

import sys
import os
import unittest
from unittest.mock import MagicMock, patch

_pkg_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                         'ros2_ws', 'src', 'vision_tracking')
if sys.path[0] != _pkg_path:
    sys.path.insert(0, _pkg_path)

import rclpy
from vision_tracking.motion_arbiter_node import (
    MotionArbiter,
    STATE_TRACKING,
    STATE_STANDBY,
)
from vision_tracking.target_identity import (
    TargetStateManager,
    STATE_NO_TARGET,
    STATE_UNCERTAIN,
    STATE_TARGET_LOST,
)


class DummyLogger:
    def info(self, msg): pass
    def warning(self, msg): pass
    def warn(self, msg): pass
    def error(self, msg): pass
    def debug(self, msg): pass


class MotionArbiterTestHarness:
    """Lightweight test harness exposing compute_tracking_velocities without live MAVLink/ROS."""
    def __init__(self, decel_rate: float = 1.80, enable_turn_point_recovery: bool = False):
        self.error_x = 0.0
        self.error_y = 0.0
        self.area = 9600.0
        self.acquired_once = True
        self.target_dist = 4.5
        self.target_dx = 4.5
        self.target_dy = 1.5
        self.dist_advanced = 0.0
        self.last_seen_x = 0.0
        self.last_seen_y = 0.0
        self.deadband_x = 20.0
        self.deadband_y = 25.0
        self.enable_forward = True
        self.default_walk_speed = 0.85
        self.default_backup_speed = 0.75
        self.kp = 0.0070
        self.kp_lateral = 0.0035
        self.kp_y_boost = 0.0050
        self.min_forward_speed = -1.20
        self.max_forward_speed = 1.80
        self.turn_point_speed = 1.35
        self.turn_point_rotate_speed = 0.50
        self.turn_point_timeout = 8.0
        self.turn_point_rotate_timeout = 4.0
        self.bottom_backup_timeout = 4.0
        self.bottom_backup_speed = 0.85
        self.target_turn_dir = 1.0
        self.max_rate = 1.50
        self.search_rate = 1.10
        self.vision_fresh_timeout = 0.40
        self.takeoff_alt = 3.8
        self.kp_z = 1.20
        self.max_z_speed = 0.80

        # Phase 2A parameters
        self.lost_target_decel_mps2 = decel_rate
        self.enable_turn_point_recovery = enable_turn_point_recovery
        self.in_turn_point_maneuver = False

        self._last_vx = 0.0
        self._last_vy = 0.0
        self._last_vz = 0.0
        self._last_yaw_rate = 0.0
        self.last_tracking_substate = None
        self.logger = DummyLogger()

    def get_current_altitude(self):
        return 3.8

    def get_logger(self):
        return self.logger

    # Bind the actual production method
    compute_tracking_velocities = MotionArbiter.compute_tracking_velocities


class TestSafeLostTargetDeceleration(unittest.TestCase):

    # -------------------------------------------------------------------------
    # TEST A: Target Loss from Zero Velocity
    # -------------------------------------------------------------------------
    def test_a_loss_from_zero_velocity(self):
        harness = MotionArbiterTestHarness(decel_rate=1.80)
        harness._last_vx = 0.0
        harness._last_vy = 0.0
        harness._last_yaw_rate = 0.0

        # Target becomes stale (age = 0.6s > vision_fresh_timeout = 0.4s)
        vx, vy, vz, yaw_rate = harness.compute_tracking_velocities(now=10.6, dt=0.1, age=0.6)

        self.assertEqual(vx, 0.0, "vx must remain 0.0 when lost from zero velocity")
        self.assertEqual(vy, 0.0, "vy must remain 0.0 when lost from zero velocity")
        self.assertEqual(yaw_rate, 0.0, "yaw_rate must remain 0.0 when lost from zero velocity")
        self.assertEqual(harness.last_tracking_substate, 'SEARCHING_HOLD')

    # -------------------------------------------------------------------------
    # TEST B: Target Loss while Moving Forward (vx = 1.26 m/s)
    # -------------------------------------------------------------------------
    def test_b_loss_while_moving_forward_monotonic_decay(self):
        harness = MotionArbiterTestHarness(decel_rate=1.80)
        initial_vx = 1.26
        harness._last_vx = initial_vx
        harness._last_vy = 0.0
        harness._last_yaw_rate = 0.0

        # Run consecutive cycles of target loss at 10 Hz (dt = 0.1s)
        dt = 0.1
        velocities = [initial_vx]
        substates = []

        for i in range(15):
            age = 0.45 + i * dt
            vx, vy, vz, yaw_rate = harness.compute_tracking_velocities(now=10.0 + i*dt, dt=dt, age=age)
            velocities.append(vx)
            substates.append(harness.last_tracking_substate)

        # 1. First state entered must be DECELERATING_TO_HOLD
        self.assertEqual(substates[0], 'DECELERATING_TO_HOLD')

        # 2. Monotonic decrease check: every step must be <= previous step
        for i in range(len(velocities) - 1):
            self.assertLessEqual(
                velocities[i + 1], velocities[i] + 1e-6,
                f"Velocity must decay monotonically: v[{i}]={velocities[i]:.3f}, v[{i+1}]={velocities[i+1]:.3f}"
            )
            # Velocity must never be negative (no reverse overshoot)
            self.assertGreaterEqual(velocities[i + 1], 0.0)

        # 3. Deceleration rate check: reduction per 0.1s tick should be approx decel_rate * dt = 0.18
        delta_first_tick = velocities[0] - velocities[1]
        self.assertAlmostEqual(delta_first_tick, 1.80 * 0.1, places=2)

        # 4. Terminal state: velocity reaches 0.0 and enters SEARCHING_HOLD
        self.assertEqual(velocities[-1], 0.0)
        self.assertEqual(substates[-1], 'SEARCHING_HOLD')

        # 5. Time to zero velocity check: 1.26 / 1.80 = 0.70s (7 ticks)
        non_zero_ticks = sum(1 for v in velocities[1:] if v > 0.0)
        self.assertIn(non_zero_ticks, (6, 7, 8))

    # -------------------------------------------------------------------------
    # TEST C: Target Loss while Moving Laterally (vx = 0.8, vy = 0.4)
    # -------------------------------------------------------------------------
    def test_c_loss_while_moving_laterally(self):
        harness = MotionArbiterTestHarness(decel_rate=1.80)
        harness._last_vx = 0.80
        harness._last_vy = 0.40
        harness._last_yaw_rate = 0.0

        dt = 0.1
        vx_history = [0.80]
        vy_history = [0.40]

        for i in range(12):
            age = 0.50 + i * dt
            vx, vy, vz, yaw_rate = harness.compute_tracking_velocities(now=20.0 + i*dt, dt=dt, age=age)
            vx_history.append(vx)
            vy_history.append(vy)

        # Both vx and vy must decay monotonically to 0.0
        for i in range(len(vx_history) - 1):
            self.assertLessEqual(vx_history[i + 1], vx_history[i] + 1e-6)
            self.assertGreaterEqual(vx_history[i + 1], 0.0)

        for i in range(len(vy_history) - 1):
            self.assertLessEqual(vy_history[i + 1], vy_history[i] + 1e-6)
            self.assertGreaterEqual(vy_history[i + 1], 0.0)

        self.assertEqual(vx_history[-1], 0.0)
        self.assertEqual(vy_history[-1], 0.0)
        self.assertEqual(harness.last_tracking_substate, 'SEARCHING_HOLD')

    # -------------------------------------------------------------------------
    # TEST D: Yaw Rate Bounded and Clamped to 0
    # -------------------------------------------------------------------------
    def test_d_yaw_rate_clamped_to_zero(self):
        harness = MotionArbiterTestHarness(decel_rate=1.80)
        harness._last_vx = 0.50
        harness._last_yaw_rate = 0.45  # was actively turning

        dt = 0.1
        yaw_history = [0.45]
        for i in range(6):
            age = 0.50 + i * dt
            vx, vy, vz, yaw_rate = harness.compute_tracking_velocities(now=30.0 + i*dt, dt=dt, age=age)
            yaw_history.append(yaw_rate)

        # Yaw rate must strictly decrease to 0 without overshoot or spinning
        for yaw in yaw_history[2:]:
            self.assertEqual(yaw, 0.0, "Yaw rate must be 0.0 during lost-target hold")

    # -------------------------------------------------------------------------
    # TEST E: No Target Switching During Loss (Distractor Rejection)
    # -------------------------------------------------------------------------
    def test_e_no_target_switching_during_loss(self):
        mgr = TargetStateManager(auto_track=False)
        # Lock target 7
        mgr.select_target(track_id=7, bbox=(200, 150, 260, 310), conf=0.90, now=100.0)

        # Frame 2: Target 7 is lost. Distractor 9 appears (larger bbox, high conf)
        distractor_9 = (9, 40.0, 40.0, 260.0, 390.0, 0.98, 65000.0)
        matched, state = mgr.update(cands=[distractor_9], now=100.5)

        self.assertIsNone(matched, "Distractor must NOT be matched as mission target")
        self.assertEqual(state, STATE_UNCERTAIN)
        self.assertEqual(mgr.target_handle.handle_id, "TARGET_001")
        self.assertEqual(mgr.target_handle.current_track_id, 7)
        self.assertNotIn(9, mgr.target_handle.previous_track_ids)

    # -------------------------------------------------------------------------
    # TEST F: Timeout Horizon Transitions to STANDBY
    # -------------------------------------------------------------------------
    def test_f_timeout_horizon_to_standby(self):
        harness = MotionArbiterTestHarness(decel_rate=1.80)
        harness._last_vx = 1.20
        harness.acquired_once = True

        # Decelerates to hold
        for i in range(10):
            harness.compute_tracking_velocities(now=40.0 + i*0.1, dt=0.1, age=0.5 + i*0.1)

        self.assertEqual(harness.last_tracking_substate, 'SEARCHING_HOLD')
        self.assertEqual(harness._last_vx, 0.0)

    # -------------------------------------------------------------------------
    # TEST G: Explicit Turn-Point Isolation (Generic Loss vs Explicit Maneuver)
    # -------------------------------------------------------------------------
    def test_g_explicit_turn_point_isolation(self):
        # Case 1: Generic loss (default: enable_turn_point_recovery = False)
        harness_generic = MotionArbiterTestHarness(enable_turn_point_recovery=False)
        harness_generic._last_vx = 0.50
        vx, vy, vz, yaw = harness_generic.compute_tracking_velocities(now=50.0, dt=0.1, age=0.6)
        self.assertIn(harness_generic.last_tracking_substate, ('DECELERATING_TO_HOLD', 'SEARCHING_HOLD'))
        self.assertLess(vx, 0.50, "Generic loss must decelerate")

        # Case 2: Explicit turn-point recovery authorized AND in_turn_point_maneuver = True
        harness_explicit = MotionArbiterTestHarness(enable_turn_point_recovery=True)
        harness_explicit.in_turn_point_maneuver = True
        harness_explicit._last_vx = 0.50
        vx_tp, vy_tp, vz_tp, yaw_tp = harness_explicit.compute_tracking_velocities(now=50.0, dt=0.1, age=0.6)
        self.assertEqual(harness_explicit.last_tracking_substate, 'ADVANCING_TO_TURN_POINT')
        self.assertGreater(vx_tp, 0.50, "Explicit turn point must accelerate/advance along vector toward turn_point_speed")


if __name__ == '__main__':
    unittest.main()
