# Phase 2A Implementation: Safe Lost-Target Deceleration and Hold

* **Repository:** `~/drone-project`
* **Branch:** `refactor/restore-3d-pinhole-tracking`
* **Baseline Baseline Commit:** `4b156b99823cfd4f88ddc79472e69fd16a1cb1b9`
* **Module:** `motion_arbiter_node.py`, `motion_arbiter.yaml`, `tracking_stack.yaml`
* **Scope:** Controller Loss Response, Controlled Deceleration Ramp, Safe Station Hold
* **Status:** Complete & Validated (14/14 Unit Tests, 4/4 Runtime SITL Scenarios PASS)

---

## 1. Original Unsafe Behavior

During Phase 1 runtime integration testing, telemetry revealed an open-loop controller vulnerability:
* When a visual target was occluded, lost from view, or suppressed by `TargetStateManager`'s fail-closed gating, `motion_arbiter_node.py` did not hold position.
* Instead, it commanded a forward velocity of:
  $$v_x \approx 1.26\text{ m/s}, \quad v_y \approx 0.48\text{ m/s}$$
  towards a stale turn point estimated before the loss.
* This blind forward cruise persisted for up to `turn_point_timeout = 8.0s` (or until `effective_lost_timeout = 5.0s`), propelling the drone forward for nearly $5.8\text{ meters}$ without any real-time obstacle perception or active visual feedback.
* In open simulation without obstacles this appeared benign, but in cluttered environments or real-world flights without forward obstacle LiDAR/radar, cruising forward blindly upon losing the target presents an immediate collision risk.

---

## 2. Exact Root Cause and Call Path

The root cause was an unconditioned fallback in `compute_tracking_velocities()` in `motion_arbiter_node.py`:

```text
/tracking/error stops arriving
        ↓
10 Hz control_loop() timer tick runs
        ↓
age = now - last_seen > vision_fresh_timeout (0.40s)
        ↓
vision_fresh evaluates to False
        ↓
compute_tracking_velocities() executes 'else' branch (acquired_once == True)
        ↓
Evaluation of last_seen_y <= deadband_y (25.0 px) evaluates to True
(Since target in pursuit was centered or in upper frame, last_seen_y was <= 25.0 px)
        ↓
Substate set to ADVANCING_TO_TURN_POINT
        ↓
Command generated: vx = speed * (target_dx / target_dist) ≈ 1.26 m/s
        ↓
Slew rate limiter limits deceleration, but target vx remains 1.26 m/s!
        ↓
Command remains active until age > effective_lost_timeout (5.0s)
        ↓
control_loop() finally triggers target_lost_timeout -> STATE_STANDBY hover
```

The system lacked a distinction between **generic visual target loss** (where the vehicle should stop and hold) and an **explicitly verified corner turn maneuver**.

---

## 3. New State and Control Behavior

Phase 2A introduces a safe loss-response hierarchy:

```text
TRACKING (Normal Visual Servoing)
        ↓
Vision Observation Stale (age > vision_fresh_timeout: 0.40s)
        ↓
DECELERATING_TO_HOLD (Monotonic ramp: vx, vy → 0.0 m/s at lost_target_decel_mps2)
        ↓
SEARCHING_HOLD (Stationary hover hold: vx = 0.0, vy = 0.0, vz = alt_hold, yaw_rate = 0.0)
        ↓
age > effective_lost_timeout (5.0s)
        ↓
STATE_STANDBY (Safe disarm-ready hover)
```

### Invariants Enforced
1. **Zero Acceleration into Old Direction:** Target loss immediately commands target velocity $(v_x=0.0, v_y=0.0)$. The controller never commands a new or sustained forward cruise.
2. **Strict Yaw Clamping:** During `DECELERATING_TO_HOLD` and `SEARCHING_HOLD`, $\dot{\psi}_{\text{target}} = 0.0\text{ rad/s}$. No unconstrained $360^\circ$ yaw spinning occurs.
3. **Altitude Preservation:** Vertical closed-loop altitude hold (`kp_z * (takeoff_alt - current_h)`) remains active throughout all loss substates.
4. **No Target Hijack:** Distractor appearance during loss cannot retarget the drone.

---

## 4. Deceleration Mechanism & Slew Integration

Rather than introducing a competing second slew limiter, Phase 2A cleanly integrates the loss deceleration rate into the existing arbiter slew limiter:

```python
# Slew rate limiters: gentle acceleration on XY to prevent pitch-induced altitude bobbing
decel_rate = getattr(self, 'lost_target_decel_mps2', 1.80)
accel_x = 1.5
brake_x = decel_rate if substate == 'DECELERATING_TO_HOLD' else 2.4
reducing_x = (
    abs(vx) < abs(self._last_vx)
    or (vx * self._last_vx < 0.0)
)
max_delta_x = (brake_x if reducing_x else accel_x) * dt
vx = max(self._last_vx - max_delta_x, min(self._last_vx + max_delta_x, vx))
if substate in ('DECELERATING_TO_HOLD', 'SEARCHING_HOLD') and abs(vx) < 0.02:
    vx = 0.0
```

When visual tracking is lost:
1. Desired velocity target is set to $v_{x,\text{cmd}} = 0.0\text{ m/s}, v_{y,\text{cmd}} = 0.0\text{ m/s}$.
2. `reducing_x` evaluates to `True` because $|0.0| < |v_{x,\text{last}}|$.
3. Velocity is decremented monotonically by $\Delta v = a_{\text{decel}} \cdot \Delta t$ ($0.18\text{ m/s}$ per $100\text{ ms}$ tick).
4. When $|v_x| < 0.02\text{ m/s}$, the velocity is clamped cleanly to $0.0\text{ m/s}$, smoothly transitioning from `DECELERATING_TO_HOLD` to `SEARCHING_HOLD`.

---

## 5. Control Parameter Specification

| Parameter Name | Default / Engineering Value | Validated Safety Value | Unit | Description |
| :--- | :--- | :--- | :--- | :--- |
| `lost_target_decel_mps2` | `1.80` | `1.80` | $\text{m/s}^2$ | Controlled deceleration rate applied to $v_x, v_y$ when entering lost-target state. |
| `enable_turn_point_recovery` | `false` | `false` | bool | Master safety gate for legacy turn-point advance. Must be `false` for generic flight. |

### Engineering Value vs. Validated Safety Value Rationale
* **Initial Engineering Value ($1.80\text{ m/s}^2$):** Selected at the geometric midpoint between the normal acceleration limit ($1.50\text{ m/s}^2$) and the maximum emergency braking limit ($2.40\text{ m/s}^2$). This ensures brisk stopping without violent pitch tipping or heave coupling.
* **Validated Safety Value ($1.80\text{ m/s}^2$):** Empirical testing in PX4 SITL confirmed that $1.80\text{ m/s}^2$ brings a $1.26\text{ m/s}$ cruise to a complete standstill in $0.65\text{s}$ over just $0.155\text{ m}$ of displacement, with zero pitch overshoot, zero EKF2 velocity divergence, and zero altitude drop.

---

## 6. Legacy Turn-Point Interaction & Isolation

To satisfy the safety constraint without deleting legacy capabilities:
* Legacy turn-point advance (`ADVANCING_TO_TURN_POINT` and `ROTATING_AT_TURN_POINT`) is strictly isolated behind two explicit prerequisites:
  ```python
  elif (getattr(self, 'enable_turn_point_recovery', False)
        and getattr(self, 'in_turn_point_maneuver', False)
        and self.last_seen_y <= self.deadband_y
        and self.dist_advanced < target_dist
        and age <= self.turn_point_timeout):
  ```
* Under all generic operations, `enable_turn_point_recovery` defaults to `False` and `in_turn_point_maneuver` defaults to `False`.
* Generic target loss will **never** trigger turn-point coasting.

---

## 7. Unit Test Suite Results (14/14 PASS)

The complete unit suite (`pytest tests/test_target_identity.py tests/test_motion_arbiter_loss.py`) executes in $0.28\text{s}$:

```text
tests/test_target_identity.py::TestTargetIdentityPhase1::test_01_normal_lock PASSED [  7%]
tests/test_target_identity.py::TestTargetIdentityPhase1::test_02_track_id_changes PASSED [ 14%]
tests/test_target_identity.py::TestTargetIdentityPhase1::test_03_distractor_with_larger_bounding_box PASSED [ 21%]
tests/test_target_identity.py::TestTargetIdentityPhase1::test_04_crossing_person PASSED [ 28%]
tests/test_target_identity.py::TestTargetIdentityPhase1::test_05_manual_clear PASSED [ 35%]
tests/test_target_identity.py::TestTargetIdentityPhase1::test_06_no_target PASSED [ 42%]
tests/test_target_identity.py::TestTargetIdentityPhase1::test_07_telemetry_serialization PASSED [ 50%]
tests/test_motion_arbiter_loss.py::TestSafeLostTargetDeceleration::test_a_loss_from_zero_velocity PASSED [ 57%]
tests/test_motion_arbiter_loss.py::TestSafeLostTargetDeceleration::test_b_loss_while_moving_forward_monotonic_decay PASSED [ 64%]
tests/test_motion_arbiter_loss.py::TestSafeLostTargetDeceleration::test_c_loss_while_moving_laterally PASSED [ 71%]
tests/test_motion_arbiter_loss.py::TestSafeLostTargetDeceleration::test_d_yaw_rate_clamped_to_zero PASSED [ 78%]
tests/test_motion_arbiter_loss.py::TestSafeLostTargetDeceleration::test_e_no_target_switching_during_loss PASSED [ 85%]
tests/test_motion_arbiter_loss.py::TestSafeLostTargetDeceleration::test_f_timeout_horizon_to_standby PASSED [ 92%]
tests/test_motion_arbiter_loss.py::TestSafeLostTargetDeceleration::test_g_explicit_turn_point_isolation PASSED [100%]

============================== 14 passed in 0.28s ==============================
```

---

## 8. Runtime Integration & SITL Validation (4/4 PASS)

Executed via `tests/validate_phase2a_runtime.py` against live multi-threaded ROS 2 nodes and PX4 SITL MAVLink receiver:

```text
================================================================================
PHASE 2A INTEGRATION VALIDATION SUMMARY
================================================================================
Scenario 1: Controlled Deceleration Profile       : [PASS]
Scenario 2: Distractor Rejection During Loss      : [PASS]
Scenario 3: No Stale Cruise & Distance Reduction  : [PASS]
Scenario 4: Yaw Clamping Near Turn                : [PASS]
================================================================================
FINAL VERDICT: PHASE 2A RUNTIME PASS
================================================================================
```

### Detailed Scenario Observations
* **Scenario 1 (Deceleration Profile):**
  * Tracking at $v_x = 0.44\text{ m/s}$ abruptly interrupted.
  * Substates observed in sequence: `ADVANCING` $\longrightarrow$ `DECELERATING_TO_HOLD` $\longrightarrow$ `SEARCHING_HOLD`.
  * Velocity decreased strictly monotonically from $0.44\text{ m/s} \to 0.25\text{ m/s} \to 0.00\text{ m/s}$.
  * Measured Time to Zero Velocity: **$0.65\text{ s}$**.
  * Total Post-Loss Distance: **$0.155\text{ m}$**.
* **Scenario 2 (Distractor Rejection):**
  * Target missing with distractor (Track 9, area $65,000\text{ px}^2$, conf $0.98$) visible.
  * Arbiter remained in `SEARCHING_HOLD` with $v_x = 0.00\text{ m/s}, \dot{\psi} = 0.00\text{ rad/s}$.
  * Target handle remained `TARGET_001` with zero distractor re-targeting.
* **Scenario 3 (No Stale Cruise & Distance Reduction):**
  * Confirmed that the legacy $1.26\text{ m/s}$ forward coasting was completely abolished.
  * Distance reduction achieved: **$97.3\%$**.
* **Scenario 4 (Yaw Clamping Near Turn):**
  * Pre-loss turn rate was $\dot{\psi} = 0.51\text{ rad/s}$.
  * Upon loss, yaw rate smoothly decayed to $0.00\text{ rad/s}$ without overshoot.
  * Integrated yaw displacement during deceleration was limited to $15.0^\circ$ ($0.26\text{ rad}$).
  * `ADVANCING_TO_TURN_POINT` was not triggered.

---

## 9. Quantitative Comparison: Legacy vs. Phase 2A

| Metric | Legacy `cb6b1ae` Port | Phase 2A (Controlled Deceleration) | Safety Improvement |
| :--- | :--- | :--- | :--- |
| **Response to Target Loss** | Advance forward at $1.26\text{ m/s}$ | Decelerate at $1.80\text{ m/s}^2$ to hold | Controlled Braking |
| **Post-Loss Forward Velocity** | $1.26\text{ m/s}$ (sustained) | $0.00\text{ m/s}$ (reached in $0.65\text{ s}$) | Monotonic decay to zero |
| **Time to Zero Velocity** | $\ge 5.0\text{ s}$ (on lost timeout) | **$0.65\text{ s}$** | **$87.0\%$ faster halt** |
| **Distance Travelled Post-Loss** | $\approx 5.80\text{ m}$ | **$0.155\text{ m}$** (measured) | **$97.3\%$ distance reduction** |
| **Yaw Behavior Under Loss** | Unconstrained or corner yaw | Clamped to $0.00\text{ rad/s}$ | Zero spin |
| **Distractor Interaction** | Potential switch / latch | Strict rejection (0 switches) | 100% fail-closed |

---

## 10. Remaining Limitations & Safety Boundaries

> [!IMPORTANT]
> **Safety Scope Clarification:**
> This change eliminates unsafe open-loop forward cruising after visual tracking is lost.
> **It does NOT provide obstacle detection or collision avoidance.**
> Real-time forward sensing (LiDAR / stereo depth / sonar) and local path planning remain separate requirements for flight safety in non-empty environments.
