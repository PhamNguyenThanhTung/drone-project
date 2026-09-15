#!/usr/bin/env python3
"""
Target Identity & State Management Subsystem.
Enforces logical mission identity continuity, fail-closed target lock,
and spatio-temporal reacquisition gating for UAV visual tracking.

Safety Invariants:
1. TRACK_ID CHANGE != TARGET_IDENTITY CHANGE
2. TARGET LOST != SELECT ANOTHER PERSON
3. NEVER auto-retarget merely because a candidate has a larger bbox,
   higher confidence, or higher area * confidence.
"""

import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any


STATE_NO_TARGET = 'NO_TARGET'
STATE_TARGET_SELECTED = 'TARGET_SELECTED'
STATE_TARGET_LOCKED = 'TARGET_LOCKED'
STATE_TRACKING = 'TRACKING'
STATE_UNCERTAIN = 'UNCERTAIN'
STATE_TARGET_LOST = 'TARGET_LOST'


def calculate_iou(box_a: Tuple[float, float, float, float],
                  box_b: Tuple[float, float, float, float]) -> float:
    """Compute Intersection-over-Union between two boxes (x1, y1, x2, y2)."""
    ax1, ay1, ax2, ay2 = box_a
    bx1, by1, bx2, by2 = box_b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0.0:
        return 0.0
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - inter
    return inter / union if union > 0.0 else 0.0


@dataclass
class TargetHandle:
    """
    Persistent logical mission identity.
    Decouples mission-level target identity from ephemeral tracker IDs.
    """
    handle_id: str                              # e.g., "TARGET_001"
    state: str = STATE_NO_TARGET                # Target state enum
    lock_mode: str = 'MANUAL'                   # 'MANUAL' or 'AUTO'
    current_track_id: Optional[int] = None      # Active ByteTrack track_id
    previous_track_ids: List[int] = field(default_factory=list)

    created_at: float = 0.0                     # Timestamp seconds
    last_seen: float = 0.0                      # Timestamp seconds of last confirmed observation

    # 2D Bounding Box & Motion State (in inference coordinates)
    last_confirmed_bbox: Optional[Tuple[float, float, float, float]] = None
    smoothed_bbox: Optional[Tuple[float, float, float, float]] = None
    velocity_2d: Tuple[float, float] = (0.0, 0.0)  # (vx_px_s, vy_px_s)
    last_confidence: float = 0.0

    # Tracking metrics
    consecutive_detections: int = 0
    consecutive_misses: int = 0
    reacquire_count: int = 0

    def is_active(self) -> bool:
        """True if the target is in an active mission state."""
        return self.state in (STATE_TARGET_LOCKED, STATE_TRACKING, STATE_UNCERTAIN)

    def is_tracking(self) -> bool:
        """True if target is currently confirmed visible and tracked."""
        return self.state == STATE_TRACKING

    def to_dict(self) -> Dict[str, Any]:
        """Serialize handle metadata for telemetry and logging."""
        return {
            'handle_id': self.handle_id,
            'state': self.state,
            'current_track_id': self.current_track_id,
            'previous_track_ids': list(self.previous_track_ids),
            'lock_mode': self.lock_mode,
            'last_seen': round(self.last_seen, 3),
            'confidence': round(self.last_confidence, 3),
            'velocity_2d': (round(self.velocity_2d[0], 2), round(self.velocity_2d[1], 2)),
            'reacquire_count': self.reacquire_count,
            'consecutive_detections': self.consecutive_detections,
            'consecutive_misses': self.consecutive_misses,
        }


class TargetStateManager:
    """
    Target State Manager & Governance Subsystem.
    Owns logical target lifecycle, fail-closed policy, and conservative reacquisition.
    """

    def __init__(self,
                 auto_track: bool = False,
                 reacquire_timeout_s: float = 3.5,
                 target_lost_timeout_s: float = 4.0,
                 reacquire_min_iou: float = 0.15,
                 alpha_smooth: float = 0.60,
                 max_implied_speed_px_s: float = 450.0,
                 separation_margin: float = 0.20):
        self.auto_track = auto_track
        self.reacquire_timeout_s = reacquire_timeout_s
        self.target_lost_timeout_s = target_lost_timeout_s
        self.reacquire_min_iou = reacquire_min_iou
        self.alpha_smooth = alpha_smooth
        self.max_implied_speed_px_s = max_implied_speed_px_s
        self.separation_margin = separation_margin

        self.target_handle: Optional[TargetHandle] = None
        self.handle_counter: int = 0
        self.last_update_time: float = 0.0

    def select_target(self,
                      track_id: int,
                      bbox: Optional[Tuple[float, float, float, float]] = None,
                      conf: float = 1.0,
                      lock_mode: str = 'MANUAL',
                      now: Optional[float] = None) -> TargetHandle:
        """Explicitly select and lock a mission target."""
        if track_id < 0:
            self.clear_target(reason='negative_id', now=now)
            return None

        now_t = now if now is not None else time.time()
        self.handle_counter += 1
        handle_id = f"TARGET_{self.handle_counter:03d}"

        self.target_handle = TargetHandle(
            handle_id=handle_id,
            state=STATE_TARGET_LOCKED,
            lock_mode=lock_mode,
            current_track_id=int(track_id),
            previous_track_ids=[],
            created_at=now_t,
            last_seen=now_t,
            last_confirmed_bbox=bbox,
            smoothed_bbox=bbox,
            last_confidence=conf,
            consecutive_detections=1,
            consecutive_misses=0,
            reacquire_count=0
        )
        return self.target_handle

    def clear_target(self, reason: str = 'operator_clear', now: Optional[float] = None) -> None:
        """Invalidate the target handle and reset to NO_TARGET."""
        self.target_handle = None

    def update(self,
               cands: List[Tuple[int, float, float, float, float, float, float]],
               all_persons: Optional[List[Tuple]] = None,
               now: Optional[float] = None,
               is_manual_flight: bool = False) -> Tuple[Optional[Tuple], str]:
        """
        Evaluate candidates against target handle.
        Returns (matched_candidate_tuple_or_None, target_state_string).
        """
        now_t = now if now is not None else time.time()
        dt = max(0.01, now_t - self.last_update_time) if self.last_update_time > 0.0 else 0.033
        self.last_update_time = now_t

        # -------------------------------------------------------------
        # CASE 1: NO ACTIVE TARGET
        # -------------------------------------------------------------
        if self.target_handle is None:
            if not self.auto_track or not cands or is_manual_flight:
                return None, STATE_NO_TARGET

            # Auto-track initial acquisition: pick candidate with highest area*conf
            best_cand = max(cands, key=lambda c: c[6] * c[5])
            if best_cand[5] >= 0.40:
                handle = self.select_target(
                    track_id=int(best_cand[0]),
                    bbox=best_cand[1:5],
                    conf=float(best_cand[5]),
                    lock_mode='AUTO',
                    now=now_t
                )
                handle.state = STATE_TRACKING
                return best_cand, STATE_TRACKING
            return None, STATE_NO_TARGET

        # -------------------------------------------------------------
        # CASE 2: ACTIVE TARGET EXISTS
        # -------------------------------------------------------------
        handle = self.target_handle
        cur_id = handle.current_track_id

        # Subcase 2A: Direct Match on current_track_id
        matched = [c for c in cands if c[0] == cur_id]
        if matched:
            cand = matched[0]
            cx_new = (cand[1] + cand[3]) / 2.0
            cy_new = (cand[2] + cand[4]) / 2.0

            if handle.last_confirmed_bbox is not None and dt > 0.001:
                cx_old = (handle.last_confirmed_bbox[0] + handle.last_confirmed_bbox[2]) / 2.0
                cy_old = (handle.last_confirmed_bbox[1] + handle.last_confirmed_bbox[3]) / 2.0
                vx_raw = (cx_new - cx_old) / dt
                vy_raw = (cy_new - cy_old) / dt
                # Filter velocity spikes
                vx_clamped = max(-500.0, min(500.0, vx_raw))
                vy_clamped = max(-500.0, min(500.0, vy_raw))
                alpha_v = 0.35
                handle.velocity_2d = (
                    alpha_v * vx_clamped + (1.0 - alpha_v) * handle.velocity_2d[0],
                    alpha_v * vy_clamped + (1.0 - alpha_v) * handle.velocity_2d[1]
                )

            # Apply EMA smoothing on bounding box
            if handle.smoothed_bbox is not None:
                alpha = self.alpha_smooth
                sx1 = alpha * cand[1] + (1.0 - alpha) * handle.smoothed_bbox[0]
                sy1 = alpha * cand[2] + (1.0 - alpha) * handle.smoothed_bbox[1]
                sx2 = alpha * cand[3] + (1.0 - alpha) * handle.smoothed_bbox[2]
                sy2 = alpha * cand[4] + (1.0 - alpha) * handle.smoothed_bbox[3]
                handle.smoothed_bbox = (sx1, sy1, sx2, sy2)
            else:
                handle.smoothed_bbox = cand[1:5]

            handle.last_confirmed_bbox = cand[1:5]
            handle.last_confidence = float(cand[5])
            handle.last_seen = now_t
            handle.consecutive_detections += 1
            handle.consecutive_misses = 0
            handle.state = STATE_TRACKING

            return cand, STATE_TRACKING

        # Subcase 2B: Direct Match MISSING -> Bounded Fail-Closed Reacquisition
        handle.consecutive_misses += 1
        age = now_t - handle.last_seen

        # If manual flight is active or loss timeout exceeded, do NOT attempt reacquire
        if is_manual_flight or age > self.reacquire_timeout_s:
            if age > self.target_lost_timeout_s:
                handle.state = STATE_TARGET_LOST
            else:
                handle.state = STATE_UNCERTAIN
            return None, handle.state

        # Attempt conservative, gated reacquisition
        reacquired_cand = self._attempt_gated_reacquisition(cands, handle, age, dt)
        if reacquired_cand is not None:
            # Reacquisition successful! Update lineage and state
            old_id = handle.current_track_id
            new_id = int(reacquired_cand[0])
            handle.previous_track_ids.append(old_id)
            handle.current_track_id = new_id

            # Reset smoothed box to eliminate anchor blending across tracks
            handle.last_confirmed_bbox = reacquired_cand[1:5]
            handle.smoothed_bbox = reacquired_cand[1:5]
            handle.last_confidence = float(reacquired_cand[5])
            handle.last_seen = now_t
            handle.consecutive_detections = 1
            handle.consecutive_misses = 0
            handle.reacquire_count += 1
            handle.state = STATE_TRACKING

            return reacquired_cand, STATE_TRACKING

        # FAIL-CLOSED: No candidate passed gates or ambiguity detected
        if age > self.target_lost_timeout_s:
            handle.state = STATE_TARGET_LOST
        else:
            handle.state = STATE_UNCERTAIN

        return None, handle.state

    def _attempt_gated_reacquisition(
        self,
        cands: List[Tuple[int, float, float, float, float, float, float]],
        handle: TargetHandle,
        age: float,
        dt: float
    ) -> Optional[Tuple]:
        """
        Conservative reacquisition evaluating scale, aspect, spatial, and ambiguity gates.
        Enforces fail-closed behavior on crossing or distractor presence.
        """
        if not cands or handle.last_confirmed_bbox is None:
            return None

        ref_x1, ref_y1, ref_x2, ref_y2 = handle.last_confirmed_bbox
        ref_w = max(1.0, ref_x2 - ref_x1)
        ref_h = max(1.0, ref_y2 - ref_y1)
        ref_area = ref_w * ref_h
        ref_ar = ref_h / ref_w
        ref_cx = (ref_x1 + ref_x2) / 2.0
        ref_cy = (ref_y1 + ref_y2) / 2.0

        # Motion-predicted center
        vx, vy = handle.velocity_2d
        pred_cx = ref_cx + vx * min(age, 1.5)
        pred_cy = ref_cy + vy * min(age, 1.5)

        scored_candidates = []

        for cand in cands:
            c_id, c_x1, c_y1, c_x2, c_y2, c_cf, c_area = cand
            c_w = max(1.0, c_x2 - c_x1)
            c_h = max(1.0, c_y2 - c_y1)
            c_cx = (c_x1 + c_x2) / 2.0
            c_cy = (c_y1 + c_y2) / 2.0
            c_ar = c_h / c_w

            # -------------------------------------------------------------
            # GATE 1: Scale Invariance Gate
            # Human box area cannot abruptly change by more than 2.8x or drop below 0.35x
            # -------------------------------------------------------------
            area_ratio = c_area / ref_area
            if area_ratio < 0.35 or area_ratio > 2.80:
                continue

            # -------------------------------------------------------------
            # GATE 2: Aspect Ratio Consistency Gate
            # Human posture aspect ratio cannot distort by > 60%
            # -------------------------------------------------------------
            ar_diff = abs(c_ar - ref_ar) / ref_ar
            if ar_diff > 0.60:
                continue

            # -------------------------------------------------------------
            # GATE 3: Physical Displacement & Implied Velocity Gate
            # -------------------------------------------------------------
            disp = ((c_cx - ref_cx) ** 2 + (c_cy - ref_cy) ** 2) ** 0.5
            v_implied = disp / max(age, 0.05)
            if v_implied > self.max_implied_speed_px_s:
                continue

            # -------------------------------------------------------------
            # GATE 4: Spatial & IoU Proximity Gate
            # -------------------------------------------------------------
            iou = calculate_iou(handle.last_confirmed_bbox, (c_x1, c_y1, c_x2, c_y2))

            dist_x = abs(c_cx - pred_cx) / max(ref_w, 30.0)
            dist_y = abs(c_cy - pred_cy) / max(ref_h, 30.0)
            dist_norm = (dist_x ** 2 + dist_y ** 2) ** 0.5

            # Must satisfy either valid IoU or close normalized distance
            is_spatially_valid = (iou >= self.reacquire_min_iou) or (dist_norm <= 1.25 and v_implied <= 350.0)
            if not is_spatially_valid:
                continue

            # Calculate composite candidate score
            proximity_score = max(0.0, 1.0 - dist_norm / 2.0)
            score = 0.50 * iou + 0.30 * proximity_score + 0.20 * min(1.0, float(c_cf))
            scored_candidates.append((score, cand))

        if not scored_candidates:
            return None

        # Sort by score descending
        scored_candidates.sort(key=lambda x: x[0], reverse=True)

        # -------------------------------------------------------------
        # GATE 5: Ambiguity & Candidate Separation Gate
        # If multiple candidates pass gates (e.g. crossing pedestrians),
        # the top candidate MUST have an unambiguous separation margin.
        # -------------------------------------------------------------
        if len(scored_candidates) == 1:
            best_score, best_cand = scored_candidates[0]
            if best_score >= 0.35:
                return best_cand
            return None

        top1_score, top1_cand = scored_candidates[0]
        top2_score, _ = scored_candidates[1]

        margin = top1_score - top2_score
        if margin < self.separation_margin:
            # Ambiguity detected! Fail-closed to prevent hijacking
            return None

        if top1_score >= 0.40:
            return top1_cand

        return None
