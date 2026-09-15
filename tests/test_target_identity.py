#!/usr/bin/env python3
"""
Unit Test Suite for Target Identity, Target Lock, and Fail-Closed Behavior.
Covers the 6 mandatory test cases from Implementation Phase 1 specification.
"""

import sys
import os
import unittest

# Ensure vision_tracking module from source is in path
_pkg_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                         'ros2_ws', 'src', 'vision_tracking')
if sys.path[0] != _pkg_path:
    sys.path.insert(0, _pkg_path)
if 'vision_tracking' in sys.modules and not hasattr(sys.modules['vision_tracking'], 'target_identity'):
    del sys.modules['vision_tracking']

from vision_tracking.target_identity import (
    TargetStateManager,
    TargetHandle,
    STATE_NO_TARGET,
    STATE_TARGET_LOCKED,
    STATE_TRACKING,
    STATE_UNCERTAIN,
    STATE_TARGET_LOST,
)


class TestTargetIdentityPhase1(unittest.TestCase):

    def setUp(self):
        self.mgr = TargetStateManager(
            auto_track=False,
            reacquire_timeout_s=3.0,
            target_lost_timeout_s=4.0,
            reacquire_min_iou=0.15,
            separation_margin=0.20
        )

    # ------------------------------------------------------------------
    # TEST 1: Normal Lock
    # Target A with track_id = 7. Handle is TARGET_001.
    # Consecutive observations keep TARGET_001 locked.
    # ------------------------------------------------------------------
    def test_01_normal_lock(self):
        # Operator locks Person A (track_id = 7) at center of frame (200, 150, 260, 310)
        box_a = (200.0, 150.0, 260.0, 310.0) # w=60, h=160, area=9600
        handle = self.mgr.select_target(track_id=7, bbox=box_a, conf=0.88, lock_mode='MANUAL', now=10.0)

        self.assertIsNotNone(handle)
        self.assertEqual(handle.handle_id, "TARGET_001")
        self.assertEqual(handle.current_track_id, 7)
        self.assertEqual(handle.previous_track_ids, [])

        # Frame 2: Person A moves slightly right (205, 150, 265, 310)
        cand_frame2 = [(7, 205.0, 150.0, 265.0, 310.0, 0.89, 9600.0)]
        matched, state = self.mgr.update(cand_frame2, now=10.1)

        self.assertIsNotNone(matched)
        self.assertEqual(matched[0], 7)
        self.assertEqual(state, STATE_TRACKING)
        self.assertEqual(self.mgr.target_handle.handle_id, "TARGET_001")
        self.assertEqual(self.mgr.target_handle.current_track_id, 7)
        self.assertEqual(self.mgr.target_handle.consecutive_detections, 2)

    # ------------------------------------------------------------------
    # TEST 2: Track ID Changes (Lineage Continuity)
    # TARGET_001 has track 7. Track 7 drops. Track 15 appears in expected window.
    # Handle remains TARGET_001; previous_track_ids becomes [7].
    # If track 15 appears far away / invalid -> rejected, remains unresolved.
    # ------------------------------------------------------------------
    def test_02_track_id_changes(self):
        # Step A: Lock track 7
        box_a = (200.0, 150.0, 260.0, 310.0)
        self.mgr.select_target(track_id=7, bbox=box_a, conf=0.85, now=10.0)

        # Step B: Track 7 disappears for 0.4s (brief tree trunk occlusion)
        matched_empty, state_empty = self.mgr.update([], now=10.4)
        self.assertIsNone(matched_empty)
        self.assertEqual(state_empty, STATE_UNCERTAIN)
        self.assertEqual(self.mgr.target_handle.handle_id, "TARGET_001")
        self.assertEqual(self.mgr.target_handle.current_track_id, 7)

        # Step C: ByteTrack assigns new track_id=15 to the re-emerging person at (210, 152, 270, 312)
        cand_reappear = [(15, 210.0, 152.0, 270.0, 312.0, 0.86, 9600.0)]
        matched, state = self.mgr.update(cand_reappear, now=10.6)

        self.assertIsNotNone(matched)
        self.assertEqual(matched[0], 15)
        self.assertEqual(state, STATE_TRACKING)
        # Verify logical mission identity survived
        self.assertEqual(self.mgr.target_handle.handle_id, "TARGET_001")
        self.assertEqual(self.mgr.target_handle.current_track_id, 15)
        self.assertEqual(self.mgr.target_handle.previous_track_ids, [7])
        self.assertEqual(self.mgr.target_handle.reacquire_count, 1)

        # Step D: Test negative case — if an unrelated track appears across the screen
        # Let's say track 15 drops, and an unrelated track 99 appears at (10, 10, 50, 80)
        self.mgr.update([], now=11.0)
        cand_unrelated = [(99, 10.0, 10.0, 50.0, 80.0, 0.90, 2800.0)] # Far away, wrong scale
        matched_bad, state_bad = self.mgr.update(cand_unrelated, now=11.2)
        self.assertIsNone(matched_bad)
        self.assertEqual(self.mgr.target_handle.handle_id, "TARGET_001")
        self.assertNotEqual(self.mgr.target_handle.current_track_id, 99)
        self.assertEqual(state_bad, STATE_UNCERTAIN)

    # ------------------------------------------------------------------
    # TEST 3: Distractor with Larger Bounding Box (P0 Regression Test)
    # Target A is lost temporarily. Distractor B is visible with much
    # larger area * confidence.
    # EXPECTED: Do NOT switch to Distractor B! Fail closed!
    # ------------------------------------------------------------------
    def test_03_distractor_with_larger_bounding_box(self):
        # Lock Target A (track_id = 1) at center (200, 150, 260, 310), area=9600
        box_a = (200.0, 150.0, 260.0, 310.0)
        self.mgr.select_target(track_id=1, bbox=box_a, conf=0.75, now=20.0)

        # Target A disappears (e.g. occluded).
        # Distractor B (track_id = 9) is close to camera: (50, 50, 250, 400), area=70000, conf=0.98
        # In old code: max(cands, key=area*conf) would immediately switch to Person B!
        cand_distractor_only = [(9, 50.0, 50.0, 250.0, 400.0, 0.98, 70000.0)]

        matched, state = self.mgr.update(cand_distractor_only, now=20.3)

        # CRITICAL ASSERTION: Must NOT switch to Person B!
        self.assertIsNone(matched, "FAIL: System hijacked by distractor with larger area!")
        self.assertIn(state, (STATE_UNCERTAIN, STATE_TARGET_LOCKED))
        self.assertEqual(self.mgr.target_handle.handle_id, "TARGET_001")
        self.assertEqual(self.mgr.target_handle.current_track_id, 1)
        self.assertNotIn(9, self.mgr.target_handle.previous_track_ids)

        # Further elapsed time without Target A leads to TARGET_LOST, NOT to Person B
        matched_late, state_late = self.mgr.update(cand_distractor_only, now=24.5)
        self.assertIsNone(matched_late)
        self.assertEqual(state_late, STATE_TARGET_LOST)
        self.assertEqual(self.mgr.target_handle.handle_id, "TARGET_001")
        self.assertEqual(self.mgr.target_handle.current_track_id, 1)

    # ------------------------------------------------------------------
    # TEST 4: Crossing Person (Ambiguity Margin Gate)
    # Target A is walking. Person B crosses target path.
    # When Target A is occluded, Person B is in the crossing zone.
    # EXPECTED: No silent target switch to B!
    # ------------------------------------------------------------------
    def test_04_crossing_person(self):
        # Lock Target A (track_id = 3) at (180, 150, 240, 310)
        box_a = (180.0, 150.0, 240.0, 310.0)
        self.mgr.select_target(track_id=3, bbox=box_a, conf=0.85, now=30.0)

        # Both Target A (track 3) and Crossing Person B (track 8) are visible
        cands_both = [
            (3, 190.0, 150.0, 250.0, 310.0, 0.85, 9600.0),
            (8, 210.0, 150.0, 270.0, 310.0, 0.82, 9600.0),
        ]
        matched, state = self.mgr.update(cands_both, now=30.1)
        self.assertEqual(matched[0], 3)

        # Target A is now temporarily occluded by Person B. Only Person B (track 8) is visible at crossing point
        # Person B has similar scale and is near the intersection
        cand_b_only = [
            (8, 200.0, 150.0, 260.0, 310.0, 0.82, 9600.0)
        ]
        # In old code, dist_norm < 1.2 would immediately transfer lock to Person 8!
        # In new code, Person B is moving in opposite/crossing direction, or if ambiguous,
        # we check separation and gating.
        # If Target A emerges shortly afterwards alongside Person B:
        cands_ambiguous = [
            (11, 195.0, 150.0, 255.0, 310.0, 0.80, 9600.0), # New track 11
            (8,  205.0, 150.0, 265.0, 310.0, 0.82, 9600.0), # Track 8
        ]
        # Both are inside the gate with close scores (margin < 0.20)
        matched_cross, state_cross = self.mgr.update(cands_ambiguous, now=30.4)

        # Must reject ambiguous match!
        self.assertIsNone(matched_cross, "FAIL: Ambiguous crossing caused false reacquisition!")
        self.assertEqual(state_cross, STATE_UNCERTAIN)

        # Once Target A clears the crossing and is clearly separated:
        # Target A continued right along its trajectory to (250, 150, 310, 310),
        # while Person B crossed and moved left to (130, 150, 190, 310)
        cands_separated = [
            (11, 250.0, 150.0, 310.0, 310.0, 0.85, 9600.0), # Target A continued right
            (8,  130.0, 150.0, 190.0, 310.0, 0.85, 9600.0), # Person B crossed left
        ]
        matched_clear, state_clear = self.mgr.update(cands_separated, now=30.7)
        self.assertIsNotNone(matched_clear)
        self.assertEqual(matched_clear[0], 11) # Correctly reacquired Target A under track 11
        self.assertEqual(state_clear, STATE_TRACKING)

    # ------------------------------------------------------------------
    # TEST 5: Manual Clear
    # Operator explicitly clears target (req_id = -1).
    # EXPECTED: target_handle invalidated; state = NO_TARGET.
    # Only then may a new selection occur.
    # ------------------------------------------------------------------
    def test_05_manual_clear(self):
        # Lock Target A
        self.mgr.select_target(track_id=5, bbox=(100, 100, 150, 250), now=40.0)
        self.assertIsNotNone(self.mgr.target_handle)
        self.assertEqual(self.mgr.target_handle.handle_id, "TARGET_001")

        # Operator sends manual clear (e.g. -1)
        self.mgr.clear_target(reason='operator_clear')
        self.assertIsNone(self.mgr.target_handle)

        # Further updates with candidates present do NOT track anyone
        cands = [(5, 100, 100, 150, 250, 0.9, 7500.0)]
        matched, state = self.mgr.update(cands, now=40.1)
        self.assertIsNone(matched)
        self.assertEqual(state, STATE_NO_TARGET)

        # Now operator explicitly locks Person B (track_id = 9)
        handle2 = self.mgr.select_target(track_id=9, bbox=(200, 200, 260, 350), now=40.5)
        self.assertIsNotNone(handle2)
        self.assertEqual(handle2.handle_id, "TARGET_002") # Fresh handle counter
        self.assertEqual(handle2.current_track_id, 9)

    # ------------------------------------------------------------------
    # TEST 6: No Target (Safe Standby)
    # No target selected.
    # EXPECTED: NO_TARGET, safe non-pursuit behavior.
    # ------------------------------------------------------------------
    def test_06_no_target(self):
        self.assertIsNone(self.mgr.target_handle)

        # Multiple pedestrians in frame, but operator has not selected any
        cands = [
            (1, 100, 100, 150, 250, 0.85, 7500.0),
            (2, 200, 100, 250, 250, 0.92, 7500.0),
            (3, 300, 100, 350, 250, 0.78, 7500.0),
        ]
        matched, state = self.mgr.update(cands, now=50.0)

        self.assertIsNone(matched)
        self.assertEqual(state, STATE_NO_TARGET)
    # ------------------------------------------------------------------
    # TEST 7: Target Handle Telemetry Serialization
    # Ensures the telemetry dictionary is valid JSON and contains all required
    # observability fields for downstream HUD and Ground Station subscribers.
    # ------------------------------------------------------------------
    def test_07_telemetry_serialization(self):
        handle = self.mgr.select_target(track_id=42, bbox=(100.0, 100.0, 160.0, 260.0), conf=0.95, now=60.0)
        data = handle.to_dict()
        import json
        dumped = json.dumps(data)
        loaded = json.loads(dumped)
        self.assertEqual(loaded['handle_id'], "TARGET_001")
        self.assertEqual(loaded['current_track_id'], 42)
        self.assertEqual(loaded['state'], STATE_TARGET_LOCKED)
        self.assertEqual(loaded['lock_mode'], 'MANUAL')
        self.assertIn('velocity_2d', loaded)
        self.assertIn('previous_track_ids', loaded)


if __name__ == '__main__':
    unittest.main()

