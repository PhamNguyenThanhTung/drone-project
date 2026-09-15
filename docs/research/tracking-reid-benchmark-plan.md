# Tracking and Re-ID Benchmark Plan

This is an experiment design only. It does not modify production code or prescribe a deployment change.

## Objectives

Measure whether Re-ID improves end-to-end target following enough to justify its latency and failure modes for a moving UAV camera. Separate three questions:

1. Does the MOT tracker maintain trajectories?
2. Can the target manager keep the selected logical target?
3. Can an appearance model recover that target after lineage is lost?

## Systems under comparison

| ID | Pipeline | Purpose |
|---|---|---|
| A | Detector + tracker only | Baseline association |
| B | A + motion prediction | Quantify short-gap prediction |
| C | A + target lock | Prevent accidental target changes |
| D | A + target lock + continuous Re-ID | Upper compute/appearance comparison; run at a declared cadence |
| E | A + target lock + motion prediction + Re-ID on demand | Recommended candidate architecture |
| F (optional) | BoT-SORT with GMC + optional Re-ID | Moving-camera tracker comparison |
| G (offline upper bound) | StrongSORT-style gallery and interpolation | Robustness ceiling, not an edge deployment target |

Use the same detector weights, input resolution, confidence thresholds, hardware, and frame timestamps across systems. Compare ByteTrack and BoT-SORT separately; changing detector and tracker simultaneously confounds results.

## Datasets and scenario construction

Use public data for reproducibility and project-specific sequences for relevance:

- **MOT17/MOT20:** fixed-camera pedestrian MOT; MOT20 stresses crowding. Download from MOTChallenge.
- **UAVDT:** aerial detection/tracking with UAV viewpoint and camera motion [11].
- **VisDrone-MOT:** diverse drone altitude, scale, and density [12].
- **UAV123:** long UAV tracking sequences and camera motion [13].
- **Project SITL/video:** controlled target paths, gimbal turns, occluders, identical clothing, and known 3-D ground truth.

Annotate every sequence with: target logical identity, all visible people, occlusion interval, entry/exit times, camera angular velocity, target world position/velocity, and whether automatic search is mission-authorized.

Required scenario strata:

1. target always visible;
2. 0.2–0.5 s occlusion;
3. 1–2 s occlusion;
4. tree/large-object occlusion;
5. target cut off by another person;
6. FOV exit and re-entry;
7. many people running in the same direction;
8. similar clothing;
9. sharp drone/gimbal turn;
10. sudden target direction change.

Report at least five independent clips per stratum, with disjoint identities between train/tuning/test. For Re-ID, split by person identity and location to prevent gallery leakage.

## Metrics

### MOT-level

- **HOTA:** primary balanced detection-association-localization metric [7].
- **IDF1:** identity precision/recall over trajectories.
- **MOTA:** legacy aggregate detection metric; report but do not use alone.
- **ID switches (IDSW):** count identity discontinuities.
- Mostly-tracked / mostly-lost and fragmentation.

### Target-level

Define the selected target as a logical identity, not the tracker ID:

- target loss rate and mean time to loss;
- target-track continuity and tracking duration;
- target switch rate and **false target switch** rate (controller follows a non-target person);
- recovery rate after occlusion/re-entry;
- false recovery rate (wrong person accepted as target);
- recovery latency from first visible target frame to confirmed reacquisition;
- reject/hold rate when no safe candidate exists.

Use event-level confidence intervals (bootstrap by clip) rather than only frame averages. A single false target switch should be weighted more heavily in a safety review than several extra ID switches among non-target people.

### Re-ID-level

On a standard gallery/probe protocol report Rank-1, Rank-5, mAP, and ROC/TPR at a fixed false-accept rate. Also report performance stratified by altitude, crop height, occlusion, blur, illumination, viewpoint, and clothing similarity. These metrics answer representation quality; they do not prove end-to-end tracking benefit.

### Systems metrics

Measure end-to-end camera-to-decision latency (p50/p95/p99), detector/tracker/Re-ID component latency, FPS and frame drops, CPU utilization, GPU utilization, VRAM peak, power if available, and thermal throttling. Pin the process and record hardware/software versions. Re-ID-on-demand must report calls per minute and percentage of frames invoking it.

## Protocol

1. Run detector-only to establish recall and latency.
2. Run A–G with identical recorded frames and deterministic seeds where supported.
3. For each system, calibrate thresholds on a tuning split only. Freeze them before test evaluation.
4. Evaluate both offline replay and closed-loop SITL. Offline replay isolates perception; SITL captures gimbal/vehicle feedback and target leaving the FOV.
5. Add controlled sensor delay/drop/blur and camera-motion perturbations.
6. Repeat each clip three times for stochastic GPU/runtime variance; report mean and 95% bootstrap intervals.
7. Record every state transition, candidate score, gate rejection, Re-ID invocation, accepted recovery, and safety action in a machine-readable event log.

### Ablation matrix

| Ablation | Question |
|---|---|
| no GMC vs GMC | Does ego-motion compensation help UAV camera motion? |
| IoU-only vs center+Mahalanobis | Which motion gate is stable under scale change? |
| 2-D only vs 2-D+3-D | How many ambiguous candidates can geometry reject? |
| no appearance vs appearance only in ambiguity | Is event-triggered Re-ID sufficient? |
| gallery fixed vs EMA/recent gallery | Does adaptation improve viewpoint robustness or cause drift? |
| K=1 vs K=3/5 confirmation frames | Safety/latency tradeoff for reacquisition |
| continuous Re-ID cadence 1, 3, 5, 10 frames | Compute versus identity benefit |

## Acceptance criteria (suggested, project to approve)

The architecture should not be selected by a single leaderboard score. Suggested gates for a safety-oriented prototype:

- zero false target switches in all nominal single-target clips;
- no automatic switch when candidate scores are within the configured ambiguity margin;
- >=99% recovery for 0.2–0.5 s occlusions without Re-ID;
- measured improvement from Re-ID on 1–2 s/tree/FOV-reentry strata with false-recovery rate below the mission limit;
- p95 perception-to-control latency within the existing controller freshness budget;
- no unacceptable CPU/VRAM headroom loss under the chosen hardware mode.

The exact percentages must be agreed with the flight-safety owner; they are not claimed results.

## Interpreting outcomes

- If C matches E on nominal and short-occlusion strata, keep Re-ID dormant there.
- If E materially lowers false target switches or improves FOV re-entry with acceptable latency, deploy Re-ID on demand.
- If Re-ID improves Rank-1/mAP but worsens false recovery in UAV clips, do not enable it automatically; improve data/domain adaptation first.
- If BoT-SORT's GMC reduces ID switches under sharp turns without significant latency, prefer it over ByteTrack for moving-camera operation.

## References

See the primary-source list in [tracking-vs-target-selection-vs-reid.md](tracking-vs-target-selection-vs-reid.md), especially HOTA [7], UAVDT [11], VisDrone [12], and UAV123 [13].
