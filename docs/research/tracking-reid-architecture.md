# Tracking-First, Re-ID-on-Demand Architecture

## Decision premise

For the current drone mission, the controller follows one operator- or mission-selected person. The safest identity policy is conservative: preserve the logical target handle, predict briefly through gaps, and refuse an automatic switch when evidence is weak. Continuous appearance inference is unnecessary during high-confidence tracking and can add latency and appearance-induced errors.

## Recommended data model

Maintain three IDs:

| ID | Owner | Lifetime | Meaning |
|---|---|---|---|
| `detection_index` | detector | one frame | Array position only |
| `track_id` | MOT tracker | until tracker deletion | Temporary trajectory identity |
| `target_handle` | target manager | mission session | Logical person selected by the mission |

The target manager stores the last confirmed box, velocity, covariance, timestamp, 3-D position (when valid), route constraints, and an optional appearance gallery. The flight controller consumes only a confirmed or explicitly predicted target state; it never chooses a new person directly from detector output.

## State machine

```text
NO_TARGET -> CANDIDATES -> TARGET_SELECTED -> TARGET_LOCKED -> TRACKING
                                                            |
                                                            v
                                                        UNCERTAIN
                                                            |
                                                            v
                                                        RECOVERY
                                                   /          \
                                           REID_VERIFY       TARGET_LOST
                                                   |             |
                                                   v             v
                                              REACQUIRED       SEARCH
                                                   |             |
                                                   +--> TRACKING |
                                                                 v
                                                              NO_TARGET
```

### State contract

| State | Entry condition | Exit condition / timeout | Suggested confidence/quality gate | Safety behavior | Allowed transitions |
|---|---|---|---|---|---|
| `NO_TARGET` | Startup, operator clear, or terminal loss | Candidate list stable for 2–5 frames | None | No autonomous pursuit; hover/mission-safe mode | `CANDIDATES` |
| `CANDIDATES` | One or more valid person detections | Selection score margin and temporal persistence satisfied; 1–3 s acquisition timeout | Detector confidence >= 0.45 and valid crop/box geometry | Do not command aggressive motion; show candidates | `TARGET_SELECTED`, `NO_TARGET` |
| `TARGET_SELECTED` | Operator click, reference match, or mission rule chooses candidate | Same candidate observed with tracker ID for 2–5 frames | Selection score above calibrated threshold and margin >= 0.10 | Lock is provisional; no auto-switch | `TARGET_LOCKED`, `CANDIDATES` |
| `TARGET_LOCKED` | Provisional lock confirmed | First valid state estimate or 1 s confirmation timeout | >= 2 consecutive observations with confidence >= 0.55 | Freeze logical handle; register optional gallery only from quality crops | `TRACKING`, `CANDIDATES` |
| `TRACKING` | Target observation passes motion/geometry gates | Observation quality falls below gate for 2 consecutive frames | Confidence >= 0.45, normalized innovation <= 3 sigma, 3-D gate valid | Controller follows observed state; use GMC/ego-motion compensation | `UNCERTAIN`, `TARGET_LOST` |
| `UNCERTAIN` | Missed/low-quality observation, high innovation, or conflicting candidates | Valid gated observation within 0.3–0.7 s, or uncertainty timeout | Prediction covariance below configured recovery bound | Continue low-speed prediction; cap velocity/yaw; do not switch ID | `TRACKING`, `RECOVERY`, `TARGET_LOST` |
| `RECOVERY` | Track lineage missing but target predicted region remains plausible | Candidate passes motion + 3-D gates for 0.5–2 s | Candidate inside 3 sigma gate and 3-D residual <= configured bound | Search only locally around predicted state; no global target change | `TRACKING`, `REID_VERIFY`, `TARGET_LOST` |
| `REID_VERIFY` | Recovery candidate(s) exist after longer gap or crossing ambiguity | Two or more consistent matches, or 1–3 s verification timeout | Re-ID >= `T_accept`, margin >= configured value, K-frame persistence | Re-ID is a verifier; reject ties and low scores; controller remains safe/slow | `REACQUIRED`, `SEARCH`, `TARGET_LOST` |
| `REACQUIRED` | Candidate matches gallery and motion/geometry history | Confirmed for 2–5 frames | Re-ID and geometry gates remain valid for K frames | Restore normal controller limits gradually | `TRACKING`, `UNCERTAIN` |
| `TARGET_LOST` | No valid observation beyond recovery horizon (suggested 2–4 s) | Operator reset or bounded search timeout (5–15 s) | No accepted candidate | Hover/hold and emit lost event; never silently retarget | `SEARCH`, `NO_TARGET` |
| `SEARCH` | Target lost and mission permits search | Candidate satisfies reference/mission identity, or 5–15 s search expires | Re-ID/selection >= accept threshold plus geometry gate | Slow bounded scan; no pursuit of unverified person | `REID_VERIFY`, `TARGET_SELECTED`, `NO_TARGET` |

Thresholds are starting points, not universal constants. Tune them against the benchmark plan and controller response. Use hysteresis (separate enter/exit thresholds) to avoid state chatter.

## Association pipeline

```text
Frame -> detector -> MOT (BoT-SORT preferred for moving camera; ByteTrack baseline)
                     |
                     +-> ego-motion/GMC compensation
                     +-> Kalman/OC-SORT-style prediction
                     +-> 2-D gate (IoU, center distance, Mahalanobis innovation)
                     +-> 3-D gate (range/ground position/velocity consistency)
                     +-> target lock update
                                      |
                    high confidence -> tracker only
                    uncertain        -> prediction + local candidate gate
                    long loss        -> event-triggered Re-ID + confirmation
```

Use a weighted association cost only after hard gates:

```text
C = w2d * d2d + w3d * d3d + wv * dvelocity + wa * dappearance
```

Set `wa = 0` while the target is confidently observed. Increase it only in `REID_VERIFY`; appearance must not override an impossible 3-D motion or camera-motion model. Reject a candidate when any hard gate fails, regardless of its appearance score.

### 2-D + 3-D consistency

With calibrated intrinsics and an altitude/range estimate, back-project the box center or foot point to a ground-plane ray. Compare candidate displacement with the target's predicted world position and velocity. A candidate with close image coordinates but implausible world range or velocity should be rejected. Depth uncertainty must be propagated: use a Mahalanobis gate, not a fixed pixel threshold. Geometry reduces the number of Re-ID calls; it does not prove identity when people are co-located or the camera pose estimate is poor.

## Re-ID activation policy

### Activate Re-ID when

- the target has been absent longer than the short-term tracker horizon (suggested 0.5–2 s, dependent on FPS and motion);
- a new track appears inside the predicted region but track lineage changed;
- two or more people cross and motion/3-D costs are within an ambiguity margin;
- the target leaves the field of view and re-entry is expected or mission-critical;
- the camera executes a sharp turn or large zoom and GMC/prediction covariance becomes high.

### Do not activate Re-ID when

- a valid target observation passes 2-D, 3-D, and velocity gates;
- only one or two low-confidence detector frames are missing and covariance remains bounded;
- the crop is too small, blurred, truncated, backlit, or heavily occluded for a meaningful embedding;
- the only available candidate fails geometry or has no temporal persistence;
- an operator has issued manual override or target clear.

### Confirmation rule

Use a gallery containing the initial reference plus high-quality, diverse views. For each candidate compute cosine similarity to reference, EMA, and recent gallery entries. Require:

```text
score >= T_accept
score - runner_up >= margin
and candidate is consistent for K consecutive frames
```

Use a lower `T_reject` and an explicit “unknown” band between reject and accept. Never update the gallery from an unverified recovery candidate; only update from high-confidence `TRACKING` observations to avoid identity drift.

## Lost-target timing policy

The tracker should not search for another person immediately after one missed frame. A practical policy for a 20–30 Hz camera is:

| Elapsed loss | Action |
|---|---|
| 0–0.3 s | Predict and keep normal-to-reduced controller limits |
| 0.3–1.5 s | `UNCERTAIN/RECOVERY`; local gate, bounded motion, no target switch |
| 1.5–4 s | `REID_VERIFY` if candidates exist; otherwise hold/search locally |
| 4–15 s | `TARGET_LOST` then bounded search only if mission authorizes it |
| >15 s | Stop autonomous retargeting; require operator or mission reset |

For a 10 Hz camera, scale frame counts rather than blindly using these times. The existing project timeout values should be measured against actual detector latency and gimbal slew capability; a long timeout can preserve the wrong predicted position, while a short timeout causes unsafe switches.

## Safety invariants

1. A new track ID never changes `target_handle` by itself.
2. A reference or Re-ID match cannot override a failed geometry/quality gate.
3. Low-confidence ambiguity results in hold/hover, not target switching.
4. Manual override has priority and disables autonomous reacquisition until cleared.
5. Search is bounded in angle, speed, and time; after expiry the system returns to a safe non-pursuit state.

## Architecture choices for this project

### A — No Re-ID

```text
YOLO -> tracker -> target selection -> target lock -> motion prediction
     -> 3-D geometry -> controller
```

Best for one visible target, short occlusions, predictable motion, and a mission where re-entry is out of scope. Lowest latency and operational complexity. It must fail closed after the recovery timeout.

### B — Re-ID on demand (recommended)

```text
YOLO -> tracker -> selection -> target lock -> normal tracking
                                      |
                                      +-> loss -> prediction/local gate
                                                   -> Re-ID verification
                                                   -> reacquire or safe search
```

Preserves the low cost of tracker-only operation while covering long occlusion, crossings, and re-entry. The main engineering work is state/threshold calibration and a domain-representative gallery.

### C — Person search / appearance-first

```text
reference target -> detector + Re-ID/person-search over every frame
                  -> candidate ranking -> tracker -> recurrent Re-ID recovery
```

Necessary when the mission begins from an image and the target can be anywhere among many people, or when long-range re-entry is a first-class requirement. It has the highest compute and false-match exposure and should be validated on UAV imagery before adoption.

## Sources

The tracker, Re-ID, person-search, HOTA, and UAV dataset sources are listed in [tracking-vs-target-selection-vs-reid.md](tracking-vs-target-selection-vs-reid.md). Core design assumptions follow SORT/DeepSORT [1,2], ByteTrack/BoT-SORT [3,4], StrongSORT/OC-SORT [5,6], and OSNet/FastReID [9,10].
