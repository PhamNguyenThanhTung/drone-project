# Tracking vs Target Selection vs Re-ID

## Executive distinction

The layers solve different questions and must remain separate in both data contracts and safety policy:

```text
Detection:        Which people are visible in this frame?
Tracking:         Which current detection continues each recent trajectory?
Target selection: Which track should the system follow?
Target identification: Why is that person the requested target?
Re-identification: Is a newly observed person the same previously tracked person?
```

Detection is a per-frame observation. A tracker assigns temporary track IDs and predicts state between observations; an ID is not a human identity. Target selection chooses one (or more) track IDs according to an operator command, reference image, attributes, geometry, or mission rule. Target identification supplies the criterion used by selection. Re-ID compares an observation after an identity gap with a stored appearance representation; it is an identity-recovery operation, not initial selection.

### Why “select initial target” is not “re-identify after loss”

Initial selection has a set of visible candidates and asks `which candidate satisfies the mission?`. It may be a click, nearest-person rule, or reference-image ranking. Re-ID has a previously confirmed target profile and an observation whose tracker lineage is missing or ambiguous; it asks `is this candidate the same target?`. Reusing the same embedding matcher for both is possible, but the error costs differ: an initial false selection is recoverable by an operator, while an unsafe false reacquisition can steer a drone toward the wrong person. Therefore reacquisition needs stricter thresholds, temporal/geometric gates, and a “no switch” outcome.

## Layer relationship

```text
camera frame
   |
   v
person detector ----> detections {box, confidence, class}
   |
   v
multi-object tracker -> tracks {temporary ID, state, velocity, age, covariance}
   |
   +--> target-selection policy -> locked target handle
   |       (operator/reference/attributes/geometry/motion/mission)
   |
   +--> target-identification evidence
   |       (reference embedding, attributes, 3-D position, route)
   |
   +--> controller uses only the locked target state
           |
           +--> if lineage is lost: prediction -> gated candidates -> optional Re-ID
```

The target handle should be a logical ID owned by the target manager, not the detector's transient track ID. A track ID change is an event to evaluate, not permission to change the mission target.

## Tracking families

The table summarizes the primary published designs. Performance depends strongly on detector quality, image size, hardware, and tracker configuration; FPS values should therefore be measured on the project hardware rather than copied from papers.

| Tracker | Association signal | ID stability / occlusion | Camera motion and crossings | Runtime and complexity | UAV assessment |
|---|---|---|---|---|---|
| SORT [1] | Kalman constant-velocity prediction + IoU Hungarian assignment | Very low cost; IDs break on missed detections and long occlusion | No appearance and no camera-motion compensation; weak for moving camera/crossings | CPU-friendly, simple, high FPS | Useful baseline or single-target short-gap tracker, not a crowded-scene default |
| DeepSORT [2] | SORT plus learned appearance metric and cascade matching | Fewer switches through short occlusions and crossings than SORT | Appearance helps, but original design does not model global camera motion | Re-ID inference adds GPU/CPU cost; moderate complexity | Good when identity matters, but appearance model/domain mismatch is a UAV risk |
| ByteTrack [3] | IoU motion association using both high- and low-confidence detections | Recovers true tracks missed by a high threshold; strong IDF1/HOTA on MOT benchmarks | No explicit appearance; camera motion can make IoU gating fail | Very fast, low memory, easy deployment; detector dominates cost | Strong default for real-time UAV when one target is locked and gaps are short |
| BoT-SORT [4] | ByteTrack-style association + camera-motion compensation (GMC) + optional Re-ID | Better IDF1/HOTA under motion and occlusion than motion-only baselines in its evaluations | GMC (homography/affine) directly addresses moving cameras; appearance resolves crossings | More CPU/GPU work and tuning than ByteTrack | Best first upgrade for a moving drone with multiple people |
| StrongSORT [5] | DeepSORT improvements, stronger embedding, EMA feature, AFLink/GSI interpolation | Strong long/fragmented association; interpolation helps gaps | Global motion is not its central contribution; can be expensive for edge UAV | Highest implementation and Re-ID cost in this list | High-robustness reference, but usually excessive for the current single-target SITL scope |
| OC-SORT [6] | Observation-centric motion model; corrects unreliable Kalman updates during occlusion | Better than SORT on non-linear motion and occlusion without mandatory Re-ID | Motion-only; no intrinsic GMC or appearance | Lightweight CPU implementation; low latency | Attractive when people move unpredictably and compute is constrained; combine with ego-motion compensation for UAV |

**Interpretation.** ByteTrack's low-score association is valuable because detector confidence often drops during partial occlusion [3]. BoT-SORT is specifically more suitable than a fixed-camera motion-only tracker when camera motion is material because it estimates global camera motion before association [4]. StrongSORT is a quality ceiling, not automatically the best engineering choice: every Re-ID crop and gallery update consumes compute and can introduce appearance-induced switches. OC-SORT is a useful motion-only alternative when non-linear pedestrian motion is the dominant failure mode [6].

### Tracking-by-detection modes

- **Motion-only association:** predicted box/velocity + IoU or center distance. Deterministic and cheap; identity is fragile when tracks overlap.
- **Appearance-assisted association:** motion gate first, then appearance distance for ambiguous pairs. More stable at crossings, but appearance can be unreliable under top-down views, blur, and similar clothing.
- **Long-term tracking:** maintains a gallery and/or reactivation logic after a track is deleted. This is where Re-ID belongs; it should not be conflated with ordinary frame-to-frame association.
- **Single-target tracking:** only one logical target is controlled. A multi-object tracker can still track all people, but the controller consumes one locked track. A dedicated single-object tracker can be cheaper, but it loses the context needed to reject a nearby distractor.
- **Multi-object tracking:** all visible people receive temporary IDs and trajectories. Required for target switching prevention and candidate ranking.

### Metrics and benchmark interpretation

Use HOTA as the primary balanced metric because it separates detection, association, and localization quality [7]. Report IDF1 and ID switches for identity continuity, and MOTA for legacy comparability. CLEAR-MOT MOTA can hide identity failures when false positives/negatives dominate; no single number is sufficient.

## Target selection and identification

### Selection mechanisms

1. **Manual:** operator clicks a box or sends a track ID. Highest semantic authority; requires UI latency handling and a policy for when that ID expires.
2. **Reference image:** encode one or more target crops, rank visible candidates by embedding similarity, then require a margin over the runner-up and a temporal confirmation window.
3. **Attributes:** color, clothing, backpack, hat, or pose. Useful as a filter or tie-breaker, not a sole identity proof because attributes are coarse and view-dependent.
4. **Geometric/spatial:** closest, largest, image-center, waypoint proximity, or ground-plane distance. Fully explainable and cheap, but can select the wrong person when people are near each other.
5. **Motion/route:** direction, velocity, predicted position, or mission route. Effective when the target has a known route; fails when people stop or turn.
6. **Multi-criteria score:** combine appearance, motion, 2-D/3-D geometry, detector confidence, track age, and operator constraints. Keep hard safety gates separate from the soft score.

A practical score is:

```text
S_select = w_a appearance + w_m motion + w_g geometry
           + w_p position + w_h history + w_o operator
```

Weights should be calibrated on held-out UAV sequences. Never auto-switch solely because another candidate has a higher instantaneous score; require a confidence margin, persistence for N frames, and an explicit target-loss state.

### Person Re-ID vs person search vs target selection

- **Person Re-ID** is the representation/matching problem: given a probe crop and a gallery, rank images of the same person across cameras or time. Standard outputs are Rank-1 and mAP (Market-1501, MSMT17, MARS).
- **Person search** jointly finds a person in an unconstrained scene from a reference image; it combines detection and identification, as in CUHK-SYSU and PRW [8]. The system must search the whole frame or video, not only already-associated tracks.
- **Target selection** is the mission decision layer. It can consume person-search results, but also supports clicks and geometric rules.

“There are 20 people in a frame and one is specified by a reference image” is best called **one-shot person search / reference-guided target selection**. The appropriate pipeline is:

```text
reference image(s) -> target gallery
frame -> person detector -> candidate boxes
candidate crops -> embedding (and optional attributes)
motion/3-D/quality gates -> ranked candidates
temporal confirmation + margin -> logical target lock
tracker -> maintain lock; Re-ID only for ambiguous recovery
```

## Re-ID methods and UAV limitations

| Method | What it contributes | Typical cost/strength | UAV caveat |
|---|---|---|---|
| DeepSORT embedding [2] | Lightweight CNN metric for association | Real-time-oriented; designed for frame-to-frame matching | Trained for surveillance viewpoints; weak at small/top-down crops |
| OSNet [9] | Omni-scale feature representations with strong parameter efficiency | x0.25 is suitable for edge inference; 512-D normalized features are easy to gallery-match | Clothing/color changes, oblique scale, shadows, and tiny crops reduce reliability; fine-tune on UAV data |
| FastReID [10] | Training/evaluation toolbox with strong baselines and deployment options | Broad model choices, metric-learning losses, mixed precision support | Toolbox is not a turnkey UAV model; domain adaptation and calibration remain necessary |
| BoT-SORT Re-ID [4] | Optional appearance branch integrated with tracking and GMC | Helps only when motion association is ambiguous | Running on every detection can dominate latency and create false matches under similar clothing |
| StrongSORT [5] | Stronger embeddings, EMA gallery, AFLink/GSI | High association quality and long-gap recovery | Expensive and complex for embedded UAV; use as an offline upper-bound or high-end mode |
| Lightweight Re-ID (MobileNet/OSNet-tiny/quantized) | Lower latency and VRAM | Suitable for event-triggered inference | Lower discriminability; must use stricter reject thresholds and multi-frame confirmation |

Rank-1/mAP from Market-1501 or MSMT17 must not be treated as UAV performance. UAV imagery changes scale, camera elevation, perspective, blur, and illumination; evaluate on UAVDT/VisDrone/UAV123-style data and project-specific footage [11–13].

## Current project implications

The repository's current vision path uses YOLOv8n with ByteTrack, a logical target ID selected by operator or auto-lock policy, box EMA smoothing, and a seconds-scale target timeout. The detector error contract still contains image-center error plus box area; area is a relative size cue rather than metric depth. The current motion-arbiter changes also include a calibrated 3-D pinhole distance module that publishes ground-distance and geometry-validity telemetry. That geometry should be used as an independent association/recovery signal, with its confidence and calibration error measured before treating it as identity evidence. Existing OSNet code is appropriate as an optional module, not evidence that continuous Re-ID is required.

## References

[1] Bewley et al., “Simple Online and Realtime Tracking (SORT),” ICIP 2016. https://arxiv.org/abs/1602.00763 ; official code: https://github.com/abewley/sort

[2] Wojke, Bewley, Paulus, “Simple Online and Realtime Tracking with a Deep Association Metric (Deep SORT),” ICIP 2017. https://arxiv.org/abs/1703.07402 ; official code: https://github.com/nwojke/deep_sort

[3] Zhang et al., “ByteTrack: Multi-Object Tracking by Associating Every Detection Box,” ECCV 2022. https://arxiv.org/abs/2110.06864 ; official code: https://github.com/ifzhang/ByteTrack

[4] Aharon, Orfaig, Shtok, “BoT-SORT: Robust Associations Multi-Pedestrian Tracking,” arXiv:2206.14651, 2022. https://arxiv.org/abs/2206.14651 ; official code: https://github.com/NirAharon/BoT-SORT

[5] Du et al., “StrongSORT: Make DeepSORT Great Again,” IEEE TPAMI 2023 / arXiv:2202.13514. https://arxiv.org/abs/2202.13514 ; official code: https://github.com/dyhBUPT/StrongSORT

[6] Cao et al., “Observation-Centric SORT: Rethinking SORT for Robust Multi-Object Tracking,” CVPR 2023. https://arxiv.org/abs/2203.14360 ; official code: https://github.com/noahcao/OC_SORT

[7] Luiten et al., “HOTA: A Higher Order Metric for Evaluating Multi-Object Tracking,” IJCV 2021. https://arxiv.org/abs/2009.07736

[8] Xiao et al., “Joint Detection and Identification Feature Learning for Person Search,” CVPR 2017. https://arxiv.org/abs/1611.05649

[9] Zhou et al., “Omni-Scale Feature Learning for Person Re-Identification,” ICCV 2019. https://arxiv.org/abs/1905.00953

[10] He et al., “FastReID: A Pytorch Toolbox for General Instance Re-identification,” arXiv:2006.02631, 2020. https://arxiv.org/abs/2006.02631 ; official repository: https://github.com/JDAI-CV/fast-reid

[11] Du et al., “The Unmanned Aerial Vehicle Benchmark: Object Detection and Tracking,” ECCV 2018. https://arxiv.org/abs/1804.00518

[12] Zhu et al., “Vision Meets Drones: A Challenge,” ICCV Workshops 2018 (VisDrone). https://arxiv.org/abs/1804.07437 ; official benchmark: https://github.com/VisDrone/VisDrone-Dataset

[13] Mueller et al., “A Benchmark and Simulator for UAV Tracking,” ECCV Workshops 2016 (UAV123). https://www.video.utoronto.ca/research/uois/uav123/
