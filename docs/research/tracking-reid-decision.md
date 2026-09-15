# Tracking / Target Selection / Re-ID Decision

## Executive answer

Yes, for the current scope it is technically defensible to omit Re-ID during normal operation when there is one selected target, short occlusion, predictable motion, a strong moving-camera tracker, target lock, and valid 3-D/geometric constraints. This is a conditional result, not a claim that Re-ID is useless. Re-ID becomes necessary when the identity gap exceeds the tracker/prediction horizon, the target exits and re-enters, multiple people are plausible at the predicted location, or the mission requires identity continuity through long/crowded occlusion.

The recommended architecture is **tracking-first, Re-ID-on-demand**: run the tracker every frame, preserve a logical target lock, use motion and 2-D/3-D gates for short gaps, and invoke appearance matching only for ambiguous or long-loss recovery. This minimizes latency and avoids making a noisy appearance model the authority during normal visual servoing.

## Decision matrix

Ratings are relative engineering judgments supported by the cited tracker designs; they are not project measurements.

| Method | Tracking quality | Target selection | Occlusion recovery | Long-term identity | Latency | VRAM | CPU | Complexity | UAV suitability | Safety |
|---|---|---|---|---|---|---|---|---|---|---|
| ByteTrack only | High for clean/short gaps; no appearance | Separate policy required | Short gaps via low-score detections; weak long gaps | Low | Lowest | Low | Low | Low | Very good baseline | Safe only with target lock + no auto-switch |
| BoT-SORT | High; GMC helps moving camera | Separate policy required | Better at camera motion/crossings; optional Re-ID | Medium/high when Re-ID enabled | Low/medium | Medium | Medium | Medium | Best tracker-first choice for UAV | Good with conservative gates |
| StrongSORT | Very high on difficult MOT sequences | Separate policy required | Strong long/fragmented recovery | High | Highest | Medium/high | High | High | High robustness, edge cost risk | Good only with strict reject policy |
| ByteTrack + motion prediction | High for predictable target and short loss | Separate policy required | Better local recovery | Low/medium | Low | Low | Low | Low/medium | Excellent for current scope | Strong if it fails closed |
| ByteTrack + Re-ID | High plus appearance recovery | Can support reference selection | Better long gap; false match risk | Medium/high | Medium/high | Medium | Medium | Medium | Useful when re-entry matters | Requires geometry/margin gates |
| BoT-SORT + Re-ID | High under camera motion and identity ambiguity | Separate policy or reference ranking | Best practical balance | High | Medium/high | Medium/high | Medium/high | Medium/high | Best high-robustness candidate | Safe only with unknown/reject state |
| Person-search pipeline | Detector+identity search, not just tracking | Strong for reference-specified target | Can search beyond tracker lineage | High if domain matched | Highest | High | High | Highest | Needed for many-person reference search | Highest false-accept exposure; operator confirmation preferred |

## Scenario analysis

| Scenario | Tracker alone sufficient? | Motion prediction sufficient? | Target lock sufficient? | Re-ID benefit | Policy |
|---|---|---|---|---|---|
| 1. Target always visible | Usually yes if detections and association are stable | Helpful for smoothing, not identity | Yes, essential to prevent opportunistic switching | Little; continuous Re-ID adds cost | Tracker + lock; no Re-ID |
| 2. Occlusion 0.2–0.5 s | Often yes with low-score association and GMC | Usually yes | Yes, if the predicted gate is tight | Marginal; use only if crossing ambiguity exists | Prediction first; Re-ID dormant |
| 3. Occlusion 1–2 s | Sometimes; depends on FPS, camera motion, and crowd | Often only partly; covariance grows | Preserves intent but cannot observe identity | Material benefit for candidate verification | Enter recovery; event-trigger Re-ID |
| 4. Target behind a tree | No guarantee; tracker may delete track | Short-term only; tree duration is unknown | Prevents switching but cannot reacquire globally | Strong benefit when target reappears among candidates | Hold/search locally, then Re-ID; reject uncertain matches |
| 5. Target cut off by another person | Often fails at crossing/merge | May predict through a brief cut-off | Prevents immediate switch | Useful if appearance remains reliable; geometry still gates | Do not switch on track-ID change; verify over K frames |
| 6. Target leaves FOV and returns | No; no observation lineage | No after prediction horizon | Lock states which identity is wanted, not where it is | Necessary for long-range re-entry unless external localization is available | Bounded search + Re-ID or operator reacquisition |
| 7. Many people running same direction | Motion-only can swap identities | Weak discriminator because velocities are similar | Prevents intentional retargeting but not association swaps | Helps, though similar crops can still confuse | BoT-SORT/GMC + Re-ID only on ambiguity; large reject margin |
| 8. Same clothes | Tracker can work while visible | Often sufficient for short gaps | Helps preserve target intent | Appearance benefit is low; false matches risk is high | Favor geometry/history; require multi-cue confirmation |
| 9. Drone turns sharply | Motion-only tracker may fail due to global image motion | Ego-motion-aware prediction required | Lock alone cannot compensate camera transform | Re-ID can recover after GMC failure, not replace GMC | Use GMC/BoT-SORT; invoke Re-ID after uncertainty |
| 10. Target changes direction suddenly | Constant-velocity prediction can lag | Short-term prediction may be wrong | Lock prevents target switching but can point stale | Usually little benefit if target remains visible; appearance does not predict motion | Use observation-centric/multi-model motion and cautious controller limits |

The key distinction is **observability**. Re-ID helps when a person is visible again but tracker lineage is missing; it cannot identify an invisible person and cannot rescue a poor detector or invalid camera pose.

## When a sufficiently good tracker makes Re-ID unnecessary

Re-ID can be omitted for a mission segment when all of the following are true:

1. Exactly one target is selected and a logical target lock exists.
2. The target remains within the camera's recoverable FOV.
3. Expected occlusions are shorter than the tracker's validated prediction horizon.
4. Camera motion is compensated (GMC or a calibrated ego-motion model).
5. 2-D and 3-D position/velocity gates reject nearby distractors.
6. The target's motion is not intentionally indistinguishable from a group for longer than the recovery timeout.
7. The controller fails closed: hold/hover/search, never silent retargeting.
8. Benchmark results show no unacceptable false target switches in the relevant scenarios.

Under these constraints, Re-ID is redundant during normal tracking and can be disabled to save latency, VRAM, and power.

## When Re-ID is necessary or strongly justified

- target exits the FOV and returns;
- long tree/building occlusion or deleted tracks;
- multiple plausible people occupy the prediction gate;
- group crossings where motion is not discriminative;
- mission starts from a reference image and the target is one of many detections (one-shot person search);
- the operator requires identity continuity beyond a tracker session;
- external localization is unavailable and a wrong reacquisition is worse than a safe hold.

Even then, Re-ID is evidence, not authority. Combine it with motion, 3-D consistency, crop quality, temporal persistence, and an explicit reject/unknown class.

## Recommended current-project architecture

```text
YOLO person detections
  -> BoT-SORT with GMC (evaluate against current ByteTrack baseline)
  -> candidate filtering and target-selection policy
  -> logical target lock (independent of tracker ID)
  -> 2-D + 3-D geometry + velocity consistency
  -> controller
       TRACKING: tracker only
       UNCERTAIN: prediction and bounded local recovery
       long loss/ambiguity: Re-ID verification
       no accepted match: hold/search, never auto-retarget
```

### Target selection recommendation

Use manual selection as the highest-authority path for the current project. For automatic selection, use a multi-criteria score with hard geometry/quality gates and a temporal margin. Treat a reference image as **reference-guided target selection/person search**, not as ordinary tracking.

### Target lock recommendation

Store `target_handle`, last confirmed track ID, last observation, velocity/covariance, 3-D estimate, and optional gallery. Do not replace the handle when a tracker ID changes. Require explicit operator clear or verified reacquisition to change it.

### Motion and geometry recommendation

Use a Kalman/observation-centric prediction with camera-motion compensation, then combine image-space innovation with calibrated 3-D ground/range consistency. The detector box-area signal is useful as a relative size cue but is not metric depth; the current motion-arbiter pinhole module provides a separate ground-distance estimate whose validity/confidence should be gated and benchmarked. Neither geometry signal is identity proof by itself.

### Re-ID recommendation

Keep the existing lightweight OSNet-style module optional and event-triggered. Build a gallery from high-quality confirmed target views. Do not run it on every frame by default, do not update it from unverified candidates, and do not accept a match without a score margin, multi-frame persistence, and geometry consistency.

## Evidence and limits

The tracker comparisons follow SORT, DeepSORT, ByteTrack, BoT-SORT, StrongSORT, and OC-SORT publications [1–6]. HOTA/IDF1/MOTA should be reported together [7]. OSNet and FastReID support lightweight or configurable appearance encoders [9,10], but public Re-ID benchmarks do not represent UAV viewpoint changes; validate with UAVDT, VisDrone, UAV123, and project sequences [11–13]. No public benchmark can prove safety for this controller without closed-loop SITL/flight testing.

## References

[1] Bewley et al., SORT, ICIP 2016. https://arxiv.org/abs/1602.00763

[2] Wojke et al., DeepSORT, ICIP 2017. https://arxiv.org/abs/1703.07402

[3] Zhang et al., ByteTrack, ECCV 2022. https://arxiv.org/abs/2110.06864

[4] Aharon et al., BoT-SORT, arXiv:2206.14651. https://arxiv.org/abs/2206.14651

[5] Du et al., StrongSORT, arXiv:2202.13514. https://arxiv.org/abs/2202.13514

[6] Cao et al., OC-SORT, CVPR 2023. https://arxiv.org/abs/2203.14360

[7] Luiten et al., HOTA, IJCV 2021. https://arxiv.org/abs/2009.07736

[8] Xiao et al., Person Search, CVPR 2017. https://arxiv.org/abs/1611.05649

[9] Zhou et al., OSNet, ICCV 2019. https://arxiv.org/abs/1905.00953

[10] He et al., FastReID, arXiv:2006.02631. https://arxiv.org/abs/2006.02631

[11] Du et al., UAVDT, ECCV 2018. https://arxiv.org/abs/1804.00518

[12] Zhu et al., VisDrone, ICCV Workshops 2018. https://arxiv.org/abs/1804.07437

[13] Mueller et al., UAV123, ECCV Workshops 2016. https://www.video.utoronto.ca/research/uois/uav123/

FINAL RECOMMENDATION

1. Trackers: Use the current ByteTrack path as the measured baseline; evaluate BoT-SORT with global-motion compensation as the preferred moving-UAV tracker. Keep OC-SORT as a motion-only alternative for non-linear target motion. Do not adopt StrongSORT by default because its extra appearance and interpolation cost is not justified until the benchmark shows a need.
2. Target Selection: Separate selection from tracking. Prefer manual operator selection for the current scope; otherwise use reference-guided ranking or a gated multi-criteria score. A frame with 20 people and one reference-specified person is one-shot person search/reference-guided target selection.
3. Target Lock: Introduce a logical `target_handle` independent of transient tracker IDs. Never switch it merely because a track ID changes. Require temporal confirmation and explicit reject/unknown behavior.
4. Motion Prediction: Use GMC/ego-motion compensation, a covariance-aware prediction model, and 2-D plus calibrated 3-D position/velocity gates. Treat detector box area as relative evidence only, and gate the current pinhole-distance estimate by its validity/confidence.
5. Re-ID: Make Re-ID optional and event-triggered, using a lightweight OSNet/FastReID-class encoder only after target loss or association ambiguity. Do not run or update it continuously by default.
6. When Re-ID should activate: After the validated short-term prediction horizon, on long/tree occlusion, FOV re-entry, track-lineage reset, crowded crossing ambiguity, or sharp camera motion that leaves multiple plausible candidates.
7. When Re-ID should NOT activate: During high-confidence visible tracking, brief gaps within the prediction horizon, poor-quality crops, failed geometry gates, manual override, or any case where no candidate has a decisive score margin. Hold/hover instead of switching.
8. Final production architecture: YOLO person detection -> BoT-SORT/GMC evaluated against ByteTrack -> target selection -> logical target lock -> tracker + motion prediction + 2-D/3-D geometry -> controller; on uncertainty, bounded recovery; on long loss/ambiguity, Re-ID verification; on rejection, safe hold/search with no automatic retargeting.
