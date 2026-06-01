# Idea 1 Brainstorm — Hierarchical VLA-planner + IBVS Primitives

**Parent idea:** Promote OpenVLA (or smaller VLM) to planner; let `visual_servo` execute via skill-call schema.
**Date:** 2026-04-26
**Direction:** Wrist-cam-only low-cost mobile manipulator (Piper + D405). Brainstorm 12 concrete sub-research-questions, filter, rank top 3 worth pursuing.

---

## Phase 1: Landscape — closest prior art (scite)

| Work | Year | What it does | Gap vs this repo |
|---|---|---|---|
| Code-as-Policies (Liang et al.) | 2023 | LLM emits Python over fixed skill API | No closed-loop IBVS feedback; tabletop only |
| VoxPoser (Huang et al.) | 2023 | LLM → 3D affordance map → motion plan | Open-loop trajectory; no wrist cam |
| OK-Robot, DovSG | 2024 | VLM planner → primitives on mobile robot | Third-person cam; learned grasp net not IBVS |
| EmbodiedCoder (arxiv:2510.06207) | 2025 | Modern coding model → parameterized mobile manip | Geometric primitives only; no closed-loop visual feedback |
| GraSP-VLA (arxiv:2511.04357) | 2025 | Scene-graph orchestrator over VLA primitives | VLA primitives, not classical IBVS |
| VAMOS (arxiv:2510.20818) | 2025 | Hierarchical VLA navigation, decouple planner from embodiment | Navigation, not manipulation |
| RoboHiMan (arxiv:2510.13149) | 2025 | Benchmark hierarchical comp-generalization | Diffusion-policy primitives, Franka |
| GRACE (arxiv:2510.07975) | 2025 | VLM → Executable Analytic Concepts → motion | Articulated objects; no wrist cam IBVS |
| Kinodynamic-TAMP+VLM (Kwon & Kim) | 2025 | VLM guides motion sampler | TAMP planner, not closed-loop control |
| LITEN (arxiv:2510.19752) | 2025 | VLM affordance learning at inference time | Per-task in-context, no servo backend |
| LIRA (Nat. Comms Chem 2025) | 2025 | VLM error inspection in SDL | Open-loop chemistry workflow |

**Structural gap.** No published hierarchical-VLA system uses a **classical closed-loop IBVS controller with a documented recovery state machine** (LOST/STALL/BLIND_PUSH_TIMEOUT) as the low-level primitive backend on a **wrist-camera-only low-cost mobile manipulator**. This is exactly what `src/manipulation_perception/visual_servo` already provides.

---

## Phase 2: 12 Generated Sub-ideas

### A. Skill-call schema design study
Compare Code-as-Policies (Python) vs JSON tool-call vs ROS-action-goal as planner→executor protocol on identical Piper+IBVS stack. Measure SR, latency, planner hallucination rate, recoverability.
- **Risk:** LOW. **Effort:** 1 week. **Type:** diagnostic.

### B. Wrist-cam-only small-VLM planner
Position paper: show small VLM (Qwen2-VL-2B / SmolVLM-256M) on wrist cam alone routes correctly when IBVS handles alignment. Tests whether ego-centric clutter degrades planning vs third-person.
- **Risk:** MEDIUM. **Effort:** 2 weeks. **Type:** empirical.

### C. IBVS-as-affordance verifier (gating)
Before executing `pick(target_class)`, query IBVS-reachability check: candidate target visible + centerable + depth-in-band reachable from current EEF pose. Reject hallucinated targets at planning time.
- **Risk:** LOW. **Effort:** 1.5 weeks. **Type:** method.

### D. Closed-loop replan on IBVS failure signals
Subscribe to `visual_servo/state`. On LOST/STALL/BLIND_PUSH_TIMEOUT, snapshot RGBD + state log, ask VLM "what failed, what next", emit revised skill plan. Generalizes LITEN/LIRA error-monitor to wrist-cam mobile manip.
- **Risk:** LOW–MEDIUM. **Effort:** 2 weeks. **Type:** method.

### E. VLA-as-skill-of-last-resort + utilization study
Diagnostic: %time across N tasks where (i) classical IBVS pick suffices, (ii) navigation+IBVS suffices, (iii) genuine VLA delta-EEF needed. Strong negative result is publishable: if 95% of tabletop tasks need zero VLA, the "VLA-as-controller" hype is misallocated for this hardware class.
- **Risk:** LOW. **Effort:** 2 weeks. **Type:** diagnostic / position.

### F. Mobile-base aware skill grammar
Add `nav_then_pick(landmark, target)` and `approach_to_servo(pose_hint)`. VLM decomposes "fetch cup from kitchen" → nav + IBVS pick. Compare against MoManipVLA end-to-end.
- **Risk:** MEDIUM. **Effort:** 3 weeks. **Type:** empirical.

### G. Few-shot skill-arg grounding via in-context demos
Provide 3–5 (image, instruction, skill_call) examples in VLM prompt. Measure SR jump on novel objects vs zero-shot. No training.
- **Risk:** LOW. **Effort:** 1 week. **Type:** method.

### H. On-device vs cloud VLM planner latency study
Profile GPT-4o vs Qwen2-VL-7B on Jetson AGX Orin. Repo already runs OpenVLA remote at 0.5 Hz. Show on-device VLM planner at 1 Hz suffices because IBVS at 20 Hz absorbs latency. Systems contribution.
- **Risk:** LOW. **Effort:** 1.5 weeks. **Type:** systems / empirical.

### I. Failure-mode taxonomy benchmark for wrist-cam mobile manip
Adapt RoboHiMan to wrist-cam IBVS+VLM stack on Piper. Public dataset of failure clips labelled (planner-error / perception-error / IBVS-error). Benchmark contribution.
- **Risk:** MEDIUM (data labour). **Effort:** 4 weeks. **Type:** benchmark.

### J. Zero-shot novel skill via Code-as-Policies
When skill missing, VLM writes ROS Python over `servo_twist_pub` + `gripper_command`. Largely subsumed by Code-as-Policies / RoboCodeX / EmbodiedCoder.
- **Risk:** MEDIUM. **Type:** method. **Verdict: KILL — not novel.**

### K. Hybrid grasp localization: VLM coarse pixel + IBVS refine
VLM emits coarse 2D pixel + approach direction from RGB; IBVS uses that as the seed target instead of detector class match. Tests whether VLM **spatial picker** is good enough even when VLM **controller** is not. Resolves the central tension in this repo.
- **Risk:** LOW. **Effort:** 2 weeks. **Type:** empirical / method.

### L. Skill-schema as ROS interface contract
Engineering: define `manipulation_msgs/SkillCall.msg` + `SkillFeedback.msg`, unify visual_servo + nav + gripper under one action server, open-source release.
- **Risk:** LOW. **Effort:** 1.5 weeks. **Type:** systems / tooling. **Verdict: necessary infra but low research novelty — fold into chosen idea.**

---

## Phase 3: First-pass filter

| Idea | Verdict | Reason |
|---|---|---|
| J | KILL | Code-as-Policies clone |
| L | FOLD INTO 1 | infra prereq |
| A | KEEP (low priority) | nice diagnostic, narrow |
| B | KEEP | clean position paper |
| C | KEEP | direct novelty |
| D | STRONG KEEP | best paper angle |
| E | STRONG KEEP | strong diagnostic / position |
| F | KEEP | extends to mobile, large effort |
| G | KEEP (small) | quick win |
| H | KEEP | systems flavored |
| I | KEEP | benchmark contribution |
| K | STRONG KEEP | resolves repo tension cleanly |

---

## Phase 4: Deep validation — top 3 picks

### 🏆 Pick 1: **D — Closed-loop replan on IBVS failure signals**

**Hypothesis.** A VLM that observes the IBVS failure state machine (LOST, STALL, BLIND_PUSH_TIMEOUT) plus an RGBD snapshot can emit recovery skill plans that lift end-to-end pick SR by ≥20 pp over the current "abort + return-to-ready" policy. Failure handling is the dominant residual error mode once basic IBVS works.

**Minimum experiment.** 30 tabletop tasks × 3 perturbations (occluder placed, target rotated, target nudged mid-grasp). Two conditions: (i) baseline IBVS abort-and-retry; (ii) VLM replan on failure event. Metric: SR @ 3 attempts, recovery time.

**Novelty.** LITEN does inference-time affordance learning, LIRA does VLM error inspection in chemistry, but no work uses the **structured ROS state stream of a classical IBVS controller** as the failure-detection signal that gates VLM replanning. Closest: GraSP-VLA (uses scene graph not control-state). **Novelty 8/10.**

**Reviewer's likely objection.** "Failure recovery results dominated by manual prompt engineering." Mitigation: ablation with fixed prompt template, varying only state snapshot.

**Pilot.** No GPU train. ~2 h robot time. Confirm before launch.

**Predicted score:** 8/10. Strong RA-L / IROS fit.

---

### 🥈 Pick 2: **K — VLM-coarse-pixel + IBVS-refine grasp localization**

**Hypothesis.** VLM is a strong **target picker** (semantic segmentation + spatial reasoning) but a weak **controller** (precise EEF deltas). Splitting the responsibility — VLM emits target pixel + approach hint, IBVS handles pixel-to-grasp closed-loop — outperforms both end-to-end OpenVLA and detector-only IBVS on cluttered scenes with ambiguous targets ("the *empty* cup", "the cup *behind* the bowl").

**Minimum experiment.** 5 cluttered tabletop scenes × 4 referring expressions each = 20 tasks. Three arms: (i) detector + IBVS (current baseline, requires class prompt); (ii) OpenVLA end-to-end; (iii) VLM coarse-pixel + IBVS refine. Metric: target-selection accuracy + grasp SR.

**Novelty.** GRACE emits affordance blueprints but not a per-pixel servoing seed. EmbodiedCoder fits geometric primitives. **The VLM-pixel → IBVS-seed loop is undocumented for wrist-cam D405 close-range.** Novelty 7.5/10.

**Reviewer's likely objection.** "Just a Pixel-VLA / SAM2 + IBVS combo." Mitigation: include PixelVLA (arxiv:2511.01571) and SAM2-mask + IBVS as baselines.

**Pilot.** No train. ~3 h robot time.

**Predicted score:** 7.5/10. Best ICRA fit.

---

### 🥉 Pick 3: **E — VLA utilization diagnostic + position paper**

**Hypothesis.** On low-cost wrist-cam mobile manipulators, ≥85% of standard household pick-and-place tasks decompose into primitives that classical IBVS executes more reliably than any current VLA. The VLA-as-controller paradigm is mis-deployed for this hardware class.

**Minimum experiment.** Curate 50 tasks across {tabletop, shelf, container, low surface, base-relocation}. For each, label which primitive sequence solves it. Run all three conditions: (a) IBVS-only with VLM target picker; (b) VLA end-to-end; (c) VLA-as-fallback only. Report SR and per-task primitive coverage.

**Novelty.** Nobody has measured the actual VLA need-rate on a low-cost mobile platform. Most papers assume VLA-as-controller is the right abstraction. **A strong negative result here re-frames a sub-field.** Novelty 7/10 (position paper depth).

**Reviewer's likely objection.** "Task selection bias." Mitigation: open dataset + community-contributed task list.

**Pilot.** ~6 h robot time across multiple sessions.

**Predicted score:** 7/10. Workshop or CoRL "Out of Distribution" track.

---

## Phase 5: Pilots — SKIPPED

All three picks need real-robot time (2–6 h) not GPU time. **Need user confirmation before booking robot.** No autonomous pilot launched.

---

## Phase 6: Recommended Execution Order

| Order | Idea | Why |
|---|---|---|
| 1 | **L** (skill-schema infra) | Prereq for everything else. 1.5 weeks. |
| 2 | **K** (VLM-pixel + IBVS-refine) | Cleanest experimental story. Validates that hierarchical split works at all. |
| 3 | **D** (replan on IBVS failure) | Builds on K's stack. Highest paper score. |
| 4 | **E** (VLA utilization study) | Once K and D are running, the 50-task sweep is a 1-week add-on. |

Single 6-month research arc lands 1 strong paper (D) + 1 short paper / RA-L (K) + 1 position paper (E) — and ships a usable system.

---

## Eliminated Ideas

| Idea | Reason |
|---|---|
| J — Code-as-Policies-style snippet emission | Subsumed by Code-as-Policies / EmbodiedCoder / RoboCodeX |
| Train custom hierarchical VLA from scratch | Compute infeasible; gains likely marginal vs frozen-backbone hybrid |
| Replace IBVS with diffusion policy primitives | Discards what already works; orthogonal to direction |
| End-to-end whole-body VLA on Piper+base | MoManipVLA already exists; not novel for this class |

---

## Next Step

Pick **K or D** (or both) and invoke:

```
/research-refine-pipeline "<chosen sub-idea + repo constraints>"
```

to produce `refine-logs/FINAL_PROPOSAL.md` and `refine-logs/EXPERIMENT_PLAN.md`. Confirm robot-time pilot before run.

---

## References (scite-retrieved this session)

- Zheng, L., Cui, R., Chen, H., et al. (2025). *EmbodiedCoder.* arXiv. https://doi.org/10.48550/arxiv.2510.06207
- Neau, M., Falomir, Z., Santos, P. E. (2025). *GraSP-VLA.* arXiv. https://doi.org/10.48550/arxiv.2511.04357
- Castro, M.G., et al. (2025). *VAMOS.* arXiv. https://doi.org/10.48550/arxiv.2510.20818
- Chen, Y., et al. (2025). *RoboHiMan.* arXiv. https://doi.org/10.48550/arxiv.2510.13149
- Sun, M., Wei, J., He, Q., et al. (2025). *GRACE: Executable Analytic Concepts.* arXiv. https://doi.org/10.48550/arxiv.2510.07975
- Kwon, M., Kim, Y. J. (2025). *Kinodynamic TAMP via VLM-guided Sampling.* arXiv. https://doi.org/10.48550/arxiv.2510.26139
- Shah, A., Chen, W., et al. (2025). *LITEN.* arXiv. https://doi.org/10.48550/arxiv.2510.19752
- Zhou, Z., Veeramani, S., et al. (2025). *LIRA.* Communications Chemistry. https://doi.org/10.1038/s42004-025-01770-1
- Fu, L., Salimpour, S., et al. (2025). *ROSBag MCP Server.* arXiv. https://doi.org/10.48550/arxiv.2511.03497
- Liang, W., Sun, G., et al. (2025). *PixelVLA.* arXiv. https://doi.org/10.48550/arxiv.2511.01571
- Wu, Z., Zhou, Y., Xu, X., et al. (2025). *MoManipVLA.* CVPR 2025, 1714–1723. https://doi.org/10.1109/cvpr52734.2025.00167
