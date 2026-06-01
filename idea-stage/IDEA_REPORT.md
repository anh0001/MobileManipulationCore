# Idea Discovery Report — MobileManipulationCore

**Direction:** Visual servo grasp works; VLA (OpenVLA-7B remote, Bridge-Orig) underperforms. Find best ideas to pursue.
**Date:** 2026-04-26
**Pipeline:** scite literature survey → idea generation → novelty filter → critical review (no GPU pilots — user approval needed before training runs)

---

## Executive Summary

Repo today: classical IBVS pipeline (`visual_servo`) reliably picks a centered object using D405 wrist depth, with a **blind-push fallback** when D405 depth dies under 0.088 m. OpenVLA-7B inference runs remotely (no fine-tune, Bridge-Orig action space ≠ Piper embodiment). Three concrete failure causes for VLA: (a) embodiment + camera-view mismatch (Bridge dataset uses third-person WidowX, repo uses wrist D405 on Piper); (b) no closed-loop residual — open-loop deltas at ~0.5 Hz cannot recover sub-cm errors; (c) close-range RGB has no geometry cue exactly where IBVS already needs the blind-push hack.

**Recommended ranking:**
1. 🏆 **Hierarchical VLA-planner over existing IBVS skill primitives** — low risk, immediate win, reuses everything that works.
2. **Residual depth-conditioned servo head over frozen OpenVLA (PLD-style)** — research-novel, directly attacks the close-range failure mode.
3. **IBVS-bootstrapped self-fine-tune of OpenVLA-OFT on Piper** — uses the working IBVS as automatic teacher; Hi-ORS shows 1.5 h is enough.

Backup ideas (4–8) cover test-time scaling, depth aux supervision, mobile-base coordination, error-monitor VLM, and Bridge-V2 virtual-frame transfer (already half-wired in `robot_params.yaml`).

---

## Literature Landscape (scite, 2024–2026)

### Active themes
- **Residual RL on frozen VLA**: PLD/Probe-Learn-Distill (arxiv:2511.00091) — train tiny residual actor where base VLA fails, distill back. 99% LIBERO, 100% real Franka.
- **Inference-time policy steering, no FT**: VLA-Pilot (2511.14178), MG-Select (2510.05681, +28% real-world), SITCOM (2510.04041, MPC rollouts +24 pts).
- **Hierarchical VLA + skills**: RoboHiMan (2510.13149) benchmark shows end-to-end VLAs lose to high-level planner + low-level skill composition. EmbodiedCoder (2510.06207), GraSP-VLA (2511.04357), VAMOS (2510.20818) all decouple semantic plan from embodiment.
- **Depth-aware VLA**: QDepth-VLA (2510.14836) auxiliary depth supervision; Spatial Forcing (2510.12276) implicit 3D alignment without sensors. Both report large SR gains for fine-grained tasks.
- **Embodiment transfer / FT efficiency**: ET-VLA (2511.01224, +53% real), MAPS (2511.19878, +30% OOD, no extra params), MetaVLA (2510.05580, 76% less compute), Hi-ORS (2510.26406, 1.5 h HITL on π0).
- **Mobile manipulation**: MoManipVLA (CVPR 2025, 1714–1723) — first to transfer fixed-base VLA to mobile platform.
- **Reactivity for chunked policies**: TAS (2511.04421, +73% SR) — selector net resolves chunk lag, exactly the latency gap in our remote-inference setup.

### Structural gaps (what nobody has done well yet, and matter for THIS repo)
- **Wrist-camera-only close-range VLA on low-cost arms (Piper, SO-ARM)** — most published work is third-person/Franka.
- **VLA + classical IBVS hybrid** — almost nothing. Surveys flag IBVS as a fundamental skill but VLA literature ignores it.
- **D405 (or similar short-range RGBD) blind-zone handling** — no published method addresses the <8 cm depth dropout other than ad-hoc dead-reckoning (what this repo does).

---

## Ranked Ideas

### 🏆 Idea 1: Hierarchical VLA-planner + classical IBVS primitives (RECOMMENDED)

**Hypothesis.** The repo's IBVS already handles the hard low-level part well. A VLA used as a control policy (delta EEF at 0.5 Hz) is the wrong abstraction for this hardware. Promote OpenVLA (or a smaller VLM) to the **planner role** that emits structured calls — `pick(target_class, container)`, `place(...)`, `nav_to(landmark)` — and let `visual_servo` execute. This is the architecture RoboHiMan, EmbodiedCoder, GraSP-VLA, VAMOS converge on.

**Why fits this repo.** `manipulation_msgs/PolicyOutput` already supports structured outputs. `task_prompt_cli.py` already takes language. Adapter already routes between MoveIt-servo, gripper, navigation. Need: a thin "skill-call" message + a VLM (Qwen2-VL-7B or GPT-4o) emitting JSON skill plans.

**Pilot (≤2 h, no train).** Hand-script 8 tabletop tasks ("pick red cup, place on tray"). Replace `policy_node` with VLM client emitting skill JSON. Compare SR vs current OpenVLA-as-controller baseline.

**Novelty.** Hierarchical-VLA on **wrist-cam-only low-cost mobile manipulator with proven IBVS primitive** — published HMs (RoboHiMan, EmbodiedCoder) all use third-person + Franka/UR5.

**Risk.** Loss of generality if skill library too narrow. Mitigate: keep OpenVLA available as "skill of last resort" for novel motions.

**Reviewer score (predicted, GPT-5.4 standards):** 7.5/10 — solid systems contribution, novelty mostly in deployment study not algorithm.

---

### Idea 2: Residual depth-conditioned servo head over frozen OpenVLA

**Hypothesis.** OpenVLA gives roughly-right delta poses; precise close-range alignment needs a **small residual policy conditioned on depth + tracker error** that takes over when base VLA action would miss. Mirrors PLD (2511.00091) but specialized to the D405 blind-zone problem.

**Architecture.** Frozen OpenVLA → base action `a_base`. Small ResNet-MLP head reads (D405 depth patch around centroid, current pixel error from `manipulation/target_detections`, last `a_base`) → residual `δa`. Final `a = a_base + δa`. Train residual via off-policy RL with simple reward = (depth approached + centroid centered + grasp closed at correct band).

**Why fits this repo.** All the inputs already published on ROS topics. Reward signal trivially derivable from `visual_servo/state`. Tiny model (≤5 M params) runs on Jetson at 20 Hz, so closes the 0.5 Hz remote-inference reactivity gap.

**Pilot.** ~2 h GPU on a workstation, IBVS itself provides bootstrap demos.

**Novelty.** PLD is general; this is **PLD specialized for short-range RGBD blind-zone recovery on a low-cost arm** — a concrete, named failure mode the literature has not addressed.

**Risk.** Reward shaping for "grasp band 0.080–0.110 m" needs care.

**Reviewer score (predicted):** 8/10 — clear failure-mode narrative, reproducible setup, residual-RL is on-trend.

---

### Idea 3: IBVS-bootstrapped self-fine-tune of OpenVLA-OFT for Piper

**Hypothesis.** Use the working IBVS as an **automatic teacher**: log every successful pick (RGB, depth, joint state, achieved EEF trajectory, language prompt) into a Piper-on-Bridge-frame dataset, then LoRA-FT OpenVLA-OFT (Kim et al. 2025). Hi-ORS (2510.26406) shows 1.5 h real-world FT lifts π0 from struggling to 100% on contact-rich tasks. ET-VLA (2511.01224) shows synthetic continued pretraining bridges embodiments.

**Why fits this repo.** No teleop rig needed — IBVS does the demos. `bridge_v2_virtual_frames` block in `robot_params.yaml` is **already wired** to emulate Bridge-V2 camera conditioning (pose `(-0.18, 0, 0.50)`, pitch -45°). Flip `enabled: true` and you get Bridge-style inputs from the wrist cam.

**Pilot.** Collect 200 IBVS picks (~3 h robot time), LoRA-FT (4 h, single A100 via the remote server already used).

**Novelty.** Self-supervised demos generated by classical controller, fed into OpenVLA-OFT, evaluated on the same low-cost arm. The Bridge-V2 virtual-frame trick as a deployment lever is undocumented in the literature.

**Risk.** IBVS-only demos miss long-tail (cluttered scenes, novel objects). Mitigate: mix in 50 hand-collected diverse demos.

**Reviewer score (predicted):** 7/10 — solid empirical, novelty modest, but the self-bootstrap angle is publishable as a workshop paper.

---

### Idea 4: Test-time action sampling + IBVS-scored selection (BACKUP, no train)

Sample N=8 candidate action chunks from remote OpenVLA per step, score each by **predicted alignment with IBVS objective** (project candidate EEF delta into image-plane error reduction + depth-band approach). Pick best. MG-Select (+28% real) and SITCOM (+24 pts SIMPLER) use VLM/value-net scoring; using IBVS as a free, deterministic, physics-grounded scorer is novel. Zero training. Cost: 8× inference, mitigated by batched call to remote server.

**Reviewer score:** 6.5/10. Cheap, fast paper, narrower contribution.

---

### Idea 5: Action-chunking reactivity layer (TAS-style) (BACKUP)

Repo's remote OpenVLA runs at 0.5 Hz, IBVS runs at 20 Hz, gap is filled by adapter dead-reckoning. Add TAS-like (2511.04421) selector that caches multiple policy-emitted chunks and picks the per-step best action via depth-error feedback. Direct latency win for the deployment configuration.

---

### Idea 6: QDepth/Spatial-Forcing aux supervision in fine-tune (BACKUP)

When fine-tuning (Idea 3), add depth-token prediction head (QDepth-VLA, 2510.14836) so the model internalizes the geometry the wrist RGB hides at close range. Cheap addition to Idea 3. Could be the ablation that makes the headline number.

---

### Idea 7: VLM error monitor + recovery (BACKUP, ops-flavored)

Wrap the `visual_servo/state` topic. On `BLIND_PUSH_TIMEOUT`, `LOST`, `STALL` — snapshot RGBD, ask a VLM "did the gripper miss? where? what to retry?", emit recovery action. LITEN (2510.19752) and Code-as-Monitor pattern. Engineering, not novel — but useful product fit.

---

### Idea 8: Whole-body base+arm coordination via VLA hint (BACKUP)

`policy_params.yaml` has `use_base_hint: false`. Activate base hint, train/eval VLA-emitted base motion during reach (MoManipVLA, 2503.13446 / CVPR 2025). Higher engineering cost (Nav2 integration), but only published mobile-VLA result targets a different platform.

---

## Eliminated Ideas

- **Train OpenVLA from scratch on Piper** — compute infeasible, MetaVLA/MAPS findings show fine-tune is what matters anyway.
- **Replace OpenVLA with diffusion policy from scratch** — out of scope, repo invests in VLA path; ManiDP/CFG-DP only relevant if going bimanual.
- **Build new gripper / vacuum hardware (VacuumVLA 2511.21557)** — orthogonal to the VLA-not-working problem.
- **Humanoid whole-body (HumanoidExo, EgoMI)** — wrong embodiment.

---

## Recommended Next Step

**Pick Idea 1 first** as the 2-week sprint: it costs no training, ships immediate SR improvement, and produces the skill-call interface that **Ideas 2, 3, 5, 7 all build on**. After Idea 1 ships, run Idea 3 (self-FT bootstrap) in parallel with Idea 2 (residual head) — they are complementary, not competing.

If you want a paper rather than a product win first, start with **Idea 2** — clearest novelty story (PLD ⊕ depth-blind-zone ⊕ low-cost arm) and tightest experimental scope.

---

## Refined Proposal

Skipped this session — Phase 4.5 (`/research-refine-pipeline`) needs user direction on which idea to pursue. Once you pick (1, 2, or 3), invoke:

```
/research-refine-pipeline "<chosen idea + this repo's constraints>"
```

to produce `refine-logs/FINAL_PROPOSAL.md` and `refine-logs/EXPERIMENT_PLAN.md`.

---

## Key References (scite-retrieved, 2024–2026)

- Xiao, W., Lin, H., Peng, A., et al. (2025). *Self-Improving Vision-Language-Action Models with Data Generation via Residual RL* (PLD). arXiv. https://doi.org/10.48550/arxiv.2511.00091
- Kim, M.J., Pertsch, K., et al. (2025). *OpenVLA-OFT.* (cited via MetaVLA, MAPS).
- Lu, G., Zhao, R., Lin, H., et al. (2025). *Human-in-the-loop Online Rejection Sampling for Robotic Manipulation* (Hi-ORS). arXiv. https://doi.org/10.48550/arxiv.2510.26406
- Chen, Y., Chen, Z., et al. (2025). *RoboHiMan: A Hierarchical Evaluation Paradigm for Compositional Generalization in Long-Horizon Manipulation.* arXiv. https://doi.org/10.48550/arxiv.2510.13149
- Zheng, L., Cui, R., Chen, H., et al. (2025). *EmbodiedCoder: Parameterized Embodied Mobile Manipulation via Modern Coding Model.* arXiv. https://doi.org/10.48550/arxiv.2510.06207
- Neau, M., Falomir, Z., Santos, P. E. (2025). *GraSP-VLA: Graph-based Symbolic Action Representation for Long-Horizon Planning with VLA Policies.* arXiv. https://doi.org/10.48550/arxiv.2511.04357
- Wu, Z., Zhou, Y., Xu, X., et al. (2025). *MoManipVLA: Transferring Vision-language-action Models for General Mobile Manipulation.* CVPR 2025, 1714–1723. https://doi.org/10.1109/cvpr52734.2025.00167
- Li, Y., Chen, Y., Zhou, M., et al. (2025). *QDepth-VLA: Quantized Depth Prediction as Auxiliary Supervision for Vision-Language-Action Models.* arXiv. https://doi.org/10.48550/arxiv.2510.14836
- Li, F., Song, W., Zhao, H., et al. (2025). *Spatial Forcing: Implicit Spatial Representation Alignment for Vision-Language-Action Model.* arXiv. https://doi.org/10.48550/arxiv.2510.12276
- Saxena, A., Shah, H., Routray, S., et al. (2025). *SITCOM: Scaling Inference-Time COMpute for VLAs.* arXiv. https://doi.org/10.48550/arxiv.2510.04041
- Jang, S.-P., Kim, D., Kim, C.-Y., et al. (2025). *Verifier-free Test-Time Sampling for Vision Language Action Models* (MG-Select). arXiv. https://doi.org/10.48550/arxiv.2510.05681
- Li, Z., Liu, J., Dong, Z., et al. (2025). *Towards Deploying VLA without Fine-Tuning: VLA-Pilot.* arXiv. https://doi.org/10.48550/arxiv.2511.14178
- Weng, Y., Zhang, X., Mu, Y., et al. (2025). *Temporal Action Selection for Action Chunking* (TAS). arXiv. https://doi.org/10.48550/arxiv.2511.04421
- Li, C., Peng, Y. (2025). *Embodiment Transfer Learning for Vision-Language-Action Models* (ET-VLA). arXiv. https://doi.org/10.48550/arxiv.2511.01224
- Huang, C., Zhang, M.M., Azarcon, R., et al. (2025). *MAPS: Module-Wise Proximity Scheduling for VLA generalization.* arXiv. https://doi.org/10.48550/arxiv.2511.19878
- Li, C., Yang, Z., Zhang, H., et al. (2025). *MetaVLA: Unified Meta Co-training For Efficient Embodied Adaption.* arXiv. https://doi.org/10.48550/arxiv.2510.05580
- Castro, M.G., Rajagopal, S., Gorbatov, D., et al. (2025). *VAMOS: A Hierarchical VLA Model for Capability-Modulated Navigation.* arXiv. https://doi.org/10.48550/arxiv.2510.20818
- Shah, A., Chen, W., Godbole, A., et al. (2025). *Learning Affordances at Inference-Time for VLA Models* (LITEN). arXiv. https://doi.org/10.48550/arxiv.2510.19752
- Poria, S., Majumder, N., Hung, C.-Y., et al. (2025). *10 Open Challenges Steering the Future of VLA Models.* arXiv. https://doi.org/10.48550/arxiv.2511.05936
