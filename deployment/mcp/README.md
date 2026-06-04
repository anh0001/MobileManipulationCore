# Robot skills API (AI-callable manipulation)

Exposes every high-level robot skill (pick, home, ...) through **one generic
`ExecuteSkill` ROS 2 action** (`/execute_skill`) plus an **MCP server** that turns
each registered skill into its own typed LLM tool. AI agents call a skill by name
with one call; adding a skill needs no new message, node, launch edit, or MCP edit.

## Architecture

```
AI client ──► <skill>(args...)         (one MCP tool per skill, auto-generated)
          └─► /execute_skill  action    (manipulation_msgs/action/ExecuteSkill)
                   │  skill name + params_json
                   ▼
          skill_server  (manipulation_policy)
                   │  registry lookup + validate + dispatch
                   ▼
          Skill.execute(ctx, ...)  ──► visual_servo / arm / MoveIt ──► robot
```

`skill_server` is launched automatically by `core_launch.py` in
`control_mode:=visual_servo`.

## Skills

Defined in `src/manipulation_policy/manipulation_policy/skills/`:

- **`pick`** — grasp an object by open-vocabulary name. Args: `object` (required),
  `timeout_sec`. Outputs: `success`, `object_held`, `gripper_width`, `message`.
  `object_held` is heuristic (no force sensor): a jaw blocked by an object stops
  short of full close. Thin/soft objects can hold while still closing fully — check
  `gripper_width` and `message`.
- **`home`** — move the arm to a named pose. Args: `pose` (`ready`/`rest`),
  `time_sec`.

## Use it directly (ROS / any rclpy or rosbridge client)

```bash
ros2 action send_goal /execute_skill manipulation_msgs/action/ExecuteSkill \
  "{skill: 'pick', params_json: '{\"object\": \"bread\", \"timeout_sec\": 70}'}" --feedback
```

Goal: `string skill`, `string params_json` (JSON object of the skill's args).
Result: `bool success`, `string message`, `string result_json` (skill outputs).
Feedback: `string state`, `float32 progress`.

## Use it from an MCP LLM client

Run `skill_mcp.py` in a sourced ROS env (needs `pip install "mcp[cli]"`). It reads
the skill registry and registers one tool per skill automatically:

```jsonc
// .mcp.json
{
  "mcpServers": {
    "robot-skills": {
      "command": "bash",
      "args": ["-lc",
        "source /opt/ros/humble/setup.bash && source ~/codes/MobileManipulationCore/install/setup.bash && python3 ~/codes/MobileManipulationCore/deployment/mcp/skill_mcp.py"]
    }
  }
}
```

The agent then calls e.g. `pick(object="toy banana")` or `home(pose="rest")` →
`{success, message, ...skill outputs}`.

(The existing `ros-mcp-server` `send_action_goal` also works — point it at
`/execute_skill` — but the dedicated per-skill tools are cleaner for agents.)

## Add a new skill

1. Create `skills/<name>_skill.py` with a `Skill` subclass: set `name`,
   `description`, `params`, implement `execute(ctx, params, feedback, is_cancelled)`,
   decorate with `@register_skill`.
2. Import it in `skills/__init__.py`.

No `.action` file, no `colcon build` of `manipulation_msgs`, no launch change, no
MCP change. The skill is dispatchable on `/execute_skill` and appears as an MCP
tool on the next `skill_mcp.py` start. Shared robot plumbing (prompt publish,
remote params, arm move, live state) is on the `SkillContext` passed to `execute`.

## Notes

- The grasp pipeline + robot bringup must be running first.
- Per-object grasp offsets and the detection prompt vocabulary are in
  `config/visual_servo_params.yaml` (`grasp_offsets`) and
  `config/detection_params.yaml`. Toy fruits need a `toy <name>` label.
