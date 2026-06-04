# Pick-object API (AI-callable grasping)

Exposes the visual-servo grasp pipeline as a single **`PickObject` ROS 2 action**
(`/pick_object`) plus an optional **MCP tool** so AI agents can grasp an object by
name with one call.

## Architecture

```
AI client ──► pick_object(name)        (MCP tool, optional)
          └─► /pick_object  action      (manipulation_msgs/action/PickObject)
                   │  served by
                   ▼
          pick_orchestrator  (manipulation_policy)
                   │  prompt + gate + watch state/gripper
                   ▼
          visual_servo_node ─► adapter ─► MoveIt/servo ─► arm
```

`pick_orchestrator` is launched automatically by `core_launch.py` in
`control_mode:=visual_servo`.

## Use it directly (ROS / any rclpy or rosbridge client)

```bash
ros2 action send_goal /pick_object manipulation_msgs/action/PickObject \
  "{object: 'bread', timeout_sec: 70}" --feedback
```

Goal: `string object`, `float32 timeout_sec` (<=0 → server default).
Result: `bool success`, `bool object_held`, `geometry_msgs/Pose grasp_pose`,
`float32 gripper_width`, `string message`.
Feedback: `string state`, `float32 gripper_width`.

`object_held` is heuristic (no force sensor): a jaw blocked by an object stops
short of full close. Thin/soft objects can hold while still closing fully — check
`gripper_width` and `message`.

## Use it from an MCP LLM client

Two options:

1. **Existing `ros-mcp-server`** — already has `send_action_goal`; point it at
   `/pick_object`. No extra code.
2. **Dedicated tool** `pick_object(name)` — cleaner one-liner for agents. Run
   `pick_object_mcp.py` in a sourced ROS env (needs `pip install "mcp[cli]"`):

   ```jsonc
   // .mcp.json
   {
     "mcpServers": {
       "robot-pick": {
         "command": "bash",
         "args": ["-lc",
           "source /opt/ros/humble/setup.bash && source ~/codes/MobileManipulationCore/install/setup.bash && python3 ~/codes/MobileManipulationCore/deployment/mcp/pick_object_mcp.py"]
       }
     }
   }
   ```

   Then the agent calls `pick_object("toy banana")` → `{success, object_held,
   gripper_width, message}`.

## Notes

- The grasp pipeline + robot bringup must be running first.
- Per-object grasp offsets and the detection prompt vocabulary are in
  `config/visual_servo_params.yaml` (`grasp_offsets`) and
  `config/detection_params.yaml`. Toy fruits need a `toy <name>` label.
