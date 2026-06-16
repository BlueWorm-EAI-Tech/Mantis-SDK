# Mantis SDK Examples

Run examples from the repository root with `python -m examples.<group>.<script>`.
Motion examples require `--ip` or `--sn`; no example uses a hard-coded robot target.

## Common Options

```bash
python -m examples.basic.connection_example --ip 192.168.1.100
python -m examples.basic.connection_example --sn BW_XXXXXXX --robot-version 3.0
```

- `--ip`: robot IP address
- `--sn`: robot serial number
- `--port`: Zenoh port, default `7447`
- `--robot-version`: `2.0` or `3.0`
- `--verify / --no-verify`: enable or skip connect status verification

## Capability Matrix

| Capability | Example |
| --- | --- |
| connection by IP / SN, `robot_ip`, `robot_sn`, `system_status` | `basic/connection_example.py` |
| status subscription and status frequency | `basic/status_subscription_example.py` |
| RViz preview style arm motion | `basic/rviz_preview_example.py` |
| discovery helpers and `RobotDiscovery` callbacks | `discovery/discovery_example.py` |
| raw Zenoh `sn` topic diagnostic | `discovery/sn_topic_diagnostic.py` |
| arm joint control, limits, speed, `home`, wait, blocking | `arm/joint_control_example.py` |
| robot-side IK absolute and relative pose commands | `arm/ik_pose_example.py` |
| manual named joint pose for 3.0 experiments | `arm/manual_joint_pose_example.py` |
| gripper open, close, half_open, set_position, speed | `gripper/gripper_example.py` |
| head look helpers, set_pose, set_pitch, set_yaw, center | `head/head_example.py` |
| waist height, up, down, move, home | `waist/waist_height_example.py` |
| 3.0 waist bend, bend speed, bend_forward, bend_backward | `waist/waist_bend_3_0_example.py` |
| chassis forward, backward, strafe, turn, move, stop, friction | `chassis/chassis_example.py` |
| parallel / non-blocking motion and `robot.wait()` | `workflows/parallel_motion_example.py` |
| multi-step workflow using arms, grippers, head, and home | `workflows/coffee_workflow_example.py` |

Historical notebook experiments are archived under `archive/notebooks/` and are
not maintained as runnable examples.
