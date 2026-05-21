# Candidate Poses

## left_right_alignment_candidate_02

right:

```text
dx=+0.140
dy=+0.000
dz=-0.050
wrist_yaw=-0.700
wrist_pitch=+0.100
wrist_roll=+0.200
elbow_pitch=+0.250
shoulder_pitch=+0.000
shoulder_roll=+0.700
```

left:

```text
dx=-0.075
dy=-0.120
dz=+0.235
wrist_yaw=-0.300
wrist_pitch=-0.400
wrist_roll=-0.200
elbow_pitch=-0.420
shoulder_pitch=-0.350
shoulder_roll=+0.500
left_gripper=0.700
right_gripper=0.800  # 现场实际夹杯经验值，脚本 show_state 可能显示 ?
```

suggested replay:

```text
replay_right_pour_ready
right_x+ 28
right_z- 10
left_grip
replay_left_move_to_pour_pose_left_only
replay_left_pour_prep_frame
left_x- 15
left_y- 40
left_z+ 47
```
