# Align SDK IK Model To VR Implementation Plan

Status: Superseded.

This plan described vendoring the robot IK URDF into the SDK and running
client-side IK against that local model. The SDK architecture has since moved
IK solving to the robot ROS side:

- `Arm.ik()` publishes a pose command.
- `bw_core::ArmCommandResolverNode` owns IK model loading and solving.
- The SDK package no longer ships local IK URDF or mesh assets.

Keep this note only to explain why the older vendored-model plan should not be
implemented again.
