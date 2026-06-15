"""Legacy SDK IK module.

`Arm.ik()` now publishes pose commands to the robot ROS side, where IK is
solved by `bw_core::ArmCommandResolverNode`. This module remains only so old
imports fail with a clear migration message instead of importing Pinocchio or
CasADi on the SDK client.
"""


class MantisArmIK:
    """Compatibility placeholder for the removed client-side IK solver."""

    def __init__(self, *args, **kwargs):
        raise RuntimeError(
            "SDK client-side IK has moved to the robot ROS side. "
            "Use Mantis.left_arm.ik()/right_arm.ik(), which publishes sdk/arm_command."
        )
