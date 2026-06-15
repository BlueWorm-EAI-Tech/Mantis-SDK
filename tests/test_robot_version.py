import json
import math
from pathlib import Path
import sys
import time

import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from mantis import Mantis
from mantis.constants import (
    URDF_ARM_JOINT_NAMES,
)


class _FakePublisher:
    def __init__(self):
        self.messages = []

    def put(self, payload):
        self.messages.append(payload)


def _make_connected_robot(robot_version="3.0"):
    robot = Mantis(robot_version=robot_version)
    robot._connected = True
    return robot


def test_mantis_uses_2_0_as_default_robot_version():
    robot = Mantis()

    assert robot._robot_version == "2.0"


def test_mantis_accepts_robot_version_3_0():
    robot = Mantis(robot_version="3.0")

    assert robot._robot_version == "3.0"


def test_mantis_does_not_expose_joint_direction_reference():
    robot_v2 = Mantis()
    robot_v3 = Mantis(robot_version="3.0")

    assert not hasattr(robot_v2, "joint_direction_map")
    assert not hasattr(robot_v3, "joint_direction_map")


def test_mantis_rejects_unknown_robot_version():
    with pytest.raises(ValueError, match="robot_version"):
        Mantis(robot_version="4.0")


def test_publish_full_state_uses_raw_urdf_arm_angles():
    robot = Mantis()
    publisher = _FakePublisher()
    robot._publishers["joints"] = publisher
    robot.left_arm._positions = [0.1, 0.2, 0.3, -0.4, 0.5, -0.6, 0.7]
    robot.right_arm._positions = [-0.8, 0.9, -1.0, 1.1, -1.2, 1.3, -1.4]

    robot._publish_full_state()

    assert len(publisher.messages) == 1
    msg = json.loads(publisher.messages[0].decode("utf-8"))
    assert msg["name"][: len(URDF_ARM_JOINT_NAMES)] == list(URDF_ARM_JOINT_NAMES)
    assert msg["position"][: len(URDF_ARM_JOINT_NAMES)] == pytest.approx(
        robot.left_arm.positions + robot.right_arm.positions
    )


def test_arm_ik_absolute_control_publishes_robot_side_pose_command():
    robot = _make_connected_robot(robot_version="3.0")
    publisher = _FakePublisher()
    robot._publishers["arm_command"] = publisher

    robot.left_arm.ik(0.1, 0.2, 0.3, 0.0, 0.0, 0.0, block=False, abs=True)

    assert not hasattr(robot, "_ik_solver_instance")
    assert len(publisher.messages) == 1
    msg = json.loads(publisher.messages[0].decode("utf-8"))
    assert msg == {
        "command_type": "pose_abs",
        "side": "left",
        "pose": {
            "x": pytest.approx(0.1),
            "y": pytest.approx(0.2),
            "z": pytest.approx(0.3),
            "roll": pytest.approx(0.0),
            "pitch": pytest.approx(0.0),
            "yaw": pytest.approx(0.0),
        },
    }


def test_arm_ik_relative_control_publishes_robot_side_delta_command():
    robot = _make_connected_robot(robot_version="3.0")
    publisher = _FakePublisher()
    robot._publishers["arm_command"] = publisher

    robot.right_arm.ik(0.01, -0.02, 0.03, 0.1, -0.2, 0.3, block=False, abs=False)

    assert not hasattr(robot, "_ik_solver_instance")
    assert len(publisher.messages) == 1
    msg = json.loads(publisher.messages[0].decode("utf-8"))
    assert msg == {
        "command_type": "pose_rel",
        "side": "right",
        "delta": [
            pytest.approx(0.01),
            pytest.approx(-0.02),
            pytest.approx(0.03),
            pytest.approx(0.1),
            pytest.approx(-0.2),
            pytest.approx(0.3),
        ],
    }


def test_arm_ik_block_true_waits_without_requiring_solver_ack(monkeypatch):
    robot = _make_connected_robot(robot_version="3.0")
    robot._publishers["arm_command"] = _FakePublisher()
    waited = []

    monkeypatch.setattr(robot.left_arm, "wait", lambda: waited.append(True))

    robot.left_arm.ik(0.01, 0.0, 0.0, 0.0, 0.0, 0.0, block=True, abs=False)

    assert waited == [True]


def test_manual_joint_control_does_not_initialize_local_ik_solver():
    robot = _make_connected_robot(robot_version="3.0")
    robot._publish_full_state = lambda: None

    robot.left_arm.set_joint(0, 0.2, block=False)

    assert not hasattr(robot, "_ik_solver_instance")


def test_direct_arm_joint_control_stays_available_for_robot_version_3_0():
    robot = Mantis(robot_version="3.0")
    robot._connected = True
    robot._publish_full_state = lambda: None

    robot.left_arm.set_joint(0, 0.3, block=False)
    robot.right_arm.set_joints([0.1] * 7, block=False)

    assert robot.left_arm.positions[0] == 0.3
    assert robot.right_arm.positions == [0.1] * 7
    assert not hasattr(robot, "_ik_solver_instance")


def test_waist_bend_angle_control_is_available_for_robot_version_3_0():
    robot = Mantis(robot_version="3.0")
    robot._connected = True
    publisher = _FakePublisher()
    robot._publishers["waist_angle"] = publisher

    robot.waist.set_bend_speed(0.25)
    robot.waist.set_bend(-0.4, block=False)

    assert robot.waist.bend_angle == pytest.approx(-0.4)
    assert len(publisher.messages) == 1
    msg = json.loads(publisher.messages[0].decode("utf-8"))
    assert msg == {"angle": pytest.approx(-0.4), "max_velocity": pytest.approx(0.25)}


def test_waist_bend_direction_helpers_match_forward_backward_semantics():
    robot = Mantis(robot_version="3.0")
    robot._connected = True
    publisher = _FakePublisher()
    robot._publishers["waist_angle"] = publisher

    robot.waist.bend_forward(0.6, block=False)
    robot.waist.bend_backward(0.25, block=False)
    robot.waist.set_bend(0.0, block=False)

    messages = [json.loads(payload.decode("utf-8")) for payload in publisher.messages]
    assert messages == [
        {"angle": pytest.approx(-0.6), "max_velocity": pytest.approx(math.radians(20.0))},
        {
            "angle": pytest.approx(math.radians(5.0)),
            "max_velocity": pytest.approx(math.radians(20.0)),
        },
        {"angle": pytest.approx(0.0), "max_velocity": pytest.approx(math.radians(20.0))},
    ]


def test_waist_height_publishes_v3_pelvis_height_position_command():
    robot = Mantis(robot_version="3.0")
    robot._connected = True
    robot._publishers["joints"] = _FakePublisher()
    robot._publishers["pelvis_height"] = _FakePublisher()

    robot.waist.set_speed(0.12)
    robot.waist.set_height(0.05, block=False)

    messages = [
        json.loads(payload.decode("utf-8"))
        for payload in robot._publishers["pelvis_height"].messages
    ]
    assert messages == [{"height": pytest.approx(0.05), "max_velocity": pytest.approx(0.12)}]


def test_waist_bend_control_is_not_available_for_robot_version_2_0():
    robot = Mantis()
    robot._connected = True

    with pytest.raises(NotImplementedError, match="弯腰"):
        robot.waist.set_bend(0.5)


def test_waist_home_resets_bend_angle_for_robot_version_3_0():
    robot = Mantis(robot_version="3.0")
    robot._connected = True
    robot._publishers["joints"] = _FakePublisher()
    robot._publishers["pelvis_height"] = _FakePublisher()
    robot._publishers["waist_angle"] = _FakePublisher()
    robot._publish_full_state = lambda: None

    robot.waist.set_bend(-0.8, block=False)
    robot.waist.home(block=False)

    messages = [
        json.loads(payload.decode("utf-8"))
        for payload in robot._publishers["waist_angle"].messages
    ]
    assert messages[-1] == {
        "angle": pytest.approx(0.0),
        "max_velocity": pytest.approx(math.radians(20.0)),
    }
    assert robot.waist.bend_angle == pytest.approx(0.0)


def test_wait_requires_fresh_stopped_status_samples_before_returning(monkeypatch):
    robot = Mantis(robot_version="3.0")
    robot._connected = True
    robot._last_status_update_time = 0.0
    robot._system_status = {
        "motion_names": ["waist"],
        "motion_states": [0],
    }

    time_calls = []
    wait_calls = []

    def fake_time():
        time_calls.append(True)
        return 0.0 if len(time_calls) == 1 else 2.0

    def fake_sleep(duration):
        wait_calls.append(duration)
        if len(wait_calls) > 20:
            raise TimeoutError("wait kept polling for fresh stopped samples")

    monkeypatch.setattr(time, "time", fake_time)
    monkeypatch.setattr(time, "sleep", fake_sleep)

    with pytest.raises(TimeoutError):
        robot.wait(["waist"])
