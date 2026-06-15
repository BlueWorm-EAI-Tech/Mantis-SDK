import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from mantis import Mantis
from mantis.constants import (
    LEFT_ARM_LIMITS,
    RIGHT_ARM_LIMITS,
    SERIAL_TO_URDF_MAP,
    URDF_ARM_JOINT_NAMES,
)


class _FakeIkSolver:
    def __init__(self, q_solution):
        self._joint_names = list(URDF_ARM_JOINT_NAMES)
        self._q_solution = np.array(q_solution, dtype=float)
        self._fk_left = np.eye(4)
        self._fk_right = np.eye(4)
        self._fk_right[:3, 3] = np.array([0.4, -0.5, 0.6])
        self.set_config_calls = []
        self.reset_target_calls = 0
        self.compute_fk_calls = []
        self.solve_ik_abs_calls = []
        self.solve_ik_rel_calls = []

    def get_joint_names(self):
        return list(self._joint_names)

    def set_config(self, q):
        self.set_config_calls.append(list(q))

    def reset_targets(self):
        self.reset_target_calls += 1

    def compute_fk(self, q=None):
        self.compute_fk_calls.append(None if q is None else list(q))
        return self._fk_left.copy(), self._fk_right.copy()

    def solve_ik_abs(self, t_left, t_right):
        self.solve_ik_abs_calls.append((t_left.copy(), t_right.copy()))
        return self._q_solution.copy()

    def solve_ik_rel(self, delta_left, delta_right):
        self.solve_ik_rel_calls.append(
            (np.array(delta_left, copy=True), np.array(delta_right, copy=True))
        )
        return self._q_solution.copy()


class _FakePublisher:
    def __init__(self):
        self.messages = []

    def put(self, payload):
        self.messages.append(payload)


def _make_robot_with_fake_solver(robot_version="3.0"):
    robot = Mantis(robot_version=robot_version)
    robot._connected = True
    robot._publish_full_state = lambda: None
    robot._ik_solver_instance = _FakeIkSolver(
        q_solution=[0.1 * (index + 1) for index in range(len(URDF_ARM_JOINT_NAMES))]
    )
    return robot


def _expected_arm_positions(robot, joint_names):
    solver = robot._ik_solver_instance
    solution_dict = dict(zip(solver.get_joint_names(), solver._q_solution))
    limits = LEFT_ARM_LIMITS if joint_names == robot.left_arm.joint_names else RIGHT_ARM_LIMITS
    expected = []
    for index, name in enumerate(joint_names):
        serial_value = solution_dict[SERIAL_TO_URDF_MAP[name]]
        lower, upper = limits[index]
        expected.append(max(lower, min(upper, serial_value)))
    return expected


def _expected_commanded_joint_config(robot):
    commanded_map = {}
    for index, serial_name in enumerate(robot.left_arm.joint_names):
        commanded_map[SERIAL_TO_URDF_MAP[serial_name]] = robot.left_arm.positions[index]
    for index, serial_name in enumerate(robot.right_arm.joint_names):
        commanded_map[SERIAL_TO_URDF_MAP[serial_name]] = robot.right_arm.positions[index]
    return [commanded_map[name] for name in URDF_ARM_JOINT_NAMES]


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


def test_get_commanded_joint_config_uses_raw_urdf_joint_angles():
    robot = _make_robot_with_fake_solver(robot_version="3.0")
    robot.left_arm._positions = [0.1, -0.2, 0.3, -0.4, 0.5, -0.5, 0.6]
    robot.right_arm._positions = [-0.7, 0.8, -0.8, 0.9, -1.0, 1.1, -1.2]

    assert robot._get_commanded_joint_config() == pytest.approx(
        _expected_commanded_joint_config(robot)
    )


def test_arm_ik_absolute_control_is_available_for_robot_version_3_0():
    robot = _make_robot_with_fake_solver(robot_version="3.0")

    robot.left_arm.ik(0.1, 0.2, 0.3, 0.0, 0.0, 0.0, block=False, abs=True)

    solver = robot._ik_solver_instance
    assert len(solver.solve_ik_abs_calls) == 1
    target_left, target_right = solver.solve_ik_abs_calls[0]
    assert np.allclose(target_left[:3, 3], np.array([0.1, 0.2, 0.3]))
    assert np.allclose(target_right[:3, 3], np.array([0.4, -0.5, 0.6]))
    assert robot.left_arm.positions == _expected_arm_positions(robot, robot.left_arm.joint_names)


def test_arm_ik_relative_control_is_available_for_robot_version_3_0():
    robot = _make_robot_with_fake_solver(robot_version="3.0")

    robot.right_arm.ik(0.01, -0.02, 0.03, 0.1, -0.2, 0.3, block=False, abs=False)

    solver = robot._ik_solver_instance
    assert len(solver.solve_ik_rel_calls) == 1
    delta_left, delta_right = solver.solve_ik_rel_calls[0]
    assert np.allclose(delta_left, np.zeros(6))
    assert np.allclose(delta_right, np.array([0.01, -0.02, 0.03, 0.1, -0.2, 0.3]))
    assert robot.right_arm.positions == _expected_arm_positions(robot, robot.right_arm.joint_names)


def test_arm_ik_does_not_reset_incremental_targets_after_solving():
    robot = _make_robot_with_fake_solver(robot_version="3.0")

    robot.left_arm.ik(0.01, 0.0, 0.0, 0.0, 0.0, 0.0, block=False, abs=False)

    solver = robot._ik_solver_instance
    assert solver.reset_target_calls == 0


def test_manual_joint_control_still_resets_incremental_targets_when_solver_is_active():
    robot = _make_robot_with_fake_solver(robot_version="3.0")

    robot.left_arm.set_joint(0, 0.2, block=False)

    solver = robot._ik_solver_instance
    assert solver.reset_target_calls == 1


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
    robot.waist.set_bend(-0.4)

    assert robot.waist.bend_angle == pytest.approx(-0.4)
    assert len(publisher.messages) == 1
    msg = json.loads(publisher.messages[0].decode("utf-8"))
    assert msg == {"angle": pytest.approx(-0.4), "max_velocity": pytest.approx(0.25)}


def test_waist_bend_direction_helpers_match_forward_backward_semantics():
    robot = Mantis(robot_version="3.0")
    robot._connected = True
    publisher = _FakePublisher()
    robot._publishers["waist_angle"] = publisher

    robot.waist.bend_forward(0.6)
    robot.waist.bend_backward(0.25)
    robot.waist.set_bend(0.0)

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
