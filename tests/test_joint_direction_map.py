import importlib.util
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
CONSTANTS_FILE = REPO_ROOT / "mantis" / "constants.py"


def _load_constants_module():
    spec = importlib.util.spec_from_file_location("mantis_constants", CONSTANTS_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_sdk_joint_direction_map_matches_current_runtime_convention():
    constants = _load_constants_module()

    assert constants.JOINT_DIRECTION_MAP == {
        "left_shoulder_pitch_joint": -1,
        "left_shoulder_yaw_joint": 1,
        "left_shoulder_roll_joint": -1,
        "left_elbow_pitch_joint": 1,
        "left_wrist_roll_joint": 1,
        "left_wrist_pitch_joint": -1,
        "left_wrist_yaw_joint": 1,
        "right_shoulder_pitch_joint": -1,
        "right_shoulder_yaw_joint": -1,
        "right_shoulder_roll_joint": 1,
        "right_elbow_pitch_joint": 1,
        "right_wrist_roll_joint": -1,
        "right_wrist_pitch_joint": -1,
        "right_wrist_yaw_joint": -1,
    }
