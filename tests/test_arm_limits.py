import importlib.util
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
CONSTANTS_FILE = REPO_ROOT / "mantis" / "constants.py"


def _load_constants_module():
    spec = importlib.util.spec_from_file_location("mantis_constants", CONSTANTS_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_sdk_arm_limits_match_formal_runtime_urdf_ranges():
    constants = _load_constants_module()

    expected_limits = [
        (-1.13, 1.75),
        (-0.213, 2.029),
        (-0.8, 0.82),
        (-0.395, 1.012),
        (-1.7, 1.7),
        (-0.562, 0.562),
        (-1.7, 1.7),
    ]

    assert constants.LEFT_ARM_LIMITS == expected_limits
    assert constants.RIGHT_ARM_LIMITS == expected_limits
