from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
IK_SOLVER = REPO_ROOT / "mantis" / "ik_solver.py"
ARM = REPO_ROOT / "mantis" / "arm.py"
MANIFEST = REPO_ROOT / "MANIFEST.in"
MODEL_DIR = REPO_ROOT / "mantis" / "model"


def test_sdk_ik_stub_does_not_import_local_ik_dependencies():
    text = IK_SOLVER.read_text(encoding="utf-8")

    assert "import pinocchio" not in text
    assert "import casadi" not in text
    assert "robot ROS side" in text


def test_arm_ik_publishes_pose_command_instead_of_solving_locally():
    text = ARM.read_text(encoding="utf-8")

    assert "_publish_arm_command(command)" in text
    assert "solve_ik_abs" not in text
    assert "solve_ik_rel" not in text
    assert "_get_ik_solver" not in text


def test_sdk_does_not_vendor_local_ik_model_assets():
    assert not MODEL_DIR.exists()

    if MANIFEST.exists():
        manifest = MANIFEST.read_text(encoding="utf-8")
        assert "mantis/model" not in manifest
