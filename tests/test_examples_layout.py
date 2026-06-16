from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
EXAMPLES_DIR = REPO_ROOT / "examples"

ROOT_EXAMPLE_FILES = {
    "coffee copy.ipynb",
    "coffee copy.py",
    "coffee.ipynb",
    "coffee.py",
    "measure_frequency.py",
    "test_chassis.py",
    "test_discover.py",
    "test_get_ip.py",
    "test_gripper.py",
    "test_ik.ipynb",
    "test_ik.py",
    "test_parallel_motion.py",
    "test_real.py",
    "test_sim.py",
    "test_sn_read.py",
    "test_waist.py",
    "test_waist_bend_3_0.py",
}

EXPECTED_EXAMPLES = {
    "examples/common.py",
    "examples/basic/connection_example.py",
    "examples/basic/status_subscription_example.py",
    "examples/discovery/discovery_example.py",
    "examples/discovery/sn_topic_diagnostic.py",
    "examples/arm/joint_control_example.py",
    "examples/arm/ik_pose_example.py",
    "examples/gripper/gripper_example.py",
    "examples/head/head_example.py",
    "examples/waist/waist_height_example.py",
    "examples/waist/waist_bend_3_0_example.py",
    "examples/chassis/chassis_example.py",
    "examples/workflows/parallel_motion_example.py",
    "examples/workflows/coffee_workflow_example.py",
}

CAPABILITY_KEYWORDS = {
    "connection",
    "discovery",
    "status",
    "arm",
    "ik",
    "gripper",
    "head",
    "waist",
    "chassis",
    "parallel",
    "workflow",
}


def test_root_does_not_contain_runnable_example_scripts():
    leftovers = [name for name in ROOT_EXAMPLE_FILES if (REPO_ROOT / name).exists()]

    assert leftovers == []


def test_examples_cover_public_sdk_capabilities():
    missing = [path for path in EXPECTED_EXAMPLES if not (REPO_ROOT / path).exists()]

    assert missing == []

    readme = (EXAMPLES_DIR / "README.md").read_text(encoding="utf-8").lower()
    missing_keywords = [word for word in CAPABILITY_KEYWORDS if word not in readme]

    assert missing_keywords == []


def test_examples_keep_python_3_8_argparse_compatibility():
    example_sources = [
        path.read_text(encoding="utf-8")
        for path in EXAMPLES_DIR.rglob("*.py")
    ]

    assert all("BooleanOptionalAction" not in text for text in example_sources)


def test_examples_do_not_keep_legacy_notebooks_or_hard_coded_targets():
    assert list(EXAMPLES_DIR.rglob("*.ipynb")) == []

    example_sources = [
        path.read_text(encoding="utf-8")
        for path in EXAMPLES_DIR.rglob("*.py")
    ]
    forbidden = [
        "192.168.1.111",
        "192.168.1.151",
        "BW_30ORGLJM",
        "while 1",
        "while True",
        "sim=True",
    ]

    for text in example_sources:
        assert all(pattern not in text for pattern in forbidden)
