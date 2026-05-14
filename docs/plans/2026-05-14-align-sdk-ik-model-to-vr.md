# Align SDK IK Model To VR Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Make the `mantis` SDK IK solver use the same 2.0 IK URDF model semantics as the VR IK pipeline.

**Architecture:** Vendor the VR IK URDF into the SDK package with local mesh paths, then point the SDK IK solver at that file via a small constant. Add a regression test that compares the vendored SDK IK URDF against the VR source model after normalizing mesh path prefixes.

**Tech Stack:** Python, pytest, URDF

---

### Task 1: Add failing regression test for IK model alignment

**Files:**
- Create: `tests/test_ik_model_alignment.py`
- Modify: `mantis/constants.py`

**Step 1: Write the failing test**

Assert the SDK exports an IK URDF filename constant, the vendored file exists, and its normalized content matches VR's `bw_core/assets/mantis_2_0_ik/mantis_2_0_ik.urdf`.

**Step 2: Run test to verify it fails**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest tests/test_ik_model_alignment.py -q`
Expected: FAIL because the SDK does not yet expose or ship the VR-aligned IK URDF.

### Task 2: Switch SDK IK solver to the VR-aligned IK model

**Files:**
- Create: `mantis/model/urdf/mantis_2_0_ik.urdf`
- Modify: `mantis/constants.py`
- Modify: `mantis/ik_solver.py`

**Step 1: Write minimal implementation**

Add the vendored IK URDF, update the SDK constant, and load that file from `ik_solver.py`.

**Step 2: Run targeted tests**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest tests/test_ik_model_alignment.py tests/test_robot_version.py -q`
Expected: PASS

### Task 3: Verify and summarize

**Files:**
- Verify only

**Step 1: Run regression suite**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest tests/test_ik_model_alignment.py tests/test_joint_direction_map.py tests/test_robot_version.py tests/test_arm_limits.py -q`
Expected: PASS

Run: `python -m compileall mantis`
Expected: PASS
