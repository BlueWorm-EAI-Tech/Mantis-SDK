# SDK Examples Reorg Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Move runnable SDK examples out of the repository root and ensure every public SDK capability has a discoverable example.

**Architecture:** Keep automated tests under `tests/` and user-run examples under `examples/`. Examples must be safe by default: scripts accept `--ip` or `--sn`, use small motion values, and do not keep hard-coded robot addresses. Shared argument parsing lives in `examples/common.py`.

**Tech Stack:** Python, argparse, pytest, setuptools

---

### Task 1: Add Examples Scaffold

**Files:**
- Create: `examples/common.py`
- Create: `examples/README.md`
- Modify: `pyproject.toml`

**Steps:**
1. Create shared `RobotArgs` helpers for `--ip`, `--sn`, `--port`, `--robot-version`, and `--non-blocking`.
2. Add `examples/README.md` with a capability matrix.
3. Exclude `examples` from pytest collection by keeping `testpaths = ["tests"]`.

### Task 2: Move Root Scripts Into Examples

**Files:**
- Move root `test_*.py`, `measure_frequency.py`, and `coffee*.py` scripts into `examples/`.
- Delete stale notebook duplicates from root unless they are intentionally preserved as workflow examples.

**Steps:**
1. Convert each root script from hard-coded IP/SN to shared CLI args.
2. Rename files from `test_*.py` to descriptive `*_example.py` names so they are not mistaken for automated tests.
3. Preserve coffee workflow scripts under `examples/workflows/`.
4. Move old notebook experiments to `archive/notebooks/`; they may contain historical robot targets and are not maintained as runnable examples.

### Task 3: Add Coverage Examples

**Files:**
- Create missing examples for connection, discovery, status subscription, arms, grippers, head, waist, chassis, parallel motion, and robot version 3.0 controls.

**Steps:**
1. Ensure every public SDK method family has at least one example.
2. Use small motion values and explicit `block` behavior.
3. Document 3.0-only waist bend and robot-side IK requirements.

### Task 4: Verify

**Commands:**
- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q`
- `python -m compileall -q mantis examples`
- `python -m build --wheel --outdir /tmp/mantis-sdk-examples-dist`
- Inspect wheel to ensure examples are not packaged as SDK modules.

### Task 5: Commit

**Commit message:**
- `chore: 规范 SDK examples 目录`
