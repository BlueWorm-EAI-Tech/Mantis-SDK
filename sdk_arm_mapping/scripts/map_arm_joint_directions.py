#!/usr/bin/env python3
"""Interactive arm joint direction mapping tool for Mantis SDK."""

from __future__ import annotations

import argparse
import ast
import csv
import math
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG_PATH = REPO_ROOT / "sdk_arm_mapping" / "config" / "arm_mapping.yaml"
DEFAULT_LOG_DIR = REPO_ROOT / "sdk_arm_mapping" / "logs"

JOINT_SEQUENCE_ALL = [
    "wrist_roll",
    "wrist_pitch",
    "wrist_yaw",
    "elbow_pitch",
    "shoulder_pitch",
    "shoulder_roll",
    "shoulder_yaw",
]
JOINT_INDEX_TO_METHOD = {
    0: "set_shoulder_pitch",
    1: "set_shoulder_yaw",
    2: "set_shoulder_roll",
    3: "set_elbow_pitch",
    4: "set_wrist_roll",
    5: "set_wrist_pitch",
    6: "set_wrist_yaw",
}
OBSERVATION_OPTIONS = {
    "1": "向前",
    "2": "向后",
    "3": "向上",
    "4": "向下",
    "5": "向身体内侧",
    "6": "向身体外侧",
    "7": "顺时针旋转",
    "8": "逆时针旋转",
    "9": "弯曲",
    "10": "伸直",
    "11": "不明显",
    "12": "方向不确定",
    "13": "异常/停止测试",
    "14": "__custom__",
}
CSV_FIELDS = [
    "timestamp",
    "component",
    "side",
    "joint",
    "joint_index",
    "sdk_method",
    "baseline_value_rad",
    "delta_rad",
    "target_value_rad",
    "target_value_deg",
    "direction_type",
    "observation",
    "note",
    "executed",
    "status",
    "error_message",
]


class UserAbortedTest(RuntimeError):
    """Raised when the operator chooses to stop the test."""


@dataclass(frozen=True)
class MotionPlan:
    side: str
    joint: str
    joint_index: int
    sdk_method: str
    baseline_value: float
    delta_rad: float

    @property
    def component(self) -> str:
        return f"{self.side}_arm"

    @property
    def positive_target(self) -> float:
        return self.baseline_value + self.delta_rad

    @property
    def negative_target(self) -> float:
        return self.baseline_value - self.delta_rad


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="交互式测试 Mantis 双臂单关节方向映射。默认 dry-run，不连接机器人。"
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG_PATH,
        help=f"配置文件路径，默认: {DEFAULT_CONFIG_PATH}",
    )
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument(
        "--joint",
        choices=tuple(JOINT_SEQUENCE_ALL + ["all"]),
        required=True,
        help="单个关节名，或 all 按推荐顺序逐个测试。",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="允许连接机器人并执行实机动作。",
    )
    parser.add_argument(
        "--i-understand-real-robot-risk",
        action="store_true",
        help="二次安全确认。只有和 --execute 同时出现时才允许实机执行。",
    )
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=DEFAULT_LOG_DIR,
        help=f"CSV 日志目录，默认: {DEFAULT_LOG_DIR}",
    )
    parser.add_argument(
        "--skip-return-baseline",
        action="store_true",
        help="跳过每次动作后的 baseline 回位。不建议实机使用。",
    )
    return parser.parse_args()


def _strip_yaml_comment(line: str) -> str:
    in_single = False
    in_double = False
    chars: List[str] = []
    for char in line:
        if char == "'" and not in_double:
            in_single = not in_single
        elif char == '"' and not in_single:
            in_double = not in_double
        elif char == "#" and not in_single and not in_double:
            break
        chars.append(char)
    return "".join(chars).rstrip()


def _parse_scalar(token: str) -> Any:
    stripped = token.strip()
    if stripped in {"", "null", "Null", "NULL", "~"}:
        return None
    if stripped in {"true", "True"}:
        return True
    if stripped in {"false", "False"}:
        return False
    if stripped.startswith(("'", '"')) and stripped.endswith(("'", '"')):
        return ast.literal_eval(stripped)
    if stripped.startswith("[") and stripped.endswith("]"):
        return ast.literal_eval(stripped)
    try:
        if any(symbol in stripped for symbol in (".", "e", "E")):
            return float(stripped)
        return int(stripped)
    except ValueError:
        return stripped


def _prepare_yaml_lines(text: str) -> List[Tuple[int, str]]:
    prepared: List[Tuple[int, str]] = []
    for raw_line in text.splitlines():
        cleaned = _strip_yaml_comment(raw_line)
        if not cleaned.strip():
            continue
        indent = len(cleaned) - len(cleaned.lstrip(" "))
        prepared.append((indent, cleaned.strip()))
    return prepared


def _parse_yaml_list(
    lines: Sequence[Tuple[int, str]],
    start_index: int,
    indent: int,
) -> Tuple[List[Any], int]:
    values: List[Any] = []
    index = start_index
    while index < len(lines):
        current_indent, content = lines[index]
        if current_indent < indent:
            break
        if current_indent != indent or not content.startswith("- "):
            break
        item = content[2:].strip()
        if not item:
            next_index = index + 1
            if next_index >= len(lines) or lines[next_index][0] <= current_indent:
                values.append(None)
                index = next_index
                continue
            next_indent = lines[next_index][0]
            if lines[next_index][1].startswith("- "):
                nested, index = _parse_yaml_list(lines, next_index, next_indent)
            else:
                nested, index = _parse_yaml_mapping(lines, next_index, next_indent)
            values.append(nested)
            continue
        values.append(_parse_scalar(item))
        index += 1
    return values, index


def _parse_yaml_mapping(
    lines: Sequence[Tuple[int, str]],
    start_index: int,
    indent: int,
) -> Tuple[Dict[str, Any], int]:
    mapping: Dict[str, Any] = {}
    index = start_index
    while index < len(lines):
        current_indent, content = lines[index]
        if current_indent < indent:
            break
        if current_indent != indent:
            raise ValueError(
                f"YAML 缩进不符合预期: line={index + 1}, indent={current_indent}, expected={indent}"
            )
        if content.startswith("- "):
            raise ValueError(f"YAML 位置不应出现列表项: line={index + 1}, content={content!r}")
        if ":" not in content:
            raise ValueError(f"YAML 行缺少冒号: line={index + 1}, content={content!r}")
        key, raw_value = content.split(":", 1)
        key = key.strip()
        raw_value = raw_value.strip()
        if not raw_value:
            next_index = index + 1
            if next_index >= len(lines) or lines[next_index][0] <= current_indent:
                mapping[key] = {}
                index = next_index
                continue
            next_indent = lines[next_index][0]
            if lines[next_index][1].startswith("- "):
                value, index = _parse_yaml_list(lines, next_index, next_indent)
            else:
                value, index = _parse_yaml_mapping(lines, next_index, next_indent)
            mapping[key] = value
            continue
        mapping[key] = _parse_scalar(raw_value)
        index += 1
    return mapping, index


def load_yaml_config(config_path: Path) -> Dict[str, Any]:
    text = config_path.read_text(encoding="utf-8")
    try:
        import yaml  # type: ignore
    except ImportError:
        prepared = _prepare_yaml_lines(text)
        data, next_index = _parse_yaml_mapping(prepared, 0, 0)
        if next_index != len(prepared):
            raise ValueError("配置文件解析未消费全部内容，请检查 YAML 格式。")
        return data

    data = yaml.safe_load(text)
    if not isinstance(data, dict):
        raise ValueError("配置文件顶层必须是映射结构。")
    return data


def validate_config(config: Dict[str, Any]) -> None:
    robot_cfg = config.get("robot")
    test_cfg = config.get("test")
    arms_cfg = config.get("arms")
    if not isinstance(robot_cfg, dict):
        raise ValueError("配置缺少 robot 段。")
    if not isinstance(test_cfg, dict):
        raise ValueError("配置缺少 test 段。")
    if not isinstance(arms_cfg, dict):
        raise ValueError("配置缺少 arms 段。")

    for side in ("left", "right"):
        side_cfg = arms_cfg.get(side)
        if not isinstance(side_cfg, dict):
            raise ValueError(f"arms.{side} 配置缺失。")
        baseline_pose = side_cfg.get("baseline_pose")
        if not isinstance(baseline_pose, list) or len(baseline_pose) != 7:
            raise ValueError(f"arms.{side}.baseline_pose 必须是 7 个元素的列表。")
        joints_cfg = side_cfg.get("joints")
        if not isinstance(joints_cfg, dict):
            raise ValueError(f"arms.{side}.joints 配置缺失。")

        for joint_name in JOINT_SEQUENCE_ALL:
            joint_cfg = joints_cfg.get(joint_name)
            if not isinstance(joint_cfg, dict):
                raise ValueError(f"arms.{side}.joints.{joint_name} 配置缺失。")
            index = joint_cfg.get("index")
            sdk_method = joint_cfg.get("sdk_method")
            delta_rad = joint_cfg.get("delta_rad")
            if index not in JOINT_INDEX_TO_METHOD:
                raise ValueError(f"{side}.{joint_name}.index 非法: {index!r}")
            expected_method = JOINT_INDEX_TO_METHOD[index]
            if sdk_method != expected_method:
                raise ValueError(
                    f"{side}.{joint_name}.sdk_method 应为 {expected_method!r}，当前为 {sdk_method!r}"
                )
            if not isinstance(delta_rad, (int, float)) or float(delta_rad) <= 0.0:
                raise ValueError(f"{side}.{joint_name}.delta_rad 必须为正数。")


def get_joint_names(selected_joint: str) -> List[str]:
    return JOINT_SEQUENCE_ALL.copy() if selected_joint == "all" else [selected_joint]


def build_motion_plans(config: Dict[str, Any], side: str, joint_names: Sequence[str]) -> List[MotionPlan]:
    side_cfg = config["arms"][side]
    baseline_pose = [float(value) for value in side_cfg["baseline_pose"]]
    joints_cfg = side_cfg["joints"]

    plans: List[MotionPlan] = []
    for joint_name in joint_names:
        joint_cfg = joints_cfg[joint_name]
        joint_index = int(joint_cfg["index"])
        baseline_value = baseline_pose[joint_index]
        plans.append(
            MotionPlan(
                side=side,
                joint=joint_name,
                joint_index=joint_index,
                sdk_method=str(joint_cfg["sdk_method"]),
                baseline_value=baseline_value,
                delta_rad=float(joint_cfg["delta_rad"]),
            )
        )
    return plans


def print_test_plan(plans: Sequence[MotionPlan]) -> None:
    print("=" * 72)
    print("测试计划")
    print("=" * 72)
    for plan in plans:
        print(f"side: {plan.side}")
        print(f"joint: {plan.joint}")
        print(f"joint_index: {plan.joint_index}")
        print(f"sdk_method: {plan.sdk_method}")
        print(f"baseline_value: {plan.baseline_value:.6f} rad")
        print(f"delta_rad: {plan.delta_rad:.6f} rad")
        print(f"positive_target: {plan.positive_target:.6f} rad")
        print(f"negative_target: {plan.negative_target:.6f} rad")
        print("-" * 72)


def print_action_sequence(plans: Sequence[MotionPlan], execute: bool, skip_return_baseline: bool) -> None:
    print("动作序列")
    print("=" * 72)
    for plan in plans:
        print(f"{plan.component} / {plan.joint}")
        print(f"  1. 回到 baseline_pose -> joint[{plan.joint_index}] = {plan.baseline_value:.6f} rad")
        print(
            f"  2. 执行 positive_delta -> {plan.positive_target:.6f} rad "
            f"({math.degrees(plan.positive_target):.2f} deg)"
        )
        if skip_return_baseline:
            print("  3. 按参数要求跳过动作后回 baseline")
        else:
            print("  3. 回到 baseline_pose")
        print(
            f"  4. 执行 negative_delta -> {plan.negative_target:.6f} rad "
            f"({math.degrees(plan.negative_target):.2f} deg)"
        )
        if skip_return_baseline:
            print("  5. 按参数要求跳过动作后回 baseline")
        else:
            print("  5. 回到 baseline_pose")
        if execute and len(plans) > 1:
            print("  6. 下一个关节开始前会再次等待人工按 Enter 确认")
        print("-" * 72)


def create_robot(config: Dict[str, Any]):
    if str(REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(REPO_ROOT))

    from mantis import Mantis

    robot_cfg = config["robot"]
    robot_kwargs: Dict[str, Any] = {"port": int(robot_cfg.get("port", 7447))}
    robot_version = str(robot_cfg.get("robot_version", "") or "").strip()
    if robot_version:
        robot_kwargs["robot_version"] = robot_version

    serial_number = str(robot_cfg.get("sn", "") or "").strip()
    ip_address = str(robot_cfg.get("ip", "") or "").strip()
    if serial_number:
        robot_kwargs["sn"] = serial_number
    elif ip_address:
        robot_kwargs["ip"] = ip_address

    return Mantis(**robot_kwargs)


def connect_robot(robot: Any, config: Dict[str, Any]) -> None:
    robot_cfg = config["robot"]
    timeout = float(robot_cfg.get("connect_timeout", 5.0))
    serial_number = str(robot_cfg.get("sn", "") or "").strip()
    ip_address = str(robot_cfg.get("ip", "") or "").strip()
    kwargs: Dict[str, Any] = {"timeout": timeout}
    if serial_number:
        kwargs["sn"] = serial_number
    elif ip_address:
        kwargs["ip"] = ip_address

    connected = robot.connect(**kwargs)
    if not connected:
        raise RuntimeError("连接机器人失败，未收到可用状态。")


def create_csv_logger(log_dir: Path) -> Tuple[Path, Any, csv.DictWriter]:
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f"arm_joint_mapping_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    handle = log_path.open("w", newline="", encoding="utf-8")
    writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
    writer.writeheader()
    handle.flush()
    return log_path, handle, writer


def write_log_row(handle: Any, writer: csv.DictWriter, row: Dict[str, str]) -> None:
    writer.writerow(row)
    handle.flush()


def prompt_for_joint_start(plan: MotionPlan) -> None:
    input(
        f"\n即将开始 {plan.component} / {plan.joint}，请确认周围无障碍物后按 Enter 继续，"
        "Ctrl+C 取消。"
    )


def prompt_before_motion(plan: MotionPlan, direction_type: str, target_value: float) -> None:
    print()
    print(f"即将执行: {plan.component} / {plan.joint} / {direction_type}")
    print(f"目标角度: {target_value:.6f} rad / {math.degrees(target_value):.2f} deg")
    input("确认周围安全并观察视角就绪后，按 Enter 执行该动作，Ctrl+C 取消。")


def prompt_for_observation() -> Tuple[str, str, bool]:
    print()
    print("观察结果菜单")
    for option, label in OBSERVATION_OPTIONS.items():
        if option == "14":
            print(f"{option}. 自定义输入")
        else:
            print(f"{option}. {label}")

    choice = ""
    for _ in range(5):
        choice = input("请输入编号 [1-14]: ").strip()
        if choice in OBSERVATION_OPTIONS:
            break
        print("输入无效，请重新输入合法编号。")
    else:
        raise ValueError("观察结果输入错误次数过多，停止本次测试。")

    if choice == "14":
        observation = input("请输入自定义观察结果: ").strip() or "自定义输入"
    else:
        observation = OBSERVATION_OPTIONS[choice]
    note = input("备注（可留空）: ").strip()
    return observation, note, choice == "13"


def move_to_baseline(arm: Any, baseline_pose: Sequence[float]) -> None:
    arm.set_joints(list(baseline_pose), block=False)
    arm.wait()


def build_row(
    plan: MotionPlan,
    direction_type: str,
    target_value: float,
    observation: str,
    note: str,
    executed: bool,
    status: str,
    error_message: str,
) -> Dict[str, str]:
    return {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "component": plan.component,
        "side": plan.side,
        "joint": plan.joint,
        "joint_index": str(plan.joint_index),
        "sdk_method": plan.sdk_method,
        "baseline_value_rad": f"{plan.baseline_value:.6f}",
        "delta_rad": f"{plan.delta_rad:.6f}",
        "target_value_rad": f"{target_value:.6f}",
        "target_value_deg": f"{math.degrees(target_value):.2f}",
        "direction_type": direction_type,
        "observation": observation,
        "note": note,
        "executed": "true" if executed else "false",
        "status": status,
        "error_message": error_message,
    }


def run_single_motion(
    robot: Any,
    arm: Any,
    baseline_pose: Sequence[float],
    plan: MotionPlan,
    direction_type: str,
    target_value: float,
    settle_time_sec: float,
    return_to_baseline: bool,
    log_handle: Any,
    writer: csv.DictWriter,
) -> None:
    executed = False
    observation = ""
    note = ""

    try:
        print(f"\n先回到 baseline_pose: {list(baseline_pose)}")
        move_to_baseline(arm, baseline_pose)

        prompt_before_motion(plan, direction_type, target_value)
        joint_method = getattr(arm, plan.sdk_method)
        joint_method(target_value, block=False)
        arm.wait()
        executed = True

        if settle_time_sec > 0.0:
            time.sleep(settle_time_sec)

        print("请人工观察关节运动方向，并录入观察结果。")
        observation, note, aborted = prompt_for_observation()
        if aborted:
            row = build_row(
                plan=plan,
                direction_type=direction_type,
                target_value=target_value,
                observation=observation,
                note=note,
                executed=executed,
                status="aborted",
                error_message="operator requested stop",
            )
            write_log_row(log_handle, writer, row)
            robot.stop()
            raise UserAbortedTest(f"用户在 {plan.component} / {plan.joint} 处中止测试。")

        if return_to_baseline:
            print("动作完成，回到 baseline_pose。")
            move_to_baseline(arm, baseline_pose)
        else:
            print("根据参数设置，跳过动作后的 baseline 回位。")

        row = build_row(
            plan=plan,
            direction_type=direction_type,
            target_value=target_value,
            observation=observation,
            note=note,
            executed=executed,
            status="ok",
            error_message="",
        )
        write_log_row(log_handle, writer, row)
        print("本次观察结果已写入 CSV。")
    except UserAbortedTest:
        raise
    except Exception as exc:
        row = build_row(
            plan=plan,
            direction_type=direction_type,
            target_value=target_value,
            observation=observation,
            note=note,
            executed=executed,
            status="error",
            error_message=str(exc),
        )
        write_log_row(log_handle, writer, row)
        raise


def safe_stop_and_disconnect(robot: Any) -> None:
    if robot is None:
        return
    try:
        robot.stop()
    except Exception as exc:
        print(f"清理时 robot.stop() 失败: {exc}", file=sys.stderr)
    try:
        robot.disconnect()
    except Exception as exc:
        print(f"清理时 robot.disconnect() 失败: {exc}", file=sys.stderr)


def main() -> int:
    args = parse_args()
    config = load_yaml_config(args.config)
    validate_config(config)
    config_confirm_before_each_motion = bool(
        config["test"].get("confirm_before_each_motion", True)
    )
    config_return_to_baseline = bool(
        config["test"].get("return_to_baseline_after_each_motion", True)
    )

    joint_names = get_joint_names(args.joint)
    plans = build_motion_plans(config, args.side, joint_names)
    print_test_plan(plans)
    print_action_sequence(plans, execute=args.execute, skip_return_baseline=args.skip_return_baseline)

    if not args.execute:
        print("当前为 dry-run 模式：不会连接机器人，不会执行实机动作。")
        return 0

    if not args.i_understand_real_robot_risk:
        print(
            "拒绝执行：实机模式必须同时传入 --execute 和 --i-understand-real-robot-risk。",
            file=sys.stderr,
        )
        return 2

    if args.skip_return_baseline:
        print("警告：你启用了 --skip-return-baseline，这不建议在实机环境中使用。")
    if not config_confirm_before_each_motion:
        print("提示：配置里关闭了 confirm_before_each_motion，但实机模式仍会强制逐动作 Enter 确认。")
    if not config_return_to_baseline and not args.skip_return_baseline:
        print(
            "提示：配置里关闭了 return_to_baseline_after_each_motion，"
            "但脚本默认仍会在每次动作后回到 baseline。"
        )

    robot = None
    log_handle = None
    try:
        robot = create_robot(config)
        connect_robot(robot, config)

        arm = robot.left_arm if args.side == "left" else robot.right_arm
        baseline_pose = [float(value) for value in config["arms"][args.side]["baseline_pose"]]
        default_speed = float(config["test"].get("default_speed", 0.20))
        settle_time_sec = float(config["test"].get("settle_time_sec", 1.0))
        return_to_baseline = not args.skip_return_baseline

        log_path, log_handle, writer = create_csv_logger(args.log_dir)
        print(f"CSV 日志文件: {log_path}")

        arm.set_speed(default_speed)
        print(f"已设置 {args.side}_arm 速度为 {default_speed:.3f} rad/s")

        for plan in plans:
            if len(plans) > 1:
                prompt_for_joint_start(plan)

            run_single_motion(
                robot=robot,
                arm=arm,
                baseline_pose=baseline_pose,
                plan=plan,
                direction_type="positive_delta",
                target_value=plan.positive_target,
                settle_time_sec=settle_time_sec,
                return_to_baseline=return_to_baseline,
                log_handle=log_handle,
                writer=writer,
            )
            run_single_motion(
                robot=robot,
                arm=arm,
                baseline_pose=baseline_pose,
                plan=plan,
                direction_type="negative_delta",
                target_value=plan.negative_target,
                settle_time_sec=settle_time_sec,
                return_to_baseline=return_to_baseline,
                log_handle=log_handle,
                writer=writer,
            )

        print("测试完成。")
        return 0
    except UserAbortedTest as exc:
        print(str(exc), file=sys.stderr)
        return 3
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，正在尝试停止并断开机器人。", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"执行失败: {exc}", file=sys.stderr)
        raise
    finally:
        if log_handle is not None:
            log_handle.close()
        safe_stop_and_disconnect(robot)


if __name__ == "__main__":
    raise SystemExit(main())
