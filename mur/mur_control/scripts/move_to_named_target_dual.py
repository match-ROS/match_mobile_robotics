#!/usr/bin/env python3
import sys
from typing import List, Optional

import rospy
import actionlib
import moveit_commander
import moveit_msgs.msg


def _parse_group_names(raw: object) -> List[str]:
    if raw is None:
        return []
    if isinstance(raw, list):
        return [str(x).strip() for x in raw if str(x).strip()]
    # allow comma-separated
    s = str(raw).strip()
    if not s:
        return []
    return [p.strip() for p in s.split(",") if p.strip()]


def _normalize_ns(ns: str) -> str:
    ns = (ns or "").strip()
    if not ns:
        return ""
    if not ns.startswith("/"):
        ns = "/" + ns
    return ns.rstrip("/")


def _wait_for_move_group(moveit_ns: str, timeout_s: float) -> None:
    action_name = _normalize_ns(moveit_ns) + "/move_group"
    rospy.loginfo("Waiting for MoveIt move_group action server at '%s' (timeout %.1fs)...", action_name, timeout_s)
    client = actionlib.SimpleActionClient(action_name, moveit_msgs.msg.MoveGroupAction)
    ok = client.wait_for_server(rospy.Duration.from_sec(timeout_s))
    if not ok:
        raise RuntimeError(f"Timed out waiting for action server: {action_name}")


def _go_named_target(group: moveit_commander.MoveGroupCommander, target: str, timeout_s: float) -> None:
    group.set_planning_time(timeout_s)
    targets = []
    try:
        targets = group.get_named_targets()
    except Exception:
        # best-effort; we'll still try set_named_target
        pass

    if targets and target not in targets:
        raise RuntimeError(f"Named target '{target}' not found for group '{group.get_name()}'. Available: {targets}")

    rospy.loginfo("Group '%s': going to named target '%s'...", group.get_name(), target)
    group.set_named_target(target)
    ok = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    if not ok:
        raise RuntimeError(f"MoveIt execution failed for group '{group.get_name()}' to target '{target}'")


def main() -> int:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_to_named_target_dual", anonymous=True)

    group_names = _parse_group_names(rospy.get_param("~group_names", "UR_arm_l,UR_arm_r"))
    target_name = str(rospy.get_param("~target_name", "Home_custom")).strip()
    moveit_ns = _normalize_ns(str(rospy.get_param("~moveit_ns", rospy.get_namespace())))
    robot_description = str(rospy.get_param("~robot_description", "robot_description")).strip() or "robot_description"
    wait_timeout_s = float(rospy.get_param("~wait_timeout", 120.0))
    planning_time_s = float(rospy.get_param("~planning_time", 10.0))
    vel_scale = float(rospy.get_param("~max_velocity_scaling_factor", 0.3))
    acc_scale = float(rospy.get_param("~max_acceleration_scaling_factor", 0.3))

    if not group_names:
        rospy.logerr("No group names provided. Set ~group_names (e.g. 'UR_arm_l,UR_arm_r').")
        return 2
    if not target_name:
        rospy.logerr("Empty target name. Set ~target_name (e.g. 'Home_custom').")
        return 2

    try:
        _wait_for_move_group(moveit_ns, wait_timeout_s)

        for name in group_names:
            group = moveit_commander.MoveGroupCommander(
                name,
                robot_description=robot_description,
                ns=moveit_ns,
            )
            group.set_max_velocity_scaling_factor(vel_scale)
            group.set_max_acceleration_scaling_factor(acc_scale)
            _go_named_target(group, target_name, planning_time_s)

        rospy.loginfo("Done: moved %s to '%s'.", group_names, target_name)
        return 0
    except Exception as e:
        rospy.logerr("Failed to move groups %s to '%s': %s", group_names, target_name, e)
        return 1
    finally:
        try:
            moveit_commander.roscpp_shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())

