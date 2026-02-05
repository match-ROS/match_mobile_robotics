#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Interactive Robot Control Menu
==============================
Script interattivo per controllare il robot in tempo reale tramite menù nel terminale.

Funzionalità:
- Visualizzare la posizione corrente dell'end effector
- Inviare pose predefinite al robot come goal (via MoveIt o velocità diretta)
- Salvare nuove pose dalla posizione corrente
- Abilitare/disabilitare la velocità repulsiva degli ostacoli
- Selezionare il link e l'ostacolo per le velocità repulsive
- Passare da un controller all'altro (MoveIt <-> Velocità)
- Movimento in loop attraverso una sequenza di pose

Usage:
    rosrun cartesian_velocity_controller robot_interactive_control.py

Autore: cartesian_velocity_controller package
Versione: 2.0.0 (Refactored)
"""

import sys
import os
import argparse
from typing import Optional, List

# Add the scripts directory to the path for package imports
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

from interactive_control import InteractiveController


def _parent_namespace(resolved_node_name: str) -> str:
    """
    Return the parent namespace of a resolved ROS node name.

    Examples:
      "/mur620/cartesian_velocity_controller_l" -> "/mur620"
      "/a/b/c" -> "/a/b"
      "cartesian_velocity_controller" (unresolved) -> "/"
    """
    try:
        name = str(resolved_node_name or "")
        if not name.startswith("/"):
            return "/"
        parts = [p for p in name.split("/") if p]
        if len(parts) <= 1:
            return "/"
        return "/" + "/".join(parts[:-1])
    except Exception:
        return "/"


def _resolve_topic_in_controller_ns(controller_node_name: str, topic: Optional[str]) -> Optional[str]:
    """
    Resolve a topic name using the *controller node namespace* (not this script's namespace).

    The cartesian controller params often store relative topic names like:
      "joint_group_vel_controller_l/unsafe/command"
    which should resolve to:
      "/mur620/joint_group_vel_controller_l/unsafe/command"
    when the controller node is "/mur620/cartesian_velocity_controller_l".
    """
    if not topic:
        return None
    t = str(topic)
    if t.startswith("/"):
        return t
    parent = _parent_namespace(controller_node_name)
    if parent == "/":
        return "/" + t.lstrip("/")
    return parent.rstrip("/") + "/" + t.lstrip("/")


def _wait_for_pub_connections(pub, timeout_s: float, topic: str) -> bool:
    """Wait for at least one subscriber connection."""
    try:
        import rospy
    except Exception:
        return False

    end_t = rospy.Time.now() + rospy.Duration(max(0.0, float(timeout_s)))
    while pub.get_num_connections() == 0:
        if rospy.is_shutdown():
            return False
        if timeout_s > 0 and rospy.Time.now() > end_t:
            rospy.logwarn(f"No subscribers connected to {topic}")
            return False
        rospy.sleep(0.05)
    return True


def _publish_direct_joint_velocity(
    controller_node_name: str,
    joint_velocities: List[float],
    velocity_command_topic_override: Optional[str] = None,
    rate_hz: float = 50.0,
    duration_s: float = 1.0,
    wait_for_subscriber_timeout_s: float = 2.0,
    stop_at_end: bool = True,
) -> bool:
    """
    Publish joint velocity commands directly to the underlying joint velocity controller.

    This is useful to quickly validate wiring and the downstream velocity controller by
    publishing on the same topic that the cartesian controller uses (std_msgs/Float64MultiArray).
    """
    import rospy
    from std_msgs.msg import Float64MultiArray

    controller_node_name = rospy.resolve_name(controller_node_name)

    # Determine target topic:
    # - CLI override wins
    # - otherwise read from controller params (private namespace)
    topic_param = None
    try:
        topic_param = rospy.get_param(f"{controller_node_name}/velocity_command_topic", None)
    except Exception:
        topic_param = None

    # Prefer override, otherwise controller param
    topic_raw = velocity_command_topic_override or topic_param
    topic = _resolve_topic_in_controller_ns(controller_node_name, topic_raw)
    if not topic:
        rospy.logerr("Could not resolve velocity command topic (override/param missing)")
        return False

    pub = rospy.Publisher(topic, Float64MultiArray, queue_size=1)
    if wait_for_subscriber_timeout_s is not None and float(wait_for_subscriber_timeout_s) > 0:
        if not _wait_for_pub_connections(pub, float(wait_for_subscriber_timeout_s), topic):
            rospy.logerr(f"No subscribers on {topic} (timeout). Not publishing.")
            return False

    msg = Float64MultiArray()
    msg.data = [float(v) for v in joint_velocities]

    rate = rospy.Rate(max(0.1, float(rate_hz)))
    start = rospy.Time.now()

    rospy.loginfo(
        f"Publishing direct joint velocities to {topic} "
        f"(n={len(msg.data)}, rate={rate_hz} Hz, duration={duration_s}s)"
    )

    ok = True
    try:
        while not rospy.is_shutdown():
            if duration_s is not None and float(duration_s) > 0:
                if (rospy.Time.now() - start).to_sec() >= float(duration_s):
                    break
            pub.publish(msg)
            rate.sleep()
    except rospy.ROSInterruptException:
        ok = False
    except KeyboardInterrupt:
        ok = False
    finally:
        if stop_at_end and not rospy.is_shutdown():
            zero = Float64MultiArray()
            zero.data = [0.0 for _ in msg.data]
            pub.publish(zero)
            rospy.sleep(0.05)

    return ok


def main():
    """Entry point for the interactive robot control script."""
    parser = argparse.ArgumentParser(description="Interactive control for cartesian_velocity_controller")
    parser.add_argument("--mur-ns", default="/mur620", help="Robot namespace (e.g. /mur620)")
    parser.add_argument("--arm", choices=["l", "r"], help="Select arm profile (l/r) for MUR620 dual-arm")
    parser.add_argument("--controller-node-name", default=None, help="Controller node name (e.g. /mur620/cartesian_velocity_controller_l)")
    parser.add_argument("--controller-manager-ns", default=None, help="controller_manager namespace (e.g. /mur620/controller_manager)")
    parser.add_argument("--velocity-controller", default=None, help="Velocity controller name to start (controller_manager)")
    parser.add_argument("--moveit-controller", default=None, help="MoveIt trajectory controller name to stop/start (controller_manager)")
    parser.add_argument("--move-group-name", default=None, help="MoveIt move group name (e.g. UR_arm_l)")
    parser.add_argument("--global-frame", default=None, help="Global TF frame (default: base_link)")
    parser.add_argument("--ee-frame", default=None, help="End-effector TF frame (e.g. UR10_l/tool0)")
    parser.add_argument("--target-pose-topic", default=None, help="Topic for PoseStamped targets (default: <controller_node>/target_pose)")
    # Non-interactive / scripting helpers
    parser.add_argument("--switch-to", choices=["velocity", "moveit", "none"], default="none",
                        help="Optionally switch controller before sending a pose (velocity/moveit/none)")
    parser.add_argument("--send-pose", default=None, help="Send a saved pose by name and exit (no menu)")
    parser.add_argument("--send-custom-pose", nargs="+", type=float, default=None,
                        help="Send a custom pose and exit. Format: x y z [qx qy qz qw]")

    # Direct joint velocity publish (debug / wiring verification)
    parser.add_argument(
        "--direct-joint",
        default=None,
        help="Publish a velocity on ONE joint by INDEX (1..N) and exit. "
             "N is inferred from the downstream ros_control JointGroupVelocityController (`.../joints`).",
    )
    parser.add_argument(
        "--direct-vel",
        type=float,
        default=None,
        help="Velocity [rad/s] to apply to --direct-joint (all other joints set to 0).",
    )
    parser.add_argument(
        "--direct-nj",
        type=int,
        default=None,
        help="Optional override for number of joints N (only if auto-detect fails).",
    )
    parser.add_argument("--direct-joint-vel", nargs="+", type=float, default=None,
                        help="Publish joint velocities directly (std_msgs/Float64MultiArray) and exit. "
                             "Provide N floats (e.g. 6 for UR10).")
    parser.add_argument("--direct-velocity-topic", default=None,
                        help="Override velocity command topic. If omitted, uses "
                             "<controller_node>/velocity_command_topic param, resolved in controller namespace.")
    parser.add_argument("--direct-rate", type=float, default=50.0, help="Direct velocity publish rate [Hz]")
    parser.add_argument("--direct-duration", type=float, default=1.0,
                        help="How long to publish [s]. Use 0 for 'until Ctrl+C'.")
    parser.add_argument("--direct-wait-subscriber-timeout", type=float, default=2.0,
                        help="Wait for a subscriber on the command topic [s]. Use 0 to skip waiting.")
    parser.add_argument("--direct-no-stop-at-end", action="store_true",
                        help="Do not publish a final zero-velocity command on exit (NOT recommended).")

    # Optional second robot (batch send in one process)
    parser.add_argument("--mur-ns-2", default=None, help="Second robot namespace (e.g. /mur620_2)")
    parser.add_argument("--arm-2", choices=["l", "r"], default=None, help="Second robot arm profile (l/r)")
    parser.add_argument("--controller-node-name-2", default=None, help="Second controller node name")
    parser.add_argument("--controller-manager-ns-2", default=None, help="Second controller_manager namespace")
    parser.add_argument("--velocity-controller-2", default=None, help="Second velocity controller name")
    parser.add_argument("--moveit-controller-2", default=None, help="Second MoveIt controller name")
    parser.add_argument("--move-group-name-2", default=None, help="Second MoveIt move group name")
    parser.add_argument("--global-frame-2", default=None, help="Second robot global TF frame")
    parser.add_argument("--ee-frame-2", default=None, help="Second robot end-effector TF frame")
    parser.add_argument("--target-pose-topic-2", default=None, help="Second robot PoseStamped target topic")
    parser.add_argument("--switch-to-2", choices=["velocity", "moveit", "none"], default="none",
                        help="Optionally switch controller for second robot before sending")
    parser.add_argument("--send-pose-2", default=None, help="Send a saved pose to second robot and exit")
    parser.add_argument("--send-custom-pose-2", nargs="+", type=float, default=None,
                        help="Send a custom pose to second robot and exit. Format: x y z [qx qy qz qw]")

    args, _unknown = parser.parse_known_args()

    controller_node_name = args.controller_node_name
    controller_manager_ns = args.controller_manager_ns
    velocity_controller = args.velocity_controller
    moveit_controller = args.moveit_controller
    move_group_name = args.move_group_name
    global_frame = args.global_frame or "base_link"
    ee_frame = args.ee_frame

    if args.arm:
        controller_node_name = controller_node_name or f"{args.mur_ns}/cartesian_velocity_controller_{args.arm}"
        controller_manager_ns = controller_manager_ns or f"{args.mur_ns}/controller_manager"
        velocity_controller = velocity_controller or f"joint_group_vel_controller_{args.arm}/unsafe"
        moveit_controller = moveit_controller or f"UR10_{args.arm}/arm_controller"
        move_group_name = move_group_name or f"UR_arm_{args.arm}"
        ee_frame = ee_frame or f"UR10_{args.arm}/tool0"

    # Non-interactive mode: publish a single joint velocity (minimal debug UX) and exit
    if args.direct_joint is not None or args.direct_vel is not None:
        if args.direct_joint is None or args.direct_vel is None:
            raise ValueError("--direct-joint and --direct-vel must be provided together")

        import rospy
        from interactive_control.core.ros_interface import ROSInterface

        # Ensure node initialized once
        already_initialized = False
        try:
            already_initialized = bool(rospy.core.is_initialized())
        except Exception:
            try:
                _ = rospy.get_name()
                already_initialized = True
            except Exception:
                already_initialized = False

        if not already_initialized:
            rospy.init_node("robot_interactive_control_direct_single_joint_vel", anonymous=True)

        ros = ROSInterface(controller_node_name=controller_node_name or "cartesian_velocity_controller",
                           global_frame=global_frame,
                           ee_frame=ee_frame or "tool0",
                           target_pose_topic=args.target_pose_topic)
        if not ros.initialize(node_name="robot_interactive_control_direct_single_joint_vel"):
            sys.exit(1)

        # Infer N from ros_control controller params (preferred) / feedback topic.
        n_joints = int(args.direct_nj) if args.direct_nj is not None else int(ros.get_command_joint_count(timeout_s=2.0) or 0)
        if n_joints <= 0:
            rospy.logerr("Could not infer number of joints N. Provide --direct-nj N.")
            sys.exit(2)

        sel = str(args.direct_joint).strip()
        try:
            n = int(sel)
        except ValueError:
            rospy.logerr(f"Invalid joint selector '{sel}'. Expected an integer 1..{n_joints}.")
            sys.exit(2)

        if not (1 <= n <= n_joints):
            rospy.logerr(f"Invalid joint index {n}. Expected 1..{n_joints}.")
            sys.exit(2)

        cmd = [0.0 for _ in range(n_joints)]
        cmd[n - 1] = float(args.direct_vel)

        # Use the same publishing semantics as the existing --direct-joint-vel mode:
        # duration==0 -> publish until Ctrl+C.
        ok1 = _publish_direct_joint_velocity(
            controller_node_name=controller_node_name or "cartesian_velocity_controller",
            joint_velocities=cmd,
            velocity_command_topic_override=args.direct_velocity_topic,
            rate_hz=float(args.direct_rate),
            duration_s=float(args.direct_duration),
            wait_for_subscriber_timeout_s=float(args.direct_wait_subscriber_timeout),
            stop_at_end=not bool(args.direct_no_stop_at_end),
        )
        sys.exit(0 if ok1 else 2)

    # Non-interactive mode: publish direct joint velocities and exit
    if args.direct_joint_vel is not None:
        import rospy
        already_initialized = False
        try:
            already_initialized = bool(rospy.core.is_initialized())
        except Exception:
            try:
                _ = rospy.get_name()
                already_initialized = True
            except Exception:
                already_initialized = False

        if not already_initialized:
            rospy.init_node("robot_interactive_control_direct_joint_vel", anonymous=True)

        ok1 = _publish_direct_joint_velocity(
            controller_node_name=controller_node_name or "cartesian_velocity_controller",
            joint_velocities=list(args.direct_joint_vel),
            velocity_command_topic_override=args.direct_velocity_topic,
            rate_hz=float(args.direct_rate),
            duration_s=float(args.direct_duration),
            wait_for_subscriber_timeout_s=float(args.direct_wait_subscriber_timeout),
            stop_at_end=not bool(args.direct_no_stop_at_end),
        )
        sys.exit(0 if ok1 else 2)

    controller = InteractiveController(
        controller_node_name=controller_node_name or "cartesian_velocity_controller",
        global_frame=global_frame,
        ee_frame=ee_frame or "tool0",
        target_pose_topic=args.target_pose_topic,
        velocity_controller=velocity_controller,
        moveit_controller=moveit_controller,
        controller_manager_ns=controller_manager_ns,
        move_group_name=move_group_name,
    )

    # Build optional second controller config
    controller2 = None
    if args.mur_ns_2 or args.controller_node_name_2 or args.send_pose_2 or args.send_custom_pose_2:
        mur_ns_2 = args.mur_ns_2 or args.mur_ns
        controller_node_name_2 = args.controller_node_name_2
        controller_manager_ns_2 = args.controller_manager_ns_2
        velocity_controller_2 = args.velocity_controller_2
        moveit_controller_2 = args.moveit_controller_2
        move_group_name_2 = args.move_group_name_2
        global_frame_2 = args.global_frame_2 or global_frame
        ee_frame_2 = args.ee_frame_2

        if args.arm_2:
            controller_node_name_2 = controller_node_name_2 or f"{mur_ns_2}/cartesian_velocity_controller_{args.arm_2}"
            controller_manager_ns_2 = controller_manager_ns_2 or f"{mur_ns_2}/controller_manager"
            velocity_controller_2 = velocity_controller_2 or f"joint_group_vel_controller_{args.arm_2}/unsafe"
            moveit_controller_2 = moveit_controller_2 or f"UR10_{args.arm_2}/arm_controller"
            move_group_name_2 = move_group_name_2 or f"UR_arm_{args.arm_2}"
            ee_frame_2 = ee_frame_2 or f"UR10_{args.arm_2}/tool0"

        controller2 = InteractiveController(
            controller_node_name=controller_node_name_2 or "cartesian_velocity_controller",
            global_frame=global_frame_2,
            ee_frame=ee_frame_2 or "tool0",
            target_pose_topic=args.target_pose_topic_2,
            velocity_controller=velocity_controller_2,
            moveit_controller=moveit_controller_2,
            controller_manager_ns=controller_manager_ns_2,
            move_group_name=move_group_name_2,
        )

    # Non-interactive mode: send pose(s) and exit
    if args.send_pose or args.send_custom_pose or args.send_pose_2 or args.send_custom_pose_2:
        if not controller.initialize():
            sys.exit(1)

        if args.switch_to == "velocity":
            controller.controller_mgr.switch_to_velocity()
        elif args.switch_to == "moveit":
            controller.controller_mgr.switch_to_moveit()

        if args.send_pose:
            ok1 = controller.send_pose(args.send_pose)
        else:
            ok1 = True
            if args.send_custom_pose:
                vals = args.send_custom_pose
                if len(vals) not in (3, 7):
                    raise ValueError("--send-custom-pose expects 3 or 7 floats: x y z [qx qy qz qw]")
                if len(vals) == 3:
                    ok1 = controller.send_custom_pose(vals[0], vals[1], vals[2])
                else:
                    ok1 = controller.send_custom_pose(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6])

        ok2 = True
        if controller2 is not None and (args.send_pose_2 or args.send_custom_pose_2):
            if not controller2.initialize():
                sys.exit(1)

            if args.switch_to_2 == "velocity":
                controller2.controller_mgr.switch_to_velocity()
            elif args.switch_to_2 == "moveit":
                controller2.controller_mgr.switch_to_moveit()

            if args.send_pose_2:
                ok2 = controller2.send_pose(args.send_pose_2)
            else:
                if args.send_custom_pose_2:
                    vals = args.send_custom_pose_2
                    if len(vals) not in (3, 7):
                        raise ValueError("--send-custom-pose-2 expects 3 or 7 floats: x y z [qx qy qz qw]")
                    if len(vals) == 3:
                        ok2 = controller2.send_custom_pose(vals[0], vals[1], vals[2])
                    else:
                        ok2 = controller2.send_custom_pose(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6])

        sys.exit(0 if (ok1 and ok2) else 2)

    # Default: interactive menu
    controller.run()


if __name__ == '__main__':
    main()
