#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Ensure/load velocity controllers and optionally switch at startup (ROS1).

This is meant to be launched from a roslaunch file under the robot namespace,
e.g. /mur620, and will:
  - wait for controller_manager services
  - ensure the configured controllers are loaded (typically left/right unsafe)
  - optionally switch from MoveIt controllers to velocity controllers

It is designed to be safe if controllers are already loaded/running.
"""

from __future__ import annotations

from typing import List

import rospy
from controller_manager_msgs.srv import (
    ListControllers,
    LoadController,
    LoadControllerRequest,
    SwitchController,
    SwitchControllerRequest,
)


def _resolve_cm_ns(cm_ns: str) -> str:
    # Allow passing either "controller_manager" (relative) or "/mur620/controller_manager" (absolute)
    return rospy.resolve_name(cm_ns)


def _matches(configured: str, reported: str) -> bool:
    """Robust controller name match (exact or suffix on slash boundary)."""
    if configured == reported:
        return True
    return reported.endswith(f"/{configured}")


def _list_controllers(list_srv: rospy.ServiceProxy) -> ListControllers:
    return list_srv()


def _controller_present(resp: ListControllers, name: str) -> bool:
    for c in resp.controller:
        if _matches(name, c.name):
            return True
    return False


def _controller_running(resp: ListControllers, name: str) -> bool:
    for c in resp.controller:
        if _matches(name, c.name) and c.state == "running":
            return True
    return False


def _ensure_loaded(load_srv: rospy.ServiceProxy, list_srv: rospy.ServiceProxy, name: str) -> bool:
    resp = _list_controllers(list_srv)
    if _controller_present(resp, name):
        rospy.loginfo(f"[ensure] Controller already loaded: {name}")
        return True

    req = LoadControllerRequest()
    req.name = name
    try:
        out = load_srv(req)
    except rospy.ServiceException as e:
        rospy.logerr(f"[ensure] load_controller({name}) failed: {e}")
        return False

    if out.ok:
        rospy.loginfo(f"[ensure] Loaded controller: {name}")
        return True

    rospy.logwarn(f"[ensure] load_controller returned ok=false for: {name} (maybe already loaded?)")
    # Re-check
    resp2 = _list_controllers(list_srv)
    return _controller_present(resp2, name)


def _switch(
    switch_srv: rospy.ServiceProxy,
    start_controllers: List[str],
    stop_controllers: List[str],
    strictness: int,
    timeout_s: float,
) -> bool:
    req = SwitchControllerRequest()
    req.start_controllers = list(start_controllers)
    req.stop_controllers = list(stop_controllers)
    req.strictness = int(strictness)
    req.start_asap = False
    # controller_manager_msgs/SwitchController.srv expects float64 timeout (seconds), not rospy.Duration.
    req.timeout = max(0.0, float(timeout_s))

    try:
        out = switch_srv(req)
    except rospy.ServiceException as e:
        rospy.logerr(f"[switch] switch_controller failed: {e}")
        return False

    if out.ok:
        rospy.loginfo(
            "[switch] Switched controllers. "
            f"started={start_controllers} stopped={stop_controllers}"
        )
        return True

    rospy.logerr(
        "[switch] switch_controller returned ok=false. "
        "Make sure controllers are loaded and not conflicting. "
        f"start={start_controllers} stop={stop_controllers}"
    )
    return False


def main() -> None:
    rospy.init_node("ensure_mur620_dual_arm_velocity_controllers", anonymous=True)

    controller_manager_ns = rospy.get_param("~controller_manager_ns", "controller_manager")
    cm_ns = _resolve_cm_ns(controller_manager_ns)

    velocity_controllers = rospy.get_param(
        "~velocity_controllers",
        ["joint_group_vel_controller_l/unsafe", "joint_group_vel_controller_r/unsafe"],
    )
    moveit_controllers = rospy.get_param(
        "~moveit_controllers",
        ["UR10_l/arm_controller", "UR10_r/arm_controller"],
    )

    auto_load = bool(rospy.get_param("~auto_load", True))
    auto_switch_to_velocity = bool(rospy.get_param("~auto_switch_to_velocity", True))

    # Defaults aligned with our docs for MUR620:
    # - STRICT switching, timeout 0.0
    strictness = int(rospy.get_param("~strictness", SwitchControllerRequest.STRICT))
    timeout_s = float(rospy.get_param("~timeout_s", 0.0))

    list_srv_name = f"{cm_ns}/list_controllers"
    load_srv_name = f"{cm_ns}/load_controller"
    switch_srv_name = f"{cm_ns}/switch_controller"

    rospy.loginfo(f"[ensure] Waiting for controller_manager services under: {cm_ns}")
    rospy.wait_for_service(list_srv_name)
    rospy.wait_for_service(switch_srv_name)
    if auto_load:
        rospy.wait_for_service(load_srv_name)

    list_srv = rospy.ServiceProxy(list_srv_name, ListControllers)
    load_srv = rospy.ServiceProxy(load_srv_name, LoadController) if auto_load else None
    switch_srv = rospy.ServiceProxy(switch_srv_name, SwitchController)

    if auto_load and load_srv is not None:
        for name in velocity_controllers:
            _ensure_loaded(load_srv=load_srv, list_srv=list_srv, name=name)

    if auto_switch_to_velocity:
        # If we're already in velocity mode (all desired velocity controllers running), do nothing.
        resp = _list_controllers(list_srv)
        all_vel_running = all(_controller_running(resp, n) for n in velocity_controllers)
        if all_vel_running:
            rospy.loginfo("[switch] Velocity controllers already running. Nothing to do.")
            return

        # Perform a single atomic switch (recommended): stop both MoveIt controllers and start both velocity.
        _switch(
            switch_srv=switch_srv,
            start_controllers=velocity_controllers,
            stop_controllers=moveit_controllers,
            strictness=strictness,
            timeout_s=timeout_s,
        )


if __name__ == "__main__":
    main()

