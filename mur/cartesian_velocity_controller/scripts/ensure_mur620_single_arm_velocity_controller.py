#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Ensure/load velocity controller for a single arm and optionally switch at startup (ROS1).

This is meant to be launched from a roslaunch file under the robot namespace,
e.g. /mur620, and will:
  - wait for controller_manager services
  - ensure the configured controller is loaded (typically left or right unsafe)
  - optionally switch from MoveIt controller to velocity controller

It is designed to be safe if controllers are already loaded/running.
"""

from __future__ import annotations

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
    start_controller: str,
    stop_controller: str,
    strictness: int,
    timeout_s: float,
) -> bool:
    req = SwitchControllerRequest()
    req.start_controllers = [start_controller] if start_controller else []
    req.stop_controllers = [stop_controller] if stop_controller else []
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
            f"started=[{start_controller}] stopped=[{stop_controller}]"
        )
        return True

    rospy.logerr(
        "[switch] switch_controller returned ok=false. "
        "Make sure controllers are loaded and not conflicting. "
        f"start=[{start_controller}] stop=[{stop_controller}]"
    )
    return False


def main() -> None:
    rospy.init_node("ensure_mur620_single_arm_velocity_controller", anonymous=True)

    controller_manager_ns = rospy.get_param("~controller_manager_ns", "controller_manager")
    cm_ns = _resolve_cm_ns(controller_manager_ns)

    arm = rospy.get_param("~arm", "left")
    arm_suffix = "l" if arm == "left" else "r"

    # Single controller names (can be overridden)
    velocity_controller = rospy.get_param(
        "~velocity_controller",
        f"joint_group_vel_controller_{arm_suffix}/unsafe",
    )
    moveit_controller = rospy.get_param(
        "~moveit_controller",
        f"UR10_{arm_suffix}/arm_controller",
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

    rospy.loginfo(f"[ensure] Single arm mode: {arm} (suffix: {arm_suffix})")
    rospy.loginfo(f"[ensure] Waiting for controller_manager services under: {cm_ns}")
    rospy.wait_for_service(list_srv_name)
    rospy.wait_for_service(switch_srv_name)
    if auto_load:
        rospy.wait_for_service(load_srv_name)

    list_srv = rospy.ServiceProxy(list_srv_name, ListControllers)
    load_srv = rospy.ServiceProxy(load_srv_name, LoadController) if auto_load else None
    switch_srv = rospy.ServiceProxy(switch_srv_name, SwitchController)

    if auto_load and load_srv is not None:
        _ensure_loaded(load_srv=load_srv, list_srv=list_srv, name=velocity_controller)

    if auto_switch_to_velocity:
        # If we're already in velocity mode (velocity controller running), do nothing.
        resp = _list_controllers(list_srv)
        if _controller_running(resp, velocity_controller):
            rospy.loginfo("[switch] Velocity controller already running. Nothing to do.")
            return

        # Perform switch: stop MoveIt controller and start velocity controller.
        _switch(
            switch_srv=switch_srv,
            start_controller=velocity_controller,
            stop_controller=moveit_controller,
            strictness=strictness,
            timeout_s=timeout_s,
        )


if __name__ == "__main__":
    main()
