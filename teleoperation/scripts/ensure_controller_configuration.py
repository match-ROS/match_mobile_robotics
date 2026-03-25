#!/usr/bin/env python3
import sys
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import rospy
from controller_manager_msgs.srv import (
    ListControllers,
    ListControllersRequest,
    LoadController,
    LoadControllerRequest,
    SwitchController,
    SwitchControllerRequest,
)


def _get_str(name: str, default: str) -> str:
    value = rospy.get_param(name, default)
    return str(value) if value is not None else str(default)


def _get_bool(name: str, default: bool) -> bool:
    try:
        return bool(rospy.get_param(name, default))
    except Exception:
        return bool(default)


def _get_int(name: str, default: int) -> int:
    try:
        return int(rospy.get_param(name, default))
    except Exception:
        return int(default)


def _get_float(name: str, default: float) -> float:
    try:
        return float(rospy.get_param(name, default))
    except Exception:
        return float(default)


def _get_list(name: str, default: Sequence[str]) -> List[str]:
    value = rospy.get_param(name, list(default))
    if isinstance(value, str):
        items = [item.strip() for item in value.split(",")]
        return [item for item in items if item]
    if isinstance(value, (list, tuple)):
        return [str(item).strip() for item in value if str(item).strip()]
    return [str(item).strip() for item in default if str(item).strip()]


def _resolve_ns_prefix(ns: str) -> str:
    ns = str(ns).strip()
    if not ns:
        return ""
    if not ns.startswith("/"):
        ns = "/" + ns
    return ns.rstrip("/")


def _matches(configured: str, reported: str) -> bool:
    if configured == reported:
        return True
    return reported.endswith(f"/{configured}")


def _unique_preserve_order(items: Sequence[str]) -> List[str]:
    out: List[str] = []
    seen = set()
    for item in items:
        item = str(item).strip()
        if not item or item in seen:
            continue
        out.append(item)
        seen.add(item)
    return out


@dataclass(frozen=True)
class ControllerManagerServices:
    list_controllers: str
    load_controller: str
    switch_controller: str


def _cm_services(controller_manager_ns: str) -> ControllerManagerServices:
    cm = _resolve_ns_prefix(controller_manager_ns)
    return ControllerManagerServices(
        list_controllers=f"{cm}/list_controllers",
        load_controller=f"{cm}/load_controller",
        switch_controller=f"{cm}/switch_controller",
    )


def _wait_service(name: str, timeout_s: float) -> None:
    resolved = rospy.resolve_name(name)
    if timeout_s <= 0.0:
        rospy.loginfo("Waiting for service '%s' (no timeout)", resolved)
        rospy.wait_for_service(name)
        return
    rospy.loginfo("Waiting for service '%s' (timeout=%.3fs)", resolved, timeout_s)
    rospy.wait_for_service(name, timeout=timeout_s)


def _list(proxy: rospy.ServiceProxy) -> Optional[List[Tuple[str, str]]]:
    try:
        response = proxy(ListControllersRequest())
        return [(controller.name, controller.state) for controller in response.controller]
    except rospy.ServiceException as exc:
        rospy.logwarn("ListControllers failed: %s", str(exc))
        return None


def _controller_present(controllers: Sequence[Tuple[str, str]], name: str) -> bool:
    return any(_matches(name, controller_name) for controller_name, _state in controllers)


def _controller_running(controllers: Sequence[Tuple[str, str]], name: str) -> bool:
    return any(
        _matches(name, controller_name) and state == "running"
        for controller_name, state in controllers
    )


def _try_load(proxy: rospy.ServiceProxy, controller: str) -> bool:
    try:
        response = proxy(LoadControllerRequest(name=controller))
        return bool(response.ok)
    except rospy.ServiceException as exc:
        rospy.logwarn("LoadController('%s') failed: %s", controller, str(exc))
        return False


def _ensure_controller_loaded(
    list_proxy: rospy.ServiceProxy,
    load_proxy: rospy.ServiceProxy,
    controller: str,
    auto_load: bool,
) -> None:
    if not auto_load:
        return

    controllers = _list(list_proxy)
    if controllers is not None and _controller_present(controllers, controller):
        return

    rospy.loginfo("Controller '%s' not listed; trying to load it.", controller)
    _try_load(load_proxy, controller)


def _switch(
    proxy: rospy.ServiceProxy,
    start: Sequence[str],
    stop: Sequence[str],
    strictness: int,
    start_asap: bool,
    timeout_s: float,
) -> bool:
    request = SwitchControllerRequest()
    request.start_controllers = list(start)
    request.stop_controllers = list(stop)
    request.strictness = int(strictness)
    request.start_asap = bool(start_asap)
    request.timeout = float(timeout_s)
    try:
        response = proxy(request)
        return bool(response.ok)
    except rospy.ServiceException as exc:
        rospy.logerr("SwitchController start=%s stop=%s failed: %s", start, stop, str(exc))
        return False


def _ensure_controller_states(
    list_proxy: rospy.ServiceProxy,
    switch_proxy: rospy.ServiceProxy,
    *,
    must_run: Sequence[str],
    must_stop: Sequence[str],
    strictness: int,
    start_asap: bool,
    switch_timeout_s: float,
    verify_timeout_s: float,
    verify_period_s: float,
) -> bool:
    controllers = _list(list_proxy)
    if controllers is None:
        rospy.logwarn("Cannot list controllers; attempting a direct switch request.")
        return _switch(
            switch_proxy,
            start=must_run,
            stop=must_stop,
            strictness=strictness,
            start_asap=start_asap,
            timeout_s=switch_timeout_s,
        )

    start = [name for name in must_run if not _controller_running(controllers, name)]
    stop = [name for name in must_stop if _controller_running(controllers, name)]

    if not start and not stop:
        return True

    ok = _switch(
        switch_proxy,
        start=start,
        stop=stop,
        strictness=strictness,
        start_asap=start_asap,
        timeout_s=switch_timeout_s,
    )

    if not ok and int(strictness) == SwitchControllerRequest.STRICT:
        rospy.logwarn("Strict controller switch failed; retrying with BEST_EFFORT.")
        ok = _switch(
            switch_proxy,
            start=start,
            stop=stop,
            strictness=SwitchControllerRequest.BEST_EFFORT,
            start_asap=start_asap,
            timeout_s=switch_timeout_s,
        )

    if verify_timeout_s <= 0.0:
        return ok

    start_t = rospy.Time.now()
    last_states: Optional[List[Tuple[str, str]]] = None
    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - start_t).to_sec()
        if elapsed >= float(verify_timeout_s):
            break

        controllers = _list(list_proxy)
        if controllers is None:
            rospy.sleep(float(verify_period_s))
            continue

        last_states = controllers
        run_ok = all(_controller_running(controllers, name) for name in must_run)
        stop_ok = all(not _controller_running(controllers, name) for name in must_stop)
        if run_ok and stop_ok:
            return True

        rospy.sleep(float(verify_period_s))

    if last_states is None:
        rospy.logwarn("Cannot verify controller states after switch; proceeding with ok=%s.", ok)
        return ok

    rospy.logerr(
        "Controller state enforcement failed. must_run=%s must_stop=%s last_states=%s",
        list(must_run),
        list(must_stop),
        last_states,
    )
    return False


def main() -> None:
    rospy.init_node("ensure_controller_configuration", anonymous=False)

    robot_ns = _get_str("~robot_ns", "")
    arm_ns = _get_str("~arm_ns", "")

    controller_manager_ns_default = "/".join(
        [part for part in [robot_ns, arm_ns, "controller_manager"] if part]
    )
    controller_manager_ns = _get_str("~controller_manager_ns", controller_manager_ns_default)
    services = _cm_services(controller_manager_ns)

    running_controllers = _get_list(
        "~running_controllers",
        ["twist_controller", "force_torque_sensor_controller"],
    )
    stopped_controllers = _get_list("~stopped_controllers", ["arm_controller"])
    controllers_to_load = _get_list(
        "~controllers_to_load",
        _unique_preserve_order(list(running_controllers) + list(stopped_controllers)),
    )

    strictness = _get_int("~strictness", SwitchControllerRequest.STRICT)
    start_asap = _get_bool("~start_asap", False)
    switch_timeout_s = _get_float("~switch_timeout_s", 0.0)
    wait_services_timeout_s = _get_float("~wait_services_timeout_s", 15.0)
    auto_load = _get_bool("~auto_load_controllers", True)
    verify_timeout_s = _get_float("~controller_state_verify_timeout_s", 5.0)
    verify_period_s = _get_float("~controller_state_verify_period_s", 0.1)

    try:
        _wait_service(services.switch_controller, wait_services_timeout_s)
        _wait_service(services.list_controllers, wait_services_timeout_s)
        if auto_load:
            _wait_service(services.load_controller, wait_services_timeout_s)
    except rospy.ROSException as exc:
        rospy.logerr("Required controller_manager service not available: %s", str(exc))
        sys.exit(3)

    list_proxy = rospy.ServiceProxy(services.list_controllers, ListControllers)
    switch_proxy = rospy.ServiceProxy(services.switch_controller, SwitchController)
    load_proxy = rospy.ServiceProxy(services.load_controller, LoadController) if auto_load else None

    if auto_load and load_proxy is not None:
        for controller in controllers_to_load:
            _ensure_controller_loaded(list_proxy, load_proxy, controller, auto_load=True)

    rospy.loginfo(
        "Ensuring controller configuration on cm='%s': running=%s stopped=%s",
        _resolve_ns_prefix(controller_manager_ns),
        running_controllers,
        stopped_controllers,
    )

    if not _ensure_controller_states(
        list_proxy,
        switch_proxy,
        must_run=running_controllers,
        must_stop=stopped_controllers,
        strictness=strictness,
        start_asap=start_asap,
        switch_timeout_s=switch_timeout_s,
        verify_timeout_s=verify_timeout_s,
        verify_period_s=verify_period_s,
    ):
        sys.exit(4)

    final_states = _list(list_proxy)
    if final_states is not None:
        rospy.loginfo("Final controller states on '%s': %s", _resolve_ns_prefix(controller_manager_ns), final_states)

    rospy.loginfo("Controller configuration applied successfully.")
    sys.exit(0)


if __name__ == "__main__":
    main()
