#!/usr/bin/env python3
import os
import sys
from dataclasses import dataclass
from typing import IO, List, Optional, Tuple, Union

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
    v = rospy.get_param(name, default)
    return str(v) if v is not None else str(default)


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


def _resolve_ns_prefix(ns: str) -> str:
    # Accept values like "mur620b/UR10_l/controller_manager" or "/mur620b/..."
    ns = str(ns).strip()
    if not ns:
        return ""
    if not ns.startswith("/"):
        ns = "/" + ns
    return ns.rstrip("/")


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


def _try_load(proxy: rospy.ServiceProxy, controller: str) -> bool:
    try:
        req = LoadControllerRequest(name=controller)
        resp = proxy(req)
        return bool(resp.ok)
    except rospy.ServiceException as e:
        rospy.logwarn("LoadController('%s') failed: %s", controller, str(e))
        return False


def _switch(
    proxy: rospy.ServiceProxy,
    start: List[str],
    stop: List[str],
    strictness: int,
    start_asap: bool,
    timeout_s: float,
) -> bool:
    req = SwitchControllerRequest()
    req.start_controllers = list(start)
    req.stop_controllers = list(stop)
    req.strictness = int(strictness)
    req.start_asap = bool(start_asap)
    req.timeout = float(timeout_s)
    try:
        resp = proxy(req)
        return bool(resp.ok)
    except rospy.ServiceException as e:
        rospy.logerr("SwitchController start=%s stop=%s failed: %s", start, stop, str(e))
        return False


def _list(proxy: rospy.ServiceProxy) -> Optional[List[Tuple[str, str]]]:
    try:
        resp = proxy(ListControllersRequest())
        out: List[Tuple[str, str]] = []
        for c in resp.controller:
            out.append((c.name, c.state))
        return out
    except rospy.ServiceException as e:
        rospy.logwarn("ListControllers failed: %s", str(e))
        return None


def _states_map(controllers: List[Tuple[str, str]]) -> dict:
    return {name: state for (name, state) in controllers}


def _ensure_controller_states(
    list_proxy: rospy.ServiceProxy,
    switch_proxy: rospy.ServiceProxy,
    *,
    must_run: List[str],
    must_stop: List[str],
    strictness: int,
    start_asap: bool,
    switch_timeout_s: float,
    verify_timeout_s: float,
    verify_period_s: float,
) -> bool:
    """
    Ensure required controller states, without failing on no-op requests.

    - Controllers in must_run are expected to be in state 'running'
    - Controllers in must_stop are expected to be NOT 'running' (missing is treated as not running)
    """
    controllers = _list(list_proxy)
    if controllers is None:
        # Fallback: we cannot compute a no-op-safe request nor verify. Try one switch request.
        rospy.logwarn("Cannot list controllers; attempting a direct switch request (no state verification).")
        return _switch(
            switch_proxy,
            start=list(must_run),
            stop=list(must_stop),
            strictness=strictness,
            start_asap=start_asap,
            timeout_s=switch_timeout_s,
        )

    states = _states_map(controllers)
    start = [c for c in must_run if states.get(c) != "running"]
    stop = [c for c in must_stop if states.get(c) == "running"]

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

    # If STRICT fails, retry BEST_EFFORT. This matches the intent "make sure states are correct"
    # more than "atomic switch must succeed".
    if not ok and int(strictness) == 2:
        rospy.logwarn("Strict controller switch failed; retrying with BEST_EFFORT.")
        ok = _switch(
            switch_proxy,
            start=start,
            stop=stop,
            strictness=1,
            start_asap=start_asap,
            timeout_s=switch_timeout_s,
        )

    # Verify resulting states (even if ok==False; the switch might have partially succeeded).
    if verify_timeout_s <= 0.0:
        return ok

    start_t = rospy.Time.now()
    last_states: Optional[dict] = None
    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - start_t).to_sec()
        if elapsed >= float(verify_timeout_s):
            break
        controllers = _list(list_proxy)
        if controllers is None:
            rospy.sleep(float(verify_period_s))
            continue
        last_states = _states_map(controllers)

        run_ok = all(last_states.get(c) == "running" for c in must_run)
        stop_ok = all(last_states.get(c) != "running" for c in must_stop)
        if run_ok and stop_ok:
            return True
        rospy.sleep(float(verify_period_s))

    if last_states is None:
        rospy.logwarn("Cannot verify controller states after switch; proceeding with switch result ok=%s.", ok)
        return ok

    rospy.logerr(
        "Controller state enforcement failed. must_run=%s must_stop=%s last_states=%s",
        must_run,
        must_stop,
        last_states,
    )
    return False


def _ensure_controller_loaded(
    list_proxy: rospy.ServiceProxy,
    load_proxy: rospy.ServiceProxy,
    name: str,
    auto_load: bool,
) -> None:
    if not auto_load:
        return
    controllers = _list(list_proxy)
    if controllers is None:
        _try_load(load_proxy, name)
        return

    if any(cn == name for (cn, _st) in controllers):
        return

    rospy.loginfo("Controller '%s' not listed; trying to load it.", name)
    _try_load(load_proxy, name)


def _build_default_robot_description_param(moveit_ns: str) -> str:
    ns = _resolve_ns_prefix(moveit_ns)
    if not ns:
        return "robot_description"
    return f"{ns}/robot_description"


def _acquire_lock(lock_file: str, wait_timeout_s: float) -> Optional[IO[str]]:
    """
    Cross-process mutex to avoid concurrent MoveIt execute_trajectory preemption.
    Uses a filesystem lock; lock is released when returned file is closed.
    """
    lock_file = str(lock_file).strip()
    if not lock_file:
        return None

    try:
        import fcntl
    except Exception as e:
        rospy.logwarn("Execution lock requested but fcntl not available: %s", str(e))
        return None

    lock_dir = os.path.dirname(lock_file) or "/tmp"
    try:
        os.makedirs(lock_dir, exist_ok=True)
    except Exception:
        pass

    f = open(lock_file, "a+")
    start_t = rospy.Time.now()
    warned = False
    while not rospy.is_shutdown():
        try:
            fcntl.flock(f.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            try:
                f.seek(0)
                f.truncate()
                f.write(f"pid={os.getpid()}\n")
                f.flush()
            except Exception:
                pass
            return f
        except BlockingIOError:
            if not warned:
                rospy.logwarn("Another go-home node is running; waiting for lock '%s'...", lock_file)
                warned = True

            if wait_timeout_s > 0.0:
                elapsed = (rospy.Time.now() - start_t).to_sec()
                if elapsed >= wait_timeout_s:
                    rospy.logerr("Timeout waiting for lock '%s' (%.3fs).", lock_file, elapsed)
                    try:
                        f.close()
                    except Exception:
                        pass
                    return None

            rospy.sleep(0.1)
        except Exception as e:
            rospy.logwarn("Failed to acquire lock '%s': %s", lock_file, str(e))
            try:
                f.close()
            except Exception:
                pass
            return None


def _plan_and_execute_named_pose(
    moveit_ns: str,
    robot_description_param: str,
    move_group_name: str,
    named_target: str,
    planning_time: float,
    num_planning_attempts: int,
    vel_scale: float,
    acc_scale: float,
    goal_tolerance: float,
    execute: bool,
) -> bool:
    try:
        import moveit_commander
    except Exception as e:
        rospy.logerr("Cannot import moveit_commander: %s", str(e))
        return False

    moveit_commander.roscpp_initialize(sys.argv)
    group = moveit_commander.MoveGroupCommander(
        move_group_name,
        robot_description=str(robot_description_param),
        ns=str(_resolve_ns_prefix(moveit_ns)),
    )

    group.set_planning_time(float(planning_time))
    group.set_num_planning_attempts(int(num_planning_attempts))
    group.set_max_velocity_scaling_factor(float(vel_scale))
    group.set_max_acceleration_scaling_factor(float(acc_scale))
    group.set_goal_joint_tolerance(float(goal_tolerance))
    group.set_start_state_to_current_state()

    try:
        group.set_named_target(str(named_target))
    except Exception as e:
        try:
            targets = group.get_named_targets()
        except Exception:
            targets = []
        rospy.logerr(
            "Named target '%s' not available for group '%s'. Available: %s. Error: %s",
            named_target,
            move_group_name,
            targets,
            str(e),
        )
        return False

    plan_res: Union[object, Tuple[bool, object, float, int]]
    try:
        plan_res = group.plan()
    except Exception as e:
        rospy.logerr("Planning to named target '%s' failed: %s", named_target, str(e))
        return False

    plan = plan_res[1] if isinstance(plan_res, tuple) and len(plan_res) >= 2 else plan_res
    if plan is None:
        rospy.logerr("Planning returned None for target '%s'.", named_target)
        return False

    if not execute:
        rospy.loginfo("Planning completed (execute:=false).")
        return True

    try:
        ok = bool(group.execute(plan, wait=True))
        group.stop()
        group.clear_pose_targets()
        return ok
    except Exception as e:
        rospy.logerr("Execution failed for target '%s': %s", named_target, str(e))
        return False


def main() -> None:
    rospy.init_node("moveit_go_home_and_switch", anonymous=False)

    # High-level config
    robot_ns = _get_str("~robot_ns", "")
    arm_ns = _get_str("~arm_ns", "")

    # Concurrency guard (default enabled): serialize two instances on the same robot.
    use_lock = _get_bool("~use_execution_lock", True)
    lock_file_default = (
        f"/tmp/teleoperation_moveit_go_home_{robot_ns}.lock" if robot_ns else "/tmp/teleoperation_moveit_go_home.lock"
    )
    lock_file = _get_str("~execution_lock_file", lock_file_default)
    lock_wait_timeout_s = _get_float("~execution_lock_wait_timeout_s", 0.0)

    # Controller switching config
    controller_manager_ns_default = "/".join([s for s in [robot_ns, arm_ns, "controller_manager"] if s])
    controller_manager_ns = _get_str("~controller_manager_ns", controller_manager_ns_default)
    cm = _cm_services(controller_manager_ns)

    twist_controller = _get_str("~twist_controller", "twist_controller")
    arm_controller = _get_str("~arm_controller", "arm_controller")
    force_torque_controller = _get_str(
        "~force_torque_controller", "force_torque_sensor_controller"
    )

    strictness = _get_int("~strictness", 2)
    start_asap = _get_bool("~start_asap", False)
    switch_timeout_s = _get_float("~switch_timeout_s", 0.0)
    wait_services_timeout_s = _get_float("~wait_services_timeout_s", 15.0)
    auto_load = _get_bool("~auto_load_controllers", True)
    controller_state_verify_timeout_s = _get_float("~controller_state_verify_timeout_s", 5.0)
    controller_state_verify_period_s = _get_float("~controller_state_verify_period_s", 0.1)

    # MoveIt config
    moveit_ns_default = f"/{robot_ns}" if robot_ns else ""
    moveit_ns = _get_str("~moveit_ns", moveit_ns_default)
    robot_description_param = _get_str(
        "~robot_description_param", _build_default_robot_description_param(moveit_ns)
    )
    move_group_name = _get_str("~move_group_name", "")
    home_pose_name = _get_str("~home_pose_name", "home")

    planning_time = _get_float("~planning_time", 5.0)
    num_planning_attempts = _get_int("~num_planning_attempts", 5)
    vel_scale = _get_float("~max_velocity_scaling_factor", 0.5)
    acc_scale = _get_float("~max_acceleration_scaling_factor", 0.5)
    goal_tolerance = _get_float("~goal_joint_tolerance", 0.001)
    execute = _get_bool("~execute", True)

    if not move_group_name:
        rospy.logerr("Missing required param '~move_group_name'.")
        sys.exit(2)

    # Wait for controller_manager services
    try:
        _wait_service(cm.switch_controller, wait_services_timeout_s)
        _wait_service(cm.list_controllers, wait_services_timeout_s)
        if auto_load:
            _wait_service(cm.load_controller, wait_services_timeout_s)
    except rospy.ROSException as e:
        rospy.logerr("Required controller_manager service not available: %s", str(e))
        sys.exit(3)

    list_proxy = rospy.ServiceProxy(cm.list_controllers, ListControllers)
    switch_proxy = rospy.ServiceProxy(cm.switch_controller, SwitchController)
    load_proxy = rospy.ServiceProxy(cm.load_controller, LoadController) if auto_load else None

    if auto_load and load_proxy is not None:
        _ensure_controller_loaded(list_proxy, load_proxy, twist_controller, auto_load=True)
        _ensure_controller_loaded(list_proxy, load_proxy, arm_controller, auto_load=True)
        _ensure_controller_loaded(
            list_proxy, load_proxy, force_torque_controller, auto_load=True
        )

    lock_handle: Optional[IO[str]] = None
    exit_code = 0
    try:
        if use_lock:
            lock_handle = _acquire_lock(lock_file, lock_wait_timeout_s)
            if lock_handle is None:
                rospy.logerr("Cannot continue without execution lock (use_execution_lock:=true).")
                sys.exit(7)

        # 1) Ensure twist stopped, arm + force/torque controllers running
        rospy.loginfo(
            "Ensuring controllers for go-home: arm='%s' and force_torque='%s' running, twist='%s' stopped (cm=%s)",
            arm_controller,
            force_torque_controller,
            twist_controller,
            _resolve_ns_prefix(controller_manager_ns),
        )
        if not _ensure_controller_states(
            list_proxy,
            switch_proxy,
            must_run=[arm_controller, force_torque_controller],
            must_stop=[twist_controller],
            strictness=strictness,
            start_asap=start_asap,
            switch_timeout_s=switch_timeout_s,
            verify_timeout_s=controller_state_verify_timeout_s,
            verify_period_s=controller_state_verify_period_s,
        ):
            rospy.logerr("Cannot enforce required controller states for go-home.")
            exit_code = 4
            return

        # 2) Plan + execute home
        rospy.loginfo(
            "MoveIt go-home: group='%s' named_target='%s' (moveit_ns='%s', robot_description='%s')",
            move_group_name,
            home_pose_name,
            _resolve_ns_prefix(moveit_ns),
            robot_description_param,
        )
        ok_home = _plan_and_execute_named_pose(
            moveit_ns=moveit_ns,
            robot_description_param=robot_description_param,
            move_group_name=move_group_name,
            named_target=home_pose_name,
            planning_time=planning_time,
            num_planning_attempts=num_planning_attempts,
            vel_scale=vel_scale,
            acc_scale=acc_scale,
            goal_tolerance=goal_tolerance,
            execute=execute,
        )
        if not ok_home:
            rospy.logerr(
                "Go-home failed. Attempting to restore twist and force_torque controllers anyway."
            )
            _ensure_controller_states(
                list_proxy,
                switch_proxy,
                must_run=[twist_controller, force_torque_controller],
                must_stop=[arm_controller],
                strictness=strictness,
                start_asap=start_asap,
                switch_timeout_s=switch_timeout_s,
                verify_timeout_s=controller_state_verify_timeout_s,
                verify_period_s=controller_state_verify_period_s,
            )
            exit_code = 5
            return

        # 3) Ensure arm stopped, twist + force/torque controllers running
        rospy.loginfo(
            "Ensuring controllers after go-home: twist='%s' and force_torque='%s' running, arm='%s' stopped",
            twist_controller,
            force_torque_controller,
            arm_controller,
        )
        if not _ensure_controller_states(
            list_proxy,
            switch_proxy,
            must_run=[twist_controller, force_torque_controller],
            must_stop=[arm_controller],
            strictness=strictness,
            start_asap=start_asap,
            switch_timeout_s=switch_timeout_s,
            verify_timeout_s=controller_state_verify_timeout_s,
            verify_period_s=controller_state_verify_period_s,
        ):
            rospy.logerr("Cannot enforce required controller states after go-home.")
            exit_code = 6
            return

        rospy.loginfo("Done.")
        exit_code = 0
    finally:
        if lock_handle is not None:
            try:
                lock_handle.close()
            except Exception:
                pass

    sys.exit(exit_code)


if __name__ == "__main__":
    main()

