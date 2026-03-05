#!/usr/bin/env python3
import sys
import rospy

import rosservice
from std_srvs.srv import Trigger
from ur_dashboard_msgs.srv import GetLoadedProgram


def _get_float_param(name: str, default: float) -> float:
    try:
        return float(rospy.get_param(name, default))
    except Exception:
        return float(default)


def _get_int_param(name: str, default: int) -> int:
    try:
        return int(rospy.get_param(name, default))
    except Exception:
        return int(default)


def _call_quit(service_name: str, resolved_name: str) -> None:
    srv_type = rosservice.get_service_type(service_name) or ""

    if srv_type == "std_srvs/Trigger":
        proxy = rospy.ServiceProxy(service_name, Trigger)
        resp = proxy()
        if resp.success:
            rospy.loginfo("Quit OK: %s", resp.message)
        else:
            rospy.logwarn("Quit returned success=false: %s", resp.message)
        return

    if srv_type == "ur_dashboard_msgs/GetLoadedProgram" or srv_type.endswith("/GetLoadedProgram"):
        proxy = rospy.ServiceProxy(service_name, GetLoadedProgram)
        resp = proxy()
        if resp.success:
            rospy.loginfo("Quit OK (answer='%s', program_name='%s')", resp.answer, resp.program_name)
        else:
            rospy.logwarn(
                "Quit returned success=false (answer='%s', program_name='%s')",
                resp.answer,
                resp.program_name,
            )
        return

    raise rospy.ServiceException(f"Unsupported quit service type '{srv_type}' for {resolved_name}")


def _call_connect(service_name: str, resolved_name: str) -> Trigger:
    srv_type = rosservice.get_service_type(service_name) or ""
    if srv_type != "" and srv_type != "std_srvs/Trigger":
        raise rospy.ServiceException(f"Unsupported connect service type '{srv_type}' for {resolved_name}")
    proxy = rospy.ServiceProxy(service_name, Trigger)
    return proxy()


def main() -> None:
    rospy.init_node("ur_dashboard_quit_connect", anonymous=False)

    quit_service = str(rospy.get_param("~quit_service", "dashboard/quit"))
    connect_service = str(rospy.get_param("~connect_service", "dashboard/connect"))

    wait_timeout_s = _get_float_param("~wait_timeout_s", 10.0)
    delay_before_quit_s = _get_float_param("~delay_before_quit_s", 0.0)
    delay_between_quit_connect_s = _get_float_param("~delay_between_quit_connect_s", 0.25)

    connect_retries = max(0, _get_int_param("~connect_retries", 1))
    retry_sleep_s = _get_float_param("~retry_sleep_s", 0.5)

    quit_service_resolved = rospy.resolve_name(quit_service)
    connect_service_resolved = rospy.resolve_name(connect_service)

    rospy.loginfo(
        "Waiting for services quit='%s' connect='%s' (timeout=%.3fs)",
        quit_service_resolved,
        connect_service_resolved,
        wait_timeout_s,
    )

    try:
        if wait_timeout_s <= 0.0:
            rospy.wait_for_service(quit_service)
            rospy.wait_for_service(connect_service)
        else:
            rospy.wait_for_service(quit_service, timeout=wait_timeout_s)
            rospy.wait_for_service(connect_service, timeout=wait_timeout_s)
    except rospy.ROSException as e:
        rospy.logerr("Service(s) not available: %s", str(e))
        sys.exit(2)

    if delay_before_quit_s > 0.0:
        rospy.sleep(delay_before_quit_s)

    try:
        _call_quit(quit_service, quit_service_resolved)
    except rospy.ServiceException as e:
        rospy.logwarn("Quit call to '%s' failed (will still try connect): %s", quit_service_resolved, str(e))

    if delay_between_quit_connect_s > 0.0:
        rospy.sleep(delay_between_quit_connect_s)

    last_message = ""
    for attempt in range(connect_retries + 1):
        try:
            connect_resp = _call_connect(connect_service, connect_service_resolved)
        except rospy.ServiceException as e:
            last_message = str(e)
            rospy.logwarn(
                "Connect call failed (attempt %d/%d): %s",
                attempt + 1,
                connect_retries + 1,
                last_message,
            )
            if attempt < connect_retries and retry_sleep_s > 0.0:
                rospy.sleep(retry_sleep_s)
            continue

        if connect_resp.success:
            rospy.loginfo("Connect OK: %s", connect_resp.message)
            sys.exit(0)

        last_message = connect_resp.message
        rospy.logwarn(
            "Connect returned success=false (attempt %d/%d): %s",
            attempt + 1,
            connect_retries + 1,
            last_message,
        )
        if attempt < connect_retries and retry_sleep_s > 0.0:
            rospy.sleep(retry_sleep_s)

    rospy.logerr("Connect failed after retries: %s", last_message)
    sys.exit(4)


if __name__ == "__main__":
    main()

