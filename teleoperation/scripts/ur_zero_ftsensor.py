#!/usr/bin/env python3
import sys
import rospy

from std_srvs.srv import Trigger


def _get_float_param(name: str, default: float) -> float:
    try:
        return float(rospy.get_param(name, default))
    except Exception:
        return float(default)


def main() -> None:
    rospy.init_node("ur_zero_ftsensor", anonymous=False)

    service_name = str(rospy.get_param("~service", "zero_ftsensor"))
    wait_timeout_s = _get_float_param("~wait_timeout_s", 10.0)
    delay_s = _get_float_param("~delay_s", 0.5)

    resolved_service = rospy.resolve_name(service_name)
    rospy.loginfo("Waiting for service '%s' (timeout=%.3fs)", resolved_service, wait_timeout_s)

    try:
        if wait_timeout_s <= 0.0:
            rospy.wait_for_service(service_name)
        else:
            rospy.wait_for_service(service_name, timeout=wait_timeout_s)
    except rospy.ROSException as e:
        rospy.logerr("Service '%s' not available: %s", resolved_service, str(e))
        sys.exit(2)

    if delay_s > 0.0:
        rospy.sleep(delay_s)

    try:
        proxy = rospy.ServiceProxy(service_name, Trigger)
        resp = proxy()
    except rospy.ServiceException as e:
        rospy.logerr("Service call to '%s' failed: %s", resolved_service, str(e))
        sys.exit(3)

    if resp.success:
        rospy.loginfo("FT sensor zeroed successfully: %s", resp.message)
        sys.exit(0)
    else:
        rospy.logwarn("FT sensor zero request returned success=false: %s", resp.message)
        sys.exit(4)


if __name__ == "__main__":
    main()

