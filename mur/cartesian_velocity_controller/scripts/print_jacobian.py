#!/usr/bin/env python3
"""
Pretty-print the Cartesian Jacobian returned by cartesian_velocity_controller's service.

Examples:
  rosrun cartesian_velocity_controller print_jacobian.py --service "~get_jacobian"
  rosrun cartesian_velocity_controller print_jacobian.py --service "/arm_r/cartesian_velocity_node/get_jacobian" -p 5
  rosrun cartesian_velocity_controller print_jacobian.py --service "/arm_l/.../get_jacobian" --csv

Compare two Jacobians (easy diff across two nodes/namespaces):
  rosrun cartesian_velocity_controller print_jacobian.py --service-a "/arm_l/.../get_jacobian" --service-b "/arm_r/.../get_jacobian" --diff
"""

from __future__ import annotations

import argparse
import math
import sys
from typing import List, Tuple

import rospy

from cartesian_velocity_controller.srv import GetJacobian, GetJacobianRequest


def _reshape_row_major(data: List[float], rows: int, cols: int) -> List[List[float]]:
    if rows <= 0 or cols <= 0:
        return []
    if len(data) != rows * cols:
        raise ValueError(f"data size mismatch: got {len(data)}, expected {rows*cols}")
    out: List[List[float]] = []
    for r in range(rows):
        out.append([float(data[r * cols + c]) for c in range(cols)])
    return out


def _format_matrix(m: List[List[float]], precision: int) -> str:
    if not m:
        return "<empty>"
    rows = len(m)
    cols = len(m[0]) if rows else 0

    # Pre-format to compute column widths (stable comparison between runs)
    formatted: List[List[str]] = []
    col_widths = [0] * cols
    fmt = f"{{:.{precision}f}}"

    for r in range(rows):
        row_strs: List[str] = []
        for c in range(cols):
            v = m[r][c]
            if not math.isfinite(v):
                s = "nan" if math.isnan(v) else ("inf" if v > 0 else "-inf")
            else:
                s = fmt.format(v)
            row_strs.append(s)
            col_widths[c] = max(col_widths[c], len(s))
        formatted.append(row_strs)

    lines: List[str] = []
    for r in range(rows):
        parts = [formatted[r][c].rjust(col_widths[c]) for c in range(cols)]
        lines.append("[ " + "  ".join(parts) + " ]")
    return "\n".join(lines)


def _format_csv(m: List[List[float]], precision: int) -> str:
    if not m:
        return ""
    fmt = f"{{:.{precision}f}}"
    lines: List[str] = []
    for row in m:
        lines.append(",".join(fmt.format(v) if math.isfinite(v) else str(v) for v in row))
    return "\n".join(lines)


def _call(service_name: str, timeout_s: float) -> Tuple[bool, str, str, str, List[List[float]]]:
    rospy.wait_for_service(service_name, timeout=timeout_s)
    proxy = rospy.ServiceProxy(service_name, GetJacobian)
    resp = proxy(GetJacobianRequest())
    if not resp.success:
        return False, resp.message, resp.frame_id, resp.tcp_link, []
    mat = _reshape_row_major(list(resp.data), int(resp.rows), int(resp.cols))
    return True, resp.message, resp.frame_id, resp.tcp_link, mat


def _max_abs_diff(a: List[List[float]], b: List[List[float]]) -> float:
    if not a or not b:
        return float("nan")
    if len(a) != len(b) or len(a[0]) != len(b[0]):
        return float("nan")
    m = 0.0
    for r in range(len(a)):
        for c in range(len(a[0])):
            da = abs(a[r][c] - b[r][c])
            if da > m:
                m = da
    return m


def main() -> int:
    parser = argparse.ArgumentParser()
    g = parser.add_mutually_exclusive_group(required=False)
    g.add_argument("--service", help="GetJacobian service name (e.g. ~get_jacobian)")
    g.add_argument("--service-a", help="First service (for compare mode)")
    parser.add_argument("--service-b", help="Second service (for compare mode)")

    parser.add_argument("-p", "--precision", type=int, default=4, help="Decimal digits (default: 4)")
    parser.add_argument("--timeout", type=float, default=2.0, help="Service wait timeout seconds (default: 2.0)")
    parser.add_argument("--csv", action="store_true", help="Print matrix as CSV (easier diff)")
    parser.add_argument("--diff", action="store_true", help="Compare A vs B and print max|A-B|")

    args = parser.parse_args()

    rospy.init_node("print_jacobian", anonymous=True, disable_signals=True)

    # Single service mode
    if args.service:
        ok, msg, frame_id, tcp_link, mat = _call(args.service, args.timeout)
        if not ok:
            print(f"ERROR: {msg}")
            return 2
        print(f"service:  {args.service}")
        print(f"frame_id: {frame_id}")
        print(f"tcp_link: {tcp_link}")
        print(f"size:     {len(mat)} x {len(mat[0]) if mat else 0}")
        print("")
        print(_format_csv(mat, args.precision) if args.csv else _format_matrix(mat, args.precision))
        return 0

    # Compare mode
    if args.service_a and args.service_b:
        ok_a, msg_a, frame_a, tcp_a, mat_a = _call(args.service_a, args.timeout)
        ok_b, msg_b, frame_b, tcp_b, mat_b = _call(args.service_b, args.timeout)
        if not ok_a:
            print(f"ERROR A: {msg_a}")
            return 2
        if not ok_b:
            print(f"ERROR B: {msg_b}")
            return 2

        print(f"A service: {args.service_a}")
        print(f"B service: {args.service_b}")
        print(f"A frame_id: {frame_a}   tcp_link: {tcp_a}")
        print(f"B frame_id: {frame_b}   tcp_link: {tcp_b}")
        print(f"A size: {len(mat_a)} x {len(mat_a[0]) if mat_a else 0}")
        print(f"B size: {len(mat_b)} x {len(mat_b[0]) if mat_b else 0}")
        print("")

        if args.diff:
            d = _max_abs_diff(mat_a, mat_b)
            if math.isnan(d):
                print("max|A-B|: nan (size mismatch or empty matrix)")
            else:
                print(f"max|A-B|: {d:.{args.precision}g}")
            print("")

        # Print both matrices in stable format
        print("A:")
        print(_format_csv(mat_a, args.precision) if args.csv else _format_matrix(mat_a, args.precision))
        print("")
        print("B:")
        print(_format_csv(mat_b, args.precision) if args.csv else _format_matrix(mat_b, args.precision))
        return 0

    parser.print_help()
    return 1


if __name__ == "__main__":
    raise SystemExit(main())

