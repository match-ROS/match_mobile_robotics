#!/usr/bin/env python3
import bisect
import math
import os
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rosbag
import rospy
import tf2_ros
import yaml

from controller_manager_msgs.srv import (
    ListControllers,
    ListControllersRequest,
    LoadController,
    LoadControllerRequest,
    SwitchController,
    SwitchControllerRequest,
)
from geometry_msgs.msg import Twist, WrenchStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger


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


def _get_optional_float(name: str) -> Optional[float]:
    if not rospy.has_param(name):
        return None
    value = rospy.get_param(name)
    if value is None:
        return None
    text = str(value).strip()
    if not text or text.lower() in ("manifest", "none"):
        return None
    return float(text)


def _resolve_ns(ns: str) -> str:
    ns = str(ns).strip()
    if not ns:
        return ""
    if not ns.startswith("/"):
        ns = "/" + ns
    return ns.rstrip("/")


def _norm3(v) -> float:
    return math.sqrt(float(v.x) * float(v.x) + float(v.y) * float(v.y) + float(v.z) * float(v.z))


def _quat_angle_xyzw(q1: List[float], q2: List[float]) -> float:
    dot = abs(sum(float(a) * float(b) for a, b in zip(q1, q2)))
    dot = min(1.0, max(-1.0, dot))
    return 2.0 * math.acos(dot)


def _zero_twist() -> Twist:
    return Twist()


def _twist_values(msg) -> Tuple[float, float, float, float, float, float]:
    src = msg.twist if hasattr(msg, "twist") else msg
    return (
        float(src.linear.x),
        float(src.linear.y),
        float(src.linear.z),
        float(src.angular.x),
        float(src.angular.y),
        float(src.angular.z),
    )


def _twist_from_values(values: Tuple[float, float, float, float, float, float]) -> Twist:
    out = Twist()
    out.linear.x = values[0]
    out.linear.y = values[1]
    out.linear.z = values[2]
    out.angular.x = values[3]
    out.angular.y = values[4]
    out.angular.z = values[5]
    return out


@dataclass
class TwistSample:
    rel_t: float
    values: Tuple[float, float, float, float, float, float]


@dataclass
class ScaledReplay:
    output_dt_s: float
    duration_s: float
    commands_by_side: Dict[str, List[Tuple[float, float, float, float, float, float]]]
    speed_mode: str = "fixed"
    effective_speed_scale_min: float = 0.0
    effective_speed_scale_mean: float = 0.0
    effective_speed_scale_max: float = 0.0

    @property
    def tick_count(self) -> int:
        if not self.commands_by_side:
            return 0
        return len(next(iter(self.commands_by_side.values())))


@dataclass
class SourceInterval:
    start_s: float
    end_s: float
    values_by_side: Dict[str, Tuple[float, float, float, float, float, float]]

    @property
    def duration_s(self) -> float:
        return self.end_s - self.start_s


@dataclass
class AdaptiveLimitViolation:
    ratio: float
    source_start_s: float
    source_end_s: float
    tick: int
    side: str
    kind: str
    value: float
    limit: float


def _resample_time_scaled_twists(
    samples: List[TwistSample],
    source_duration_s: float,
    speed_scale: float,
    output_rate_hz: float,
) -> Tuple[float, List[Tuple[float, float, float, float, float, float]]]:
    if source_duration_s <= 0.0:
        raise ReplayAbort("Invalid source duration for replay")
    if speed_scale <= 0.0:
        raise ReplayAbort("speed_scale must be > 0")
    if output_rate_hz <= 0.0:
        raise ReplayAbort("replay_rate_hz must be > 0")
    if not samples:
        raise ReplayAbort("No twist samples available for resampling")

    eps = 1e-12
    output_dt_s = 1.0 / output_rate_hz
    output_duration_s = source_duration_s / speed_scale
    tick_count = max(1, int(math.ceil(output_duration_s / output_dt_s - eps)))

    clean: List[TwistSample] = []
    for sample in sorted(samples, key=lambda item: item.rel_t):
        rel_t = min(max(float(sample.rel_t), 0.0), source_duration_s)
        item = TwistSample(rel_t=rel_t, values=sample.values)
        if clean and abs(rel_t - clean[-1].rel_t) <= eps:
            clean[-1] = item
        else:
            clean.append(item)

    last_sample_end_s = min(source_duration_s, clean[-1].rel_t + output_dt_s)
    values: List[Tuple[float, float, float, float, float, float]] = []
    source_index = 0
    for tick in range(tick_count):
        source_a = speed_scale * tick * output_dt_s
        source_b = min(source_duration_s, speed_scale * (tick + 1) * output_dt_s)
        acc = [0.0] * 6

        while source_index < len(clean):
            next_t = clean[source_index + 1].rel_t if source_index + 1 < len(clean) else last_sample_end_s
            if next_t > source_a + eps:
                break
            source_index += 1

        idx = source_index
        while idx < len(clean):
            start_t = clean[idx].rel_t
            end_t = clean[idx + 1].rel_t if idx + 1 < len(clean) else last_sample_end_s
            if start_t >= source_b - eps:
                break
            overlap = min(source_b, end_t) - max(source_a, start_t)
            if overlap > eps:
                for component in range(6):
                    acc[component] += overlap * clean[idx].values[component]
            idx += 1

        values.append(tuple(component / output_dt_s for component in acc))

    return output_dt_s, values


def _clean_twist_samples(
    samples: List[TwistSample],
    source_duration_s: float,
    eps: float,
) -> List[TwistSample]:
    clean: List[TwistSample] = []
    for sample in sorted(samples, key=lambda item: item.rel_t):
        rel_t = min(max(float(sample.rel_t), 0.0), source_duration_s)
        item = TwistSample(rel_t=rel_t, values=sample.values)
        if clean and abs(rel_t - clean[-1].rel_t) <= eps:
            clean[-1] = item
        else:
            clean.append(item)
    return clean


def _values_norm(values: Tuple[float, float, float, float, float, float], start: int) -> float:
    return math.sqrt(
        values[start] * values[start]
        + values[start + 1] * values[start + 1]
        + values[start + 2] * values[start + 2]
    )


def _apply_common_scale(
    values: Tuple[float, float, float, float, float, float],
    scale: float,
) -> Tuple[float, float, float, float, float, float]:
    return tuple(scale * value for value in values)


def _component_limit_scale(
    values: Tuple[float, float, float, float, float, float],
    linear_limit: float,
    angular_limit: float,
) -> float:
    scale = 1.0
    for component in range(3):
        value = abs(values[component])
        if linear_limit > 0.0 and value > linear_limit and value * scale > linear_limit:
            scale = linear_limit / value
    for component in range(3, 6):
        value = abs(values[component])
        if angular_limit > 0.0 and value > angular_limit and value * scale > angular_limit:
            scale = angular_limit / value
    return scale


def _simulate_ur_twist_limiter_samples(
    samples_by_side: Dict[str, List[TwistSample]],
    linear_speed_limit: float,
    angular_speed_limit: float,
    linear_accel_step_limit: float,
    angular_accel_step_limit: float,
    linear_jerk_step_limit: float,
    angular_jerk_step_limit: float,
) -> Dict[str, List[TwistSample]]:
    filtered: Dict[str, List[TwistSample]] = {}
    zero = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    for side, samples in samples_by_side.items():
        last_command = zero
        last_acc = zero
        side_filtered: List[TwistSample] = []
        for sample in sorted(samples, key=lambda item: item.rel_t):
            output = _apply_common_scale(
                sample.values,
                _component_limit_scale(sample.values, linear_speed_limit, angular_speed_limit),
            )

            acc = tuple(output[i] - last_command[i] for i in range(6))
            acc = _apply_common_scale(
                acc,
                _component_limit_scale(acc, linear_accel_step_limit, angular_accel_step_limit),
            )

            jerk = tuple(acc[i] - last_acc[i] for i in range(6))
            jerk_scale = _component_limit_scale(
                jerk,
                linear_jerk_step_limit,
                angular_jerk_step_limit,
            )
            acc = tuple(last_acc[i] + jerk_scale * jerk[i] for i in range(6))
            output = tuple(last_command[i] + acc[i] for i in range(6))

            side_filtered.append(TwistSample(rel_t=sample.rel_t, values=output))
            last_command = output
            last_acc = acc
        filtered[side] = side_filtered

    return filtered


def _append_unique_time(times: List[float], value: float, eps: float) -> None:
    value = float(value)
    if value < 0.0:
        value = 0.0
    times.append(value)


def _unique_sorted_times(times: List[float], source_duration_s: float, eps: float) -> List[float]:
    out: List[float] = []
    for value in sorted(min(max(float(t), 0.0), source_duration_s) for t in times):
        if not out or abs(value - out[-1]) > eps:
            out.append(value)
        else:
            out[-1] = value
    if not out or out[0] > eps:
        out.insert(0, 0.0)
    else:
        out[0] = 0.0
    if out[-1] < source_duration_s - eps:
        out.append(source_duration_s)
    else:
        out[-1] = source_duration_s
    return out


def _build_source_intervals(
    samples_by_side: Dict[str, List[TwistSample]],
    source_duration_s: float,
    output_dt_s: float,
    eps: float,
) -> List[SourceInterval]:
    clean_by_side = {
        side: _clean_twist_samples(side_samples, source_duration_s, eps)
        for side, side_samples in samples_by_side.items()
    }
    if any(not side_samples for side_samples in clean_by_side.values()):
        raise ReplayAbort("No twist samples available for adaptive resampling")

    last_end_by_side: Dict[str, float] = {}
    breakpoints: List[float] = [0.0, source_duration_s]
    for side, clean in clean_by_side.items():
        last_end_by_side[side] = min(source_duration_s, clean[-1].rel_t + output_dt_s)
        for sample in clean:
            _append_unique_time(breakpoints, sample.rel_t, eps)
        _append_unique_time(breakpoints, last_end_by_side[side], eps)

    times = _unique_sorted_times(breakpoints, source_duration_s, eps)
    indices = {side: -1 for side in clean_by_side}
    intervals: List[SourceInterval] = []
    zero = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    for idx in range(len(times) - 1):
        start_s = times[idx]
        end_s = times[idx + 1]
        if end_s <= start_s + eps:
            continue

        values_by_side: Dict[str, Tuple[float, float, float, float, float, float]] = {}
        for side, clean in clean_by_side.items():
            sample_idx = indices[side]
            while sample_idx + 1 < len(clean) and clean[sample_idx + 1].rel_t <= start_s + eps:
                sample_idx += 1
            indices[side] = sample_idx

            if sample_idx < 0:
                values_by_side[side] = zero
                continue

            next_s = (
                clean[sample_idx + 1].rel_t
                if sample_idx + 1 < len(clean)
                else last_end_by_side[side]
            )
            values_by_side[side] = clean[sample_idx].values if start_s < next_s - eps else zero

        intervals.append(SourceInterval(start_s=start_s, end_s=end_s, values_by_side=values_by_side))

    if not intervals:
        raise ReplayAbort("Adaptive replay produced no source intervals")
    return intervals


def _adaptive_velocity_limited_scales(
    intervals: List[SourceInterval],
    max_speed_scale: float,
    max_linear_speed: float,
    max_angular_speed: float,
    min_required_speed_scale: float,
    eps: float,
) -> List[float]:
    scales: List[float] = []
    for interval in intervals:
        scale = max_speed_scale
        for values in interval.values_by_side.values():
            lin_norm = _values_norm(values, 0)
            ang_norm = _values_norm(values, 3)
            if max_linear_speed > 0.0 and lin_norm > eps:
                scale = min(scale, max_linear_speed / lin_norm)
            if max_angular_speed > 0.0 and ang_norm > eps:
                scale = min(scale, max_angular_speed / ang_norm)

        if min_required_speed_scale > 0.0 and scale < min_required_speed_scale - eps:
            raise ReplayAbort(
                "Adaptive velocity limits require speed_scale %.6g below adaptive_min_speed_scale %.6g"
                % (scale, min_required_speed_scale)
            )
        scales.append(max(eps, scale))
    return scales


def _advance_adaptive_source(
    intervals: List[SourceInterval],
    scales: List[float],
    source_s: float,
    interval_idx: int,
    wall_dt_s: float,
    source_duration_s: float,
    eps: float,
) -> Tuple[float, int]:
    remaining_wall_s = wall_dt_s
    current_s = source_s
    idx = interval_idx

    while remaining_wall_s > eps and current_s < source_duration_s - eps:
        while idx < len(intervals) and intervals[idx].end_s <= current_s + eps:
            idx += 1
        if idx >= len(intervals):
            return source_duration_s, idx

        interval = intervals[idx]
        source_step_to_end = interval.end_s - current_s
        if source_step_to_end <= eps:
            idx += 1
            continue

        scale = max(scales[idx], eps)
        wall_to_end_s = source_step_to_end / scale
        if wall_to_end_s > remaining_wall_s + eps:
            current_s += remaining_wall_s * scale
            remaining_wall_s = 0.0
        else:
            current_s = interval.end_s
            remaining_wall_s -= wall_to_end_s
            idx += 1

    return min(current_s, source_duration_s), idx


def _integrate_adaptive_interval(
    intervals: List[SourceInterval],
    side: str,
    source_a_s: float,
    source_b_s: float,
    interval_idx: int,
    eps: float,
) -> Tuple[Tuple[float, float, float, float, float, float], int]:
    if source_b_s <= source_a_s + eps:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), interval_idx

    acc = [0.0] * 6
    idx = interval_idx
    while idx < len(intervals) and intervals[idx].end_s <= source_a_s + eps:
        idx += 1
    start_idx = idx

    while idx < len(intervals):
        interval = intervals[idx]
        if interval.start_s >= source_b_s - eps:
            break
        overlap = min(source_b_s, interval.end_s) - max(source_a_s, interval.start_s)
        if overlap > eps:
            values = interval.values_by_side[side]
            for component in range(6):
                acc[component] += overlap * values[component]
        idx += 1

    return tuple(acc), start_idx


def _render_adaptive_time_scaled_twists(
    intervals: List[SourceInterval],
    scales: List[float],
    output_dt_s: float,
    source_duration_s: float,
    side_names: List[str],
    eps: float,
) -> Tuple[
    Dict[str, List[Tuple[float, float, float, float, float, float]]],
    List[Tuple[float, float]],
]:
    commands_by_side: Dict[str, List[Tuple[float, float, float, float, float, float]]] = {
        side: [] for side in side_names
    }
    source_bounds: List[Tuple[float, float]] = []
    source_a_s = 0.0
    profile_idx = 0
    integral_idx_by_side = {side: 0 for side in side_names}

    while source_a_s < source_duration_s - eps:
        source_b_s, profile_idx = _advance_adaptive_source(
            intervals,
            scales,
            source_a_s,
            profile_idx,
            output_dt_s,
            source_duration_s,
            eps,
        )
        if source_b_s <= source_a_s + eps:
            raise ReplayAbort("Adaptive replay source time stopped progressing")

        for side in side_names:
            integral, integral_idx = _integrate_adaptive_interval(
                intervals,
                side,
                source_a_s,
                source_b_s,
                integral_idx_by_side[side],
                eps,
            )
            integral_idx_by_side[side] = integral_idx
            commands_by_side[side].append(tuple(component / output_dt_s for component in integral))

        source_bounds.append((source_a_s, source_b_s))
        source_a_s = source_b_s

    return commands_by_side, source_bounds


def _append_adaptive_violation(
    violations: List[AdaptiveLimitViolation],
    ratio: float,
    *,
    allowed_ratio: float,
    source_start_s: float,
    source_end_s: float,
    tick: int,
    side: str,
    kind: str,
    value: float,
    limit: float,
) -> None:
    if ratio <= allowed_ratio:
        return
    violations.append(
        AdaptiveLimitViolation(
            ratio=ratio,
            source_start_s=source_start_s,
            source_end_s=source_end_s,
            tick=tick,
            side=side,
            kind=kind,
            value=value,
            limit=limit,
        )
    )


def _adaptive_output_limit_violations(
    commands_by_side: Dict[str, List[Tuple[float, float, float, float, float, float]]],
    source_bounds: List[Tuple[float, float]],
    output_dt_s: float,
    max_linear_speed: float,
    max_angular_speed: float,
    max_linear_accel: float,
    max_angular_accel: float,
    tolerance: float,
    include_stop: bool = True,
) -> List[AdaptiveLimitViolation]:
    allowed_ratio = 1.0 + max(0.0, tolerance)
    violations: List[AdaptiveLimitViolation] = []
    zero = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    for side, commands in commands_by_side.items():
        for tick, values in enumerate(commands):
            source_start_s, source_end_s = source_bounds[tick]
            if max_linear_speed > 0.0:
                value = _values_norm(values, 0)
                _append_adaptive_violation(
                    violations,
                    value / max_linear_speed,
                    allowed_ratio=allowed_ratio,
                    source_start_s=source_start_s,
                    source_end_s=source_end_s,
                    tick=tick,
                    side=side,
                    kind="linear_speed",
                    value=value,
                    limit=max_linear_speed,
                )
            if max_angular_speed > 0.0:
                value = _values_norm(values, 3)
                _append_adaptive_violation(
                    violations,
                    value / max_angular_speed,
                    allowed_ratio=allowed_ratio,
                    source_start_s=source_start_s,
                    source_end_s=source_end_s,
                    tick=tick,
                    side=side,
                    kind="angular_speed",
                    value=value,
                    limit=max_angular_speed,
                )

        if not commands:
            continue

        prev_values = zero
        stop_extra_tick = 1 if include_stop else 0
        for tick in range(len(commands) + stop_extra_tick):
            curr_values = commands[tick] if tick < len(commands) else zero
            if tick == 0:
                source_start_s, source_end_s = source_bounds[0]
            elif tick >= len(commands):
                source_start_s, source_end_s = source_bounds[-1]
            else:
                source_start_s = min(source_bounds[tick - 1][0], source_bounds[tick][0])
                source_end_s = max(source_bounds[tick - 1][1], source_bounds[tick][1])

            if max_linear_accel > 0.0:
                delta = tuple(curr_values[i] - prev_values[i] for i in range(6))
                value = _values_norm(delta, 0) / output_dt_s
                _append_adaptive_violation(
                    violations,
                    value / max_linear_accel,
                    allowed_ratio=allowed_ratio,
                    source_start_s=source_start_s,
                    source_end_s=source_end_s,
                    tick=tick,
                    side=side,
                    kind="linear_accel",
                    value=value,
                    limit=max_linear_accel,
                )
            if max_angular_accel > 0.0:
                delta = tuple(curr_values[i] - prev_values[i] for i in range(6))
                value = _values_norm(delta, 3) / output_dt_s
                _append_adaptive_violation(
                    violations,
                    value / max_angular_accel,
                    allowed_ratio=allowed_ratio,
                    source_start_s=source_start_s,
                    source_end_s=source_end_s,
                    tick=tick,
                    side=side,
                    kind="angular_accel",
                    value=value,
                    limit=max_angular_accel,
                )
            prev_values = curr_values

    violations.sort(key=lambda item: item.ratio, reverse=True)
    return violations


def _adaptive_output_limit_violation(
    commands_by_side: Dict[str, List[Tuple[float, float, float, float, float, float]]],
    source_bounds: List[Tuple[float, float]],
    output_dt_s: float,
    max_linear_speed: float,
    max_angular_speed: float,
    max_linear_accel: float,
    max_angular_accel: float,
    tolerance: float,
    include_stop: bool = True,
) -> Optional[AdaptiveLimitViolation]:
    violations = _adaptive_output_limit_violations(
        commands_by_side,
        source_bounds,
        output_dt_s,
        max_linear_speed,
        max_angular_speed,
        max_linear_accel,
        max_angular_accel,
        tolerance,
        include_stop=include_stop,
    )
    return violations[0] if violations else None


def _cap_adaptive_scales_in_window(
    intervals: List[SourceInterval],
    interval_end_times: List[float],
    scales: List[float],
    source_start_s: float,
    source_end_s: float,
    speed_scale_cap: float,
    min_required_speed_scale: float,
    eps: float,
) -> bool:
    if speed_scale_cap <= eps:
        speed_scale_cap = eps
    if min_required_speed_scale > 0.0 and speed_scale_cap < min_required_speed_scale - eps:
        raise ReplayAbort(
            "Adaptive output limits require speed_scale %.6g below adaptive_min_speed_scale %.6g"
            % (speed_scale_cap, min_required_speed_scale)
        )

    center_s = 0.5 * (source_start_s + source_end_s)
    half_width_s = max(eps, 0.5 * (source_end_s - source_start_s))
    min_scale = max(eps, min_required_speed_scale if min_required_speed_scale > 0.0 else eps)
    changed = False

    idx = bisect.bisect_right(interval_end_times, source_start_s + eps)
    while idx < len(intervals):
        interval = intervals[idx]
        if interval.start_s >= source_end_s - eps:
            break

        interval_mid_s = 0.5 * (interval.start_s + interval.end_s)
        normalized_distance = min(1.0, abs(interval_mid_s - center_s) / half_width_s)
        strength = 0.25 + 0.75 * (1.0 - normalized_distance)
        local_cap = max(min_scale, speed_scale_cap / strength)
        if local_cap < scales[idx] * (1.0 - 1e-9):
            scales[idx] = local_cap
            changed = True
        idx += 1

    return changed


def _adaptive_source_accel_limited_scales(
    intervals: List[SourceInterval],
    interval_end_times: List[float],
    scales: List[float],
    output_dt_s: float,
    max_linear_accel: float,
    max_angular_accel: float,
    min_required_speed_scale: float,
    local_smoothing_window_s: float,
    eps: float,
) -> None:
    if max_linear_accel <= 0.0 and max_angular_accel <= 0.0:
        return

    window_pad_s = max(output_dt_s, local_smoothing_window_s)
    side_names = list(intervals[0].values_by_side.keys())

    for idx in range(1, len(intervals)):
        prev_interval = intervals[idx - 1]
        curr_interval = intervals[idx]
        source_dt_s = max(output_dt_s, curr_interval.start_s - prev_interval.start_s)
        cap = math.inf
        for side in side_names:
            prev_values = prev_interval.values_by_side[side]
            curr_values = curr_interval.values_by_side[side]
            delta = tuple(curr_values[i] - prev_values[i] for i in range(6))
            if max_linear_accel > 0.0:
                source_accel = _values_norm(delta, 0) / source_dt_s
                if source_accel > eps:
                    cap = min(cap, math.sqrt(max_linear_accel / source_accel))
            if max_angular_accel > 0.0:
                source_accel = _values_norm(delta, 3) / source_dt_s
                if source_accel > eps:
                    cap = min(cap, math.sqrt(max_angular_accel / source_accel))

        if cap < math.inf:
            transition_s = curr_interval.start_s
            _cap_adaptive_scales_in_window(
                intervals,
                interval_end_times,
                scales,
                max(0.0, transition_s - window_pad_s),
                min(intervals[-1].end_s, transition_s + window_pad_s),
                cap,
                min_required_speed_scale,
                eps,
            )


def _limit_adaptive_output_commands(
    commands_by_side: Dict[str, List[Tuple[float, float, float, float, float, float]]],
    output_dt_s: float,
    max_linear_speed: float,
    max_angular_speed: float,
    max_linear_accel: float,
    max_angular_accel: float,
    linear_jerk_step_limit: float,
    angular_jerk_step_limit: float,
) -> Dict[str, List[Tuple[float, float, float, float, float, float]]]:
    if not commands_by_side:
        return commands_by_side

    # The controller limits acceleration per component. The adaptive limits are
    # checked as vector norms, so divide by sqrt(3) to keep the norm below limit.
    component_norm_margin = math.sqrt(3.0)
    linear_accel_step = (
        max_linear_accel * output_dt_s / component_norm_margin
        if max_linear_accel > 0.0
        else 0.0
    )
    angular_accel_step = (
        max_angular_accel * output_dt_s / component_norm_margin
        if max_angular_accel > 0.0
        else 0.0
    )
    sample_commands = {
        side: [
            TwistSample(rel_t=tick * output_dt_s, values=values)
            for tick, values in enumerate(commands)
        ]
        for side, commands in commands_by_side.items()
    }
    filtered = _simulate_ur_twist_limiter_samples(
        sample_commands,
        max_linear_speed,
        max_angular_speed,
        linear_accel_step,
        angular_accel_step,
        linear_jerk_step_limit,
        angular_jerk_step_limit,
    )
    return {side: [sample.values for sample in samples] for side, samples in filtered.items()}


def _resample_adaptive_time_scaled_twists(
    samples_by_side: Dict[str, List[TwistSample]],
    source_duration_s: float,
    max_speed_scale: float,
    output_rate_hz: float,
    max_linear_speed: float,
    max_angular_speed: float,
    max_linear_accel: float,
    max_angular_accel: float,
    min_required_speed_scale: float,
    accel_tolerance: float,
    local_smoothing_window_s: float,
    output_linear_jerk_step: float,
    output_angular_jerk_step: float,
) -> Tuple[
    float,
    float,
    float,
    float,
    Dict[str, List[Tuple[float, float, float, float, float, float]]],
]:
    if source_duration_s <= 0.0:
        raise ReplayAbort("Invalid source duration for replay")
    if max_speed_scale <= 0.0:
        raise ReplayAbort("speed_scale must be > 0")
    if output_rate_hz <= 0.0:
        raise ReplayAbort("replay_rate_hz must be > 0")
    if min_required_speed_scale < 0.0:
        raise ReplayAbort("adaptive_min_speed_scale must be >= 0")
    if local_smoothing_window_s < 0.0:
        raise ReplayAbort("adaptive_local_smoothing_window_s must be >= 0")
    if output_linear_jerk_step < 0.0:
        raise ReplayAbort("adaptive_filter_linear_jerk_step must be >= 0")
    if output_angular_jerk_step < 0.0:
        raise ReplayAbort("adaptive_filter_angular_jerk_step must be >= 0")

    eps = 1e-12
    output_dt_s = 1.0 / output_rate_hz
    intervals = _build_source_intervals(samples_by_side, source_duration_s, output_dt_s, eps)
    interval_end_times = [interval.end_s for interval in intervals]
    scales = _adaptive_velocity_limited_scales(
        intervals,
        max_speed_scale,
        max_linear_speed,
        max_angular_speed,
        min_required_speed_scale,
        eps,
    )
    _adaptive_source_accel_limited_scales(
        intervals,
        interval_end_times,
        scales,
        output_dt_s,
        max_linear_accel,
        max_angular_accel,
        min_required_speed_scale,
        local_smoothing_window_s,
        eps,
    )

    side_names = list(samples_by_side.keys())
    commands_by_side, source_bounds = _render_adaptive_time_scaled_twists(
        intervals,
        scales,
        output_dt_s,
        source_duration_s,
        side_names,
        eps,
    )

    commands_by_side = _limit_adaptive_output_commands(
        commands_by_side,
        output_dt_s,
        max_linear_speed,
        max_angular_speed,
        max_linear_accel,
        max_angular_accel,
        output_linear_jerk_step,
        output_angular_jerk_step,
    )
    violation = _adaptive_output_limit_violation(
        commands_by_side,
        source_bounds,
        output_dt_s,
        max_linear_speed,
        max_angular_speed,
        max_linear_accel,
        max_angular_accel,
        accel_tolerance,
        include_stop=False,
    )
    if violation is not None:
        raise ReplayAbort(
            "Adaptive output limiter left a limit violation near source %.6fs "
            "(%s %.6g > %.6g, ratio %.3f)"
            % (
                violation.source_start_s,
                violation.kind,
                violation.value,
                violation.limit,
                violation.ratio,
            )
        )

    profile_duration_s = sum(
        interval.duration_s / max(scale, eps)
        for interval, scale in zip(intervals, scales)
    )
    if profile_duration_s <= 0.0:
        raise ReplayAbort("Adaptive replay profile has invalid duration")

    overall_effective_scale = source_duration_s / profile_duration_s
    return output_dt_s, min(scales), overall_effective_scale, max(scales), commands_by_side


@dataclass
class ControllerManagerServices:
    list_controllers: str
    load_controller: str
    switch_controller: str


class ControllerManager:
    def __init__(
        self,
        controller_manager_ns: str,
        *,
        wait_services_timeout_s: float,
        auto_load: bool,
        strictness: int,
        start_asap: bool,
        switch_timeout_s: float,
        verify_timeout_s: float,
        verify_period_s: float,
    ):
        cm = _resolve_ns(controller_manager_ns)
        self.services = ControllerManagerServices(
            list_controllers=f"{cm}/list_controllers",
            load_controller=f"{cm}/load_controller",
            switch_controller=f"{cm}/switch_controller",
        )
        self.wait_services_timeout_s = wait_services_timeout_s
        self.auto_load = auto_load
        self.strictness = strictness
        self.start_asap = start_asap
        self.switch_timeout_s = switch_timeout_s
        self.verify_timeout_s = verify_timeout_s
        self.verify_period_s = verify_period_s
        self.list_proxy = None
        self.load_proxy = None
        self.switch_proxy = None

    def connect(self) -> None:
        self._wait_service(self.services.list_controllers)
        self._wait_service(self.services.switch_controller)
        if self.auto_load:
            self._wait_service(self.services.load_controller)
        self.list_proxy = rospy.ServiceProxy(self.services.list_controllers, ListControllers)
        self.switch_proxy = rospy.ServiceProxy(self.services.switch_controller, SwitchController)
        self.load_proxy = (
            rospy.ServiceProxy(self.services.load_controller, LoadController)
            if self.auto_load
            else None
        )

    def _wait_service(self, name: str) -> None:
        resolved = rospy.resolve_name(name)
        timeout = float(self.wait_services_timeout_s)
        rospy.loginfo("Waiting for service '%s' (timeout=%.3fs)", resolved, timeout)
        if timeout <= 0.0:
            rospy.wait_for_service(name)
        else:
            rospy.wait_for_service(name, timeout=timeout)

    def list_states(self) -> Optional[Dict[str, str]]:
        try:
            resp = self.list_proxy(ListControllersRequest())
        except rospy.ServiceException as exc:
            rospy.logwarn("ListControllers failed on %s: %s", self.services.list_controllers, exc)
            return None
        return {controller.name: controller.state for controller in resp.controller}

    def ensure_loaded(self, controllers: List[str]) -> None:
        if not self.auto_load or self.load_proxy is None:
            return
        states = self.list_states()
        for name in controllers:
            if states is not None and name in states:
                continue
            try:
                resp = self.load_proxy(LoadControllerRequest(name=name))
                if not resp.ok:
                    rospy.logwarn("LoadController('%s') returned ok=false", name)
            except rospy.ServiceException as exc:
                rospy.logwarn("LoadController('%s') failed: %s", name, exc)

    def ensure_states(self, must_run: List[str], must_stop: List[str]) -> bool:
        states = self.list_states()
        if states is None:
            return self._switch(list(must_run), list(must_stop))

        start = [name for name in must_run if states.get(name) != "running"]
        stop = [name for name in must_stop if states.get(name) == "running"]
        if start or stop:
            ok = self._switch(start, stop)
            if not ok and int(self.strictness) == 2:
                rospy.logwarn("STRICT controller switch failed; retrying BEST_EFFORT.")
                old = self.strictness
                self.strictness = 1
                ok = self._switch(start, stop)
                self.strictness = old
            if not ok:
                return False

        if self.verify_timeout_s <= 0.0:
            return True

        start_time = rospy.Time.now()
        last_states = None
        while not rospy.is_shutdown():
            if (rospy.Time.now() - start_time).to_sec() >= self.verify_timeout_s:
                break
            last_states = self.list_states()
            if last_states is None:
                rospy.sleep(self.verify_period_s)
                continue
            running_ok = all(last_states.get(name) == "running" for name in must_run)
            stopped_ok = all(last_states.get(name) != "running" for name in must_stop)
            if running_ok and stopped_ok:
                return True
            rospy.sleep(self.verify_period_s)

        rospy.logerr(
            "Controller verification failed. must_run=%s must_stop=%s last_states=%s",
            must_run,
            must_stop,
            last_states,
        )
        return False

    def _switch(self, start: List[str], stop: List[str]) -> bool:
        req = SwitchControllerRequest()
        req.start_controllers = list(start)
        req.stop_controllers = list(stop)
        req.strictness = int(self.strictness)
        req.start_asap = bool(self.start_asap)
        req.timeout = float(self.switch_timeout_s)
        try:
            resp = self.switch_proxy(req)
            return bool(resp.ok)
        except rospy.ServiceException as exc:
            rospy.logerr("SwitchController start=%s stop=%s failed: %s", start, stop, exc)
            return False


class ReplayAbort(RuntimeError):
    pass


class DualSlaveTwistReplay:
    def __init__(self):
        self.status_pub = rospy.Publisher(
            _get_str("~status_topic", "/teleop/dual_slave_replay/status"),
            String,
            queue_size=1,
            latch=True,
        )
        self.manifest_path = os.path.abspath(_get_str("~manifest", ""))
        if not self.manifest_path:
            raise ReplayAbort("Missing required param '~manifest'")
        with open(self.manifest_path, "r") as f:
            self.manifest = yaml.safe_load(f)
        if not isinstance(self.manifest, dict):
            raise ReplayAbort(f"Invalid manifest: {self.manifest_path}")

        bag_default = str(self.manifest.get("bag", ""))
        bag_param = _get_str("~bag", "").strip()
        self.bag_path = os.path.abspath(bag_param or bag_default)
        if not os.path.isfile(self.bag_path):
            raise ReplayAbort(f"Bag not found: {self.bag_path}")

        self.robot_ns = _get_str("~robot_ns", "mur620d")
        self.dry_run = _get_bool("~dry_run", False)
        self.home_only = _get_bool("~home_only", False)
        self.execute_home = _get_bool("~execute_home", True)
        self.start_delay_s = _get_float("~start_delay_s", 2.0)
        self.speed_scale = _get_float("~speed_scale", 1.0)
        self.time_scale = _get_float("~time_scale", 1.0)
        self.replay_rate_hz = _get_float("~replay_rate_hz", 500.0)
        self.replay_speed_mode = _get_str("~replay_speed_mode", "fixed").strip().lower()
        self.adaptive_max_linear_speed_mps = _get_float("~adaptive_max_linear_speed_mps", 0.0)
        self.adaptive_max_angular_speed_radps = _get_float("~adaptive_max_angular_speed_radps", 0.0)
        self.adaptive_max_linear_accel_mps2 = _get_float("~adaptive_max_linear_accel_mps2", 0.0)
        self.adaptive_max_angular_accel_radps2 = _get_float("~adaptive_max_angular_accel_radps2", 0.0)
        self.adaptive_min_speed_scale = _get_float("~adaptive_min_speed_scale", 0.0)
        self.adaptive_accel_tolerance = _get_float("~adaptive_accel_tolerance", 1e-3)
        self.adaptive_local_smoothing_window_s = _get_float("~adaptive_local_smoothing_window_s", 0.05)
        self.adaptive_source_filter = _get_str("~adaptive_source_filter", "ur_twist_limiter").strip().lower()
        self.adaptive_filter_linear_speed_mps = _get_float("~adaptive_filter_linear_speed_mps", 0.6)
        self.adaptive_filter_angular_speed_radps = _get_float("~adaptive_filter_angular_speed_radps", 1.0)
        self.adaptive_filter_linear_accel_step = _get_float("~adaptive_filter_linear_accel_step", 0.004)
        self.adaptive_filter_angular_accel_step = _get_float("~adaptive_filter_angular_accel_step", 0.007)
        self.adaptive_filter_linear_jerk_step = _get_float("~adaptive_filter_linear_jerk_step", 0.0008)
        self.adaptive_filter_angular_jerk_step = _get_float("~adaptive_filter_angular_jerk_step", 0.0017)
        self.publish_zero_rate_hz = _get_float("~publish_zero_rate_hz", 20.0)
        self.zero_before_s = _get_float("~zero_before_s", 0.5)
        self.zero_after_s = _get_float("~zero_after_s", 0.5)
        if self.speed_scale <= 0.0:
            raise ReplayAbort("speed_scale must be > 0")
        if self.replay_rate_hz <= 0.0:
            raise ReplayAbort("replay_rate_hz must be > 0")
        if self.replay_speed_mode not in ("fixed", "adaptive"):
            raise ReplayAbort("replay_speed_mode must be 'fixed' or 'adaptive'")
        if self.adaptive_source_filter not in ("none", "ur_twist_limiter"):
            raise ReplayAbort("adaptive_source_filter must be 'none' or 'ur_twist_limiter'")
        for param_name, param_value in [
            ("adaptive_max_linear_speed_mps", self.adaptive_max_linear_speed_mps),
            ("adaptive_max_angular_speed_radps", self.adaptive_max_angular_speed_radps),
            ("adaptive_max_linear_accel_mps2", self.adaptive_max_linear_accel_mps2),
            ("adaptive_max_angular_accel_radps2", self.adaptive_max_angular_accel_radps2),
            ("adaptive_min_speed_scale", self.adaptive_min_speed_scale),
            ("adaptive_accel_tolerance", self.adaptive_accel_tolerance),
            ("adaptive_local_smoothing_window_s", self.adaptive_local_smoothing_window_s),
            ("adaptive_filter_linear_speed_mps", self.adaptive_filter_linear_speed_mps),
            ("adaptive_filter_angular_speed_radps", self.adaptive_filter_angular_speed_radps),
            ("adaptive_filter_linear_accel_step", self.adaptive_filter_linear_accel_step),
            ("adaptive_filter_angular_accel_step", self.adaptive_filter_angular_accel_step),
            ("adaptive_filter_linear_jerk_step", self.adaptive_filter_linear_jerk_step),
            ("adaptive_filter_angular_jerk_step", self.adaptive_filter_angular_jerk_step),
        ]:
            if param_value < 0.0:
                raise ReplayAbort(f"{param_name} must be >= 0")
        if abs(self.time_scale - 1.0) > 1e-9:
            raise ReplayAbort(
                "time_scale is deprecated for trajectory replay. Use speed_scale as the "
                "area-preserving trajectory speed factor: 2.0 halves duration, 0.5 doubles it."
            )
        self.planning_time = _get_float("~planning_time", 5.0)
        self.num_planning_attempts = _get_int("~num_planning_attempts", 5)
        self.max_velocity_scaling_factor = _get_float("~max_velocity_scaling_factor", 0.1)
        self.max_acceleration_scaling_factor = _get_float("~max_acceleration_scaling_factor", 0.1)
        self.goal_joint_tolerance = _get_float("~goal_joint_tolerance", 0.001)
        self.tcp_position_tolerance_m = _get_float("~tcp_position_tolerance_m", 0.03)
        self.tcp_orientation_tolerance_rad = _get_float("~tcp_orientation_tolerance_rad", 0.15)
        self.tcp_verify_timeout_s = _get_float("~tcp_verify_timeout_s", 5.0)
        self.force_abort_threshold_n = _get_float("~force_abort_threshold_n", 150.0)
        self.torque_abort_threshold_nm = _get_float("~torque_abort_threshold_nm", 40.0)
        self.wrench_timeout_s = _get_float("~wrench_timeout_s", 0.5)
        self.zero_ft_delay_s = _get_float("~ur10e_zero_loadcell_delay_s", 0.5)
        self.zero_ft_wait_timeout_s = _get_float("~ur10e_zero_loadcell_wait_timeout_s", 10.0)
        self.moveit_ns = _get_str("~moveit_ns", f"/{self.robot_ns}")
        self.robot_description_param = _get_str(
            "~robot_description_param", f"/{self.robot_ns}/robot_description"
        )

        self.twist_controller = _get_str("~twist_controller", "twist_controller")
        self.arm_controller = _get_str("~arm_controller", "arm_controller")
        self.force_torque_controller = _get_str(
            "~force_torque_controller", "force_torque_sensor_controller"
        )
        self.controller_auto_load = _get_bool("~auto_load_controllers", True)
        self.controller_strictness = _get_int("~strictness", 2)
        self.controller_start_asap = _get_bool("~start_asap", False)
        self.controller_switch_timeout_s = _get_float("~switch_timeout_s", 0.0)
        self.controller_wait_services_timeout_s = _get_float("~wait_services_timeout_s", 15.0)
        self.controller_verify_timeout_s = _get_float("~controller_state_verify_timeout_s", 5.0)
        self.controller_verify_period_s = _get_float("~controller_state_verify_period_s", 0.1)

        self.window = self.manifest.get("window", {})
        self.replay_start_s = self._window_time("start_offset_s", "auto_start_time", "replay_start_time")
        self.replay_end_s = self._window_time("end_offset_s", "auto_end_time", "replay_end_time")
        if self.replay_end_s <= self.replay_start_s:
            raise ReplayAbort("Invalid replay window: end is not after start")

        self.arms = self._load_arms()
        self.pubs = {
            side: rospy.Publisher(arm["publish_command_topic"], Twist, queue_size=1)
            for side, arm in self.arms.items()
        }
        self.last_wrench: Dict[str, Tuple[rospy.Time, float, float]] = {}
        self.wrench_subs = [
            rospy.Subscriber(arm["wrench_topic"], WrenchStamped, self._wrench_cb, callback_args=side)
            for side, arm in self.arms.items()
        ]
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def _publish_status(self, text: str) -> None:
        rospy.loginfo("%s", text)
        self.status_pub.publish(String(data=text))

    def _window_time(self, offset_param: str, auto_key: str, replay_key: str) -> float:
        override_offset = _get_optional_float("~" + offset_param)
        if override_offset is None and replay_key in self.window:
            return float(self.window[replay_key])
        offset = override_offset
        if offset is None:
            offset = float(self.window.get(offset_param, 0.0))
        auto_time = self.window.get(auto_key)
        if auto_time is not None:
            return float(auto_time) + offset
        return float(self.window[replay_key])

    def _load_arms(self) -> Dict[str, Dict]:
        manifest_arms = self.manifest.get("arms", {})
        out = {}
        for side, default_arm_ns in [("left", "UR10_l"), ("right", "UR10_r")]:
            arm = dict(manifest_arms.get(side, {}))
            if not arm:
                raise ReplayAbort(f"Manifest missing arm entry: {side}")
            arm_ns = str(arm.get("arm_ns", default_arm_ns))
            move_group_param = f"~move_group_name_{'l' if side == 'left' else 'r'}"
            arm["move_group_name"] = _get_str(move_group_param, arm.get("move_group_name", ""))
            if not arm["move_group_name"]:
                raise ReplayAbort(f"Missing MoveIt group for {side}")
            arm["controller_manager_ns"] = _get_str(
                f"~{side}_controller_manager_ns",
                f"/{self.robot_ns}/{arm_ns}/controller_manager",
            )
            arm["publish_command_topic"] = _get_str(
                f"~{side}_command_topic",
                arm.get("command_topic", f"/{self.robot_ns}/{arm_ns}/twist_controller/command_collision_free"),
            )
            arm["wrench_topic"] = _get_str(
                f"~{side}_wrench_topic",
                f"/{self.robot_ns}/{arm_ns}/wrench",
            )
            arm["zero_ftsensor_enabled"] = _get_bool(
                f"~do_ur10e_zero_loadcell_slave_{side}",
                True,
            )
            arm["zero_ftsensor_service"] = _get_str(
                f"~{side}_zero_ftsensor_service",
                f"/{self.robot_ns}/{arm_ns}/ur_hardware_interface/zero_ftsensor",
            )
            out[side] = arm
        return out

    def _wrench_cb(self, msg: WrenchStamped, side: str) -> None:
        self.last_wrench[side] = (
            rospy.Time.now(),
            _norm3(msg.wrench.force),
            _norm3(msg.wrench.torque),
        )

    def _check_safety(self) -> None:
        now = rospy.Time.now()
        for side, sample in self.last_wrench.items():
            stamp, force_norm, torque_norm = sample
            if self.wrench_timeout_s > 0.0 and (now - stamp).to_sec() > self.wrench_timeout_s:
                continue
            if self.force_abort_threshold_n > 0.0 and force_norm >= self.force_abort_threshold_n:
                raise ReplayAbort(
                    f"{side} force abort: {force_norm:.3f} N >= {self.force_abort_threshold_n:.3f} N"
                )
            if self.torque_abort_threshold_nm > 0.0 and torque_norm >= self.torque_abort_threshold_nm:
                raise ReplayAbort(
                    f"{side} torque abort: {torque_norm:.3f} Nm >= {self.torque_abort_threshold_nm:.3f} Nm"
                )

    def _publish_zero_for(self, duration_s: float) -> None:
        if self.dry_run:
            return
        rate_hz = max(1.0, self.publish_zero_rate_hz)
        rate = rospy.Rate(rate_hz)
        end_time = rospy.Time.now() + rospy.Duration(max(0.0, duration_s))
        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            for pub in self.pubs.values():
                pub.publish(_zero_twist())
            rate.sleep()
        for pub in self.pubs.values():
            pub.publish(_zero_twist())

    def _controller_manager(self, arm: Dict) -> ControllerManager:
        cm = ControllerManager(
            arm["controller_manager_ns"],
            wait_services_timeout_s=self.controller_wait_services_timeout_s,
            auto_load=self.controller_auto_load,
            strictness=self.controller_strictness,
            start_asap=self.controller_start_asap,
            switch_timeout_s=self.controller_switch_timeout_s,
            verify_timeout_s=self.controller_verify_timeout_s,
            verify_period_s=self.controller_verify_period_s,
        )
        cm.connect()
        cm.ensure_loaded([self.arm_controller, self.twist_controller, self.force_torque_controller])
        return cm

    def _ensure_home_controllers(self, controllers: Dict[str, ControllerManager]) -> None:
        for side, cm in controllers.items():
            self._publish_status(f"{side}: enabling MoveIt controller state")
            ok = cm.ensure_states(
                must_run=[self.arm_controller, self.force_torque_controller],
                must_stop=[self.twist_controller],
            )
            if not ok:
                raise ReplayAbort(f"{side}: cannot enable MoveIt controller state")

    def _ensure_twist_controllers(self, controllers: Dict[str, ControllerManager]) -> None:
        for side, cm in controllers.items():
            self._publish_status(f"{side}: enabling twist controller state")
            ok = cm.ensure_states(
                must_run=[self.twist_controller, self.force_torque_controller],
                must_stop=[self.arm_controller],
            )
            if not ok:
                raise ReplayAbort(f"{side}: cannot enable twist controller state")

    def _joint_target_for_group(self, group, arm: Dict) -> Dict[str, float]:
        joint_state = arm.get("joint_state", {})
        names = [str(name) for name in joint_state.get("name", [])]
        positions = [float(value) for value in joint_state.get("position", [])]
        if len(names) != len(positions):
            raise ReplayAbort("Manifest joint_state name/position length mismatch")
        exact = {name: positions[i] for i, name in enumerate(names)}
        by_base = {name.split("/")[-1]: positions[i] for i, name in enumerate(names)}
        target = {}
        missing = []
        for joint in group.get_active_joints():
            if joint in exact:
                target[joint] = exact[joint]
                continue
            base = joint.split("/")[-1]
            if base in by_base:
                target[joint] = by_base[base]
                continue
            missing.append(joint)
        if missing:
            raise ReplayAbort(f"Missing joint values for MoveIt joints: {missing}")
        return target

    def _moveit_to_initial_joint_state(self) -> None:
        if self.dry_run:
            return
        try:
            import moveit_commander
        except Exception as exc:
            raise ReplayAbort(f"Cannot import moveit_commander: {exc}")

        moveit_commander.roscpp_initialize(sys.argv)
        for side, arm in self.arms.items():
            self._publish_status(f"{side}: planning to recorded initial joint state")
            group = moveit_commander.MoveGroupCommander(
                arm["move_group_name"],
                robot_description=str(self.robot_description_param),
                ns=str(_resolve_ns(self.moveit_ns)),
            )
            group.set_planning_time(float(self.planning_time))
            group.set_num_planning_attempts(int(self.num_planning_attempts))
            group.set_max_velocity_scaling_factor(float(self.max_velocity_scaling_factor))
            group.set_max_acceleration_scaling_factor(float(self.max_acceleration_scaling_factor))
            group.set_goal_joint_tolerance(float(self.goal_joint_tolerance))
            group.set_start_state_to_current_state()
            target = self._joint_target_for_group(group, arm)
            group.set_joint_value_target(target)
            try:
                plan_res = group.plan()
            except Exception as exc:
                raise ReplayAbort(f"{side}: planning failed: {exc}")
            success = True
            plan = plan_res
            if isinstance(plan_res, tuple):
                success = bool(plan_res[0])
                plan = plan_res[1] if len(plan_res) >= 2 else None
            if not success or plan is None:
                raise ReplayAbort(f"{side}: MoveIt did not find a plan")
            points = getattr(getattr(plan, "joint_trajectory", None), "points", [])
            if not points:
                raise ReplayAbort(f"{side}: MoveIt plan has no trajectory points")
            if self.execute_home:
                self._publish_status(f"{side}: executing initial joint state")
                ok = bool(group.execute(plan, wait=True))
                group.stop()
                group.clear_pose_targets()
                if not ok:
                    raise ReplayAbort(f"{side}: MoveIt execution failed")
            else:
                self._publish_status(f"{side}: plan completed, execute_home=false")

    def _verify_tcp_pose(self) -> None:
        if self.dry_run:
            return
        for side, arm in self.arms.items():
            expected = arm.get("tcp_pose_base", {})
            base_frame = str(arm.get("base_frame", expected.get("frame_id", "")))
            tcp_frame = str(arm.get("tcp_frame", expected.get("child_frame_id", "")))
            if not base_frame or not tcp_frame:
                raise ReplayAbort(f"{side}: missing base/tcp frame in manifest")
            expected_pos = [float(x) for x in expected.get("position", [])]
            expected_q = [float(x) for x in expected.get("orientation_xyzw", [])]
            if len(expected_pos) != 3 or len(expected_q) != 4:
                raise ReplayAbort(f"{side}: missing tcp_pose_base in manifest")

            deadline = rospy.Time.now() + rospy.Duration(self.tcp_verify_timeout_s)
            transform = None
            while not rospy.is_shutdown() and rospy.Time.now() < deadline:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        base_frame,
                        tcp_frame,
                        rospy.Time(0),
                        rospy.Duration(0.2),
                    )
                    break
                except Exception:
                    rospy.sleep(0.05)
            if transform is None:
                raise ReplayAbort(f"{side}: TF lookup failed for {base_frame} -> {tcp_frame}")

            actual_pos = [
                float(transform.transform.translation.x),
                float(transform.transform.translation.y),
                float(transform.transform.translation.z),
            ]
            actual_q = [
                float(transform.transform.rotation.x),
                float(transform.transform.rotation.y),
                float(transform.transform.rotation.z),
                float(transform.transform.rotation.w),
            ]
            pos_err = math.sqrt(sum((a - b) * (a - b) for a, b in zip(actual_pos, expected_pos)))
            ori_err = _quat_angle_xyzw(actual_q, expected_q)
            rospy.loginfo("%s: TCP verification pos_err=%.4f m ori_err=%.4f rad", side, pos_err, ori_err)
            if pos_err > self.tcp_position_tolerance_m:
                raise ReplayAbort(
                    f"{side}: TCP position error {pos_err:.4f} m > {self.tcp_position_tolerance_m:.4f} m"
                )
            if ori_err > self.tcp_orientation_tolerance_rad:
                raise ReplayAbort(
                    f"{side}: TCP orientation error {ori_err:.4f} rad > {self.tcp_orientation_tolerance_rad:.4f} rad"
                )

    def _zero_ft_sensors(self) -> None:
        if self.dry_run:
            return

        for side, arm in self.arms.items():
            if not arm.get("zero_ftsensor_enabled", True):
                rospy.loginfo("%s: FT sensor zero disabled", side)
                continue

            service_name = str(arm["zero_ftsensor_service"])
            resolved_service = rospy.resolve_name(service_name)
            self._publish_status(f"{side}: zeroing FT sensor {resolved_service}")

            try:
                if self.zero_ft_wait_timeout_s <= 0.0:
                    rospy.wait_for_service(service_name)
                else:
                    rospy.wait_for_service(service_name, timeout=self.zero_ft_wait_timeout_s)
            except rospy.ROSException as exc:
                raise ReplayAbort(f"{side}: FT zero service not available ({resolved_service}): {exc}")

            if self.zero_ft_delay_s > 0.0:
                rospy.sleep(self.zero_ft_delay_s)

            try:
                response = rospy.ServiceProxy(service_name, Trigger)()
            except rospy.ServiceException as exc:
                raise ReplayAbort(f"{side}: FT zero service call failed ({resolved_service}): {exc}")

            if not response.success:
                raise ReplayAbort(
                    f"{side}: FT zero request returned success=false ({resolved_service}): {response.message}"
                )
            rospy.loginfo("%s: FT sensor zeroed successfully: %s", side, response.message)

    def _load_samples(self) -> Dict[str, List[TwistSample]]:
        source_topics = {
            self.arms["left"]["command_topic"]: "left",
            self.arms["right"]["command_topic"]: "right",
        }
        samples: Dict[str, List[TwistSample]] = {side: [] for side in self.arms}
        with rosbag.Bag(self.bag_path, "r") as bag:
            for topic, msg, stamp in bag.read_messages(
                topics=list(source_topics.keys()),
                start_time=rospy.Time.from_sec(self.replay_start_s),
                end_time=rospy.Time.from_sec(self.replay_end_s),
            ):
                side = source_topics[topic]
                rel_t = stamp.to_sec() - self.replay_start_s
                samples[side].append(TwistSample(rel_t=rel_t, values=_twist_values(msg)))

        missing = [side for side, side_samples in samples.items() if not side_samples]
        if missing:
            raise ReplayAbort(f"No twist messages found in replay window for arm(s): {missing}")
        for side_samples in samples.values():
            side_samples.sort(key=lambda item: item.rel_t)
        return samples

    def _build_scaled_replay(self, samples: Dict[str, List[TwistSample]]) -> ScaledReplay:
        source_duration_s = self.replay_end_s - self.replay_start_s
        output_dt_s = 1.0 / self.replay_rate_hz
        commands_by_side: Dict[str, List[Tuple[float, float, float, float, float, float]]] = {}

        if self.replay_speed_mode == "adaptive":
            adaptive_samples = samples
            if self.adaptive_source_filter == "ur_twist_limiter":
                adaptive_samples = _simulate_ur_twist_limiter_samples(
                    samples,
                    self.adaptive_filter_linear_speed_mps,
                    self.adaptive_filter_angular_speed_radps,
                    self.adaptive_filter_linear_accel_step,
                    self.adaptive_filter_angular_accel_step,
                    self.adaptive_filter_linear_jerk_step,
                    self.adaptive_filter_angular_jerk_step,
                )
            (
                output_dt_s,
                effective_min,
                effective_mean,
                effective_max,
                commands_by_side,
            ) = _resample_adaptive_time_scaled_twists(
                adaptive_samples,
                source_duration_s,
                self.speed_scale,
                self.replay_rate_hz,
                self.adaptive_max_linear_speed_mps,
                self.adaptive_max_angular_speed_radps,
                self.adaptive_max_linear_accel_mps2,
                self.adaptive_max_angular_accel_radps2,
                self.adaptive_min_speed_scale,
                self.adaptive_accel_tolerance,
                self.adaptive_local_smoothing_window_s,
                self.adaptive_filter_linear_jerk_step,
                self.adaptive_filter_angular_jerk_step,
            )
            counts = {side: len(commands) for side, commands in commands_by_side.items()}
            if len(set(counts.values())) != 1:
                raise ReplayAbort(f"Internal adaptive replay resampling mismatch: {counts}")
            tick_count = next(iter(counts.values()))
            return ScaledReplay(
                output_dt_s=output_dt_s,
                duration_s=tick_count * output_dt_s,
                commands_by_side=commands_by_side,
                speed_mode="adaptive",
                effective_speed_scale_min=effective_min,
                effective_speed_scale_mean=effective_mean,
                effective_speed_scale_max=effective_max,
            )

        for side, side_samples in samples.items():
            side_dt_s, commands = _resample_time_scaled_twists(
                side_samples,
                source_duration_s,
                self.speed_scale,
                self.replay_rate_hz,
            )
            output_dt_s = side_dt_s
            commands_by_side[side] = commands

        counts = {side: len(commands) for side, commands in commands_by_side.items()}
        if len(set(counts.values())) != 1:
            raise ReplayAbort(f"Internal replay resampling mismatch: {counts}")

        tick_count = next(iter(counts.values()))
        return ScaledReplay(
            output_dt_s=output_dt_s,
            duration_s=tick_count * output_dt_s,
            commands_by_side=commands_by_side,
            speed_mode="fixed",
            effective_speed_scale_min=self.speed_scale,
            effective_speed_scale_mean=self.speed_scale,
            effective_speed_scale_max=self.speed_scale,
        )

    def _run_replay(self, replay: ScaledReplay) -> None:
        self._publish_status("publishing zero before replay")
        self._publish_zero_for(self.zero_before_s)

        if self.start_delay_s > 0.0:
            self._publish_status(f"waiting start delay {self.start_delay_s:.3f}s")
            end_delay = rospy.Time.now() + rospy.Duration(self.start_delay_s)
            rate = rospy.Rate(100.0)
            while not rospy.is_shutdown() and rospy.Time.now() < end_delay:
                self._check_safety()
                rate.sleep()

        self._publish_status("replaying")
        wall_start = rospy.Time.now()
        for tick in range(replay.tick_count):
            self._check_safety()
            target_time = wall_start + rospy.Duration(tick * replay.output_dt_s)
            while not rospy.is_shutdown() and rospy.Time.now() < target_time:
                self._check_safety()
                rospy.sleep(min(0.002, max(0.0, (target_time - rospy.Time.now()).to_sec())))
            if rospy.is_shutdown():
                raise ReplayAbort("ROS shutdown")
            for side, commands in replay.commands_by_side.items():
                self.pubs[side].publish(_twist_from_values(commands[tick]))

        replay_end = wall_start + rospy.Duration(replay.duration_s)
        while not rospy.is_shutdown() and rospy.Time.now() < replay_end:
            self._check_safety()
            rospy.sleep(min(0.002, max(0.0, (replay_end - rospy.Time.now()).to_sec())))

        self._publish_status("publishing zero after replay")
        self._publish_zero_for(self.zero_after_s)
        self._publish_status("complete")

    def run(self) -> None:
        samples = self._load_samples()
        replay = self._build_scaled_replay(samples)
        replay_mode_summary = (
            "mode adaptive, source_filter %s, speed_scale cap %.3f, "
            "effective speed_scale min/mean/max %.3f/%.3f/%.3f"
            % (
                self.adaptive_source_filter,
                self.speed_scale,
                replay.effective_speed_scale_min,
                replay.effective_speed_scale_mean,
                replay.effective_speed_scale_max,
            )
            if replay.speed_mode == "adaptive"
            else "mode fixed, speed_scale %.3f" % self.speed_scale
        )
        self._publish_status(
            "loaded %s twist samples, window %.6f -> %.6f, source duration %.3fs, "
            "%s, replay duration %.3fs, rate %.1f Hz, ticks %d"
            % (
                {side: len(side_samples) for side, side_samples in samples.items()},
                self.replay_start_s,
                self.replay_end_s,
                self.replay_end_s - self.replay_start_s,
                replay_mode_summary,
                replay.duration_s,
                self.replay_rate_hz,
                replay.tick_count,
            )
        )
        if self.dry_run:
            self._publish_status("dry_run_complete")
            return

        controllers = {
            side: self._controller_manager(arm)
            for side, arm in self.arms.items()
        }
        try:
            self._publish_zero_for(self.zero_before_s)
            self._ensure_home_controllers(controllers)
            self._moveit_to_initial_joint_state()
            self._ensure_twist_controllers(controllers)
            self._verify_tcp_pose()
            self._zero_ft_sensors()
            if self.home_only:
                self._publish_status("home_only_complete")
                self._publish_zero_for(self.zero_after_s)
                return
            self._run_replay(replay)
        except Exception:
            self._publish_zero_for(self.zero_after_s)
            raise


def main() -> int:
    rospy.init_node("replay_dual_slave_twist_from_bag", anonymous=False)
    try:
        replay = DualSlaveTwistReplay()
        replay.run()
        return 0
    except ReplayAbort as exc:
        try:
            replay._publish_status(f"aborted: {exc}")
        except Exception:
            pass
        rospy.logerr("%s", exc)
        return 2
    except Exception as exc:
        try:
            replay._publish_status(f"failed: {exc}")
        except Exception:
            pass
        rospy.logerr("%s", exc)
        return 1


if __name__ == "__main__":
    sys.exit(main())
