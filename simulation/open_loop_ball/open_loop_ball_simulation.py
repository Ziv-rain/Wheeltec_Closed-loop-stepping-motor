#!/usr/bin/env python3
"""Four-bar open-loop steel-ball simulation matching the firmware defaults."""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple


ANGLE_MIN_DEG = -30.0
ANGLE_MAX_DEG = 45.0
NEUTRAL_DEG = 0.20630064747864624
DEFAULT_ANGLES_DEG = [NEUTRAL_DEG, -24.0, 20.0, NEUTRAL_DEG, 35.0, -20.0, NEUTRAL_DEG]
DEFAULT_TIMES_MS = [200, 365, 335, 550, 405, 430, 800]
PHASE_NAMES = [
    "initial_level",
    "to_pos_accel",
    "to_pos_brake",
    "pos_settle",
    "to_neg_accel",
    "to_neg_brake",
    "neg_settle",
]


@dataclass(frozen=True)
class Geometry:
    a_x_mm: float = 0.0
    a_y_mm: float = 65.0
    c_x_mm: float = 190.0
    c_y_mm: float = 31.15
    bar_mm: float = 230.0
    crank_mm: float = 37.0
    link_mm: float = 33.85


@dataclass(frozen=True)
class Model:
    gravity: float = 9.80665
    rolling_gain: float = 0.70
    viscous_damping: float = 0.55
    coulomb_decel: float = 0.025
    static_accel_threshold: float = 0.035
    static_velocity_threshold: float = 0.0005
    motor_time_constant_s: float = 0.070
    motor_max_rate_deg_s: float = 450.0
    dt_s: float = 0.001


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def theta_to_alpha(theta_deg: float, geometry: Geometry = Geometry()) -> float:
    theta = math.radians(theta_deg)
    d_x = geometry.a_x_mm + geometry.bar_mm * math.cos(theta)
    d_y = geometry.a_y_mm + geometry.bar_mm * math.sin(theta)
    p_x = d_x - geometry.c_x_mm
    p_y = d_y - geometry.c_y_mm
    rho = math.hypot(p_x, p_y)
    if not abs(geometry.crank_mm - geometry.link_mm) <= rho <= (
        geometry.crank_mm + geometry.link_mm
    ):
        raise ValueError(f"four-bar cannot close at theta={theta_deg:.6f} deg")
    psi = math.atan2(p_y, p_x)
    cosine = (
        rho * rho + geometry.crank_mm**2 - geometry.link_mm**2
    ) / (2.0 * rho * geometry.crank_mm)
    return math.degrees(psi - math.acos(clamp(cosine, -1.0, 1.0)))


def alpha_to_theta(alpha_deg: float, geometry: Geometry = Geometry()) -> float:
    low, high = -4.804403713, 5.921090971
    alpha_low, alpha_high = theta_to_alpha(low, geometry), theta_to_alpha(high, geometry)
    if not alpha_low <= alpha_deg <= alpha_high:
        raise ValueError(
            f"motor angle {alpha_deg:.3f} outside four-bar branch "
            f"[{alpha_low:.3f}, {alpha_high:.3f}]"
        )
    for _ in range(70):
        middle = 0.5 * (low + high)
        if theta_to_alpha(middle, geometry) < alpha_deg:
            low = middle
        else:
            high = middle
    return 0.5 * (low + high)


def validate_profile(angles_deg: Sequence[float], times_ms: Sequence[int]) -> None:
    if len(angles_deg) != 7 or len(times_ms) != 7:
        raise ValueError("profile must contain exactly seven phases")
    for angle in angles_deg:
        if not ANGLE_MIN_DEG <= angle <= ANGLE_MAX_DEG:
            raise ValueError(f"unsafe motor angle: {angle}")
        alpha_to_theta(angle)
    for duration in times_ms:
        if duration < 5 or duration > 10000 or duration % 5:
            raise ValueError("each duration must be 5..10000 ms and divisible by 5")


def run_simulation(
    angles_deg: Sequence[float] = DEFAULT_ANGLES_DEG,
    times_ms: Sequence[int] = DEFAULT_TIMES_MS,
    model: Model = Model(),
) -> Tuple[List[Dict[str, float]], List[Dict[str, float]]]:
    validate_profile(angles_deg, times_ms)
    time_s = position_m = velocity_mps = 0.0
    alpha_actual = angles_deg[0]
    rows: List[Dict[str, float]] = []
    phase_results: List[Dict[str, float]] = []

    for phase_index, (name, command, duration_ms) in enumerate(
        zip(PHASE_NAMES, angles_deg, times_ms), start=1
    ):
        ticks = round(duration_ms / 1000.0 / model.dt_s)
        for _ in range(ticks):
            desired_rate = (command - alpha_actual) / model.motor_time_constant_s
            actual_rate = clamp(
                desired_rate,
                -model.motor_max_rate_deg_s,
                model.motor_max_rate_deg_s,
            )
            alpha_actual += actual_rate * model.dt_s
            theta = alpha_to_theta(alpha_actual)
            drive = -model.rolling_gain * model.gravity * math.sin(math.radians(theta))

            if abs(velocity_mps) < model.static_velocity_threshold:
                if abs(drive) <= model.static_accel_threshold:
                    acceleration = 0.0
                    velocity_mps = 0.0
                else:
                    acceleration = (
                        drive
                        - math.copysign(model.coulomb_decel, drive)
                        - model.viscous_damping * velocity_mps
                    )
            else:
                acceleration = (
                    drive
                    - math.copysign(model.coulomb_decel, velocity_mps)
                    - model.viscous_damping * velocity_mps
                )

            previous_velocity = velocity_mps
            velocity_mps += acceleration * model.dt_s
            if previous_velocity * velocity_mps < 0.0 and (
                abs(drive) <= model.static_accel_threshold
            ):
                velocity_mps = 0.0
                acceleration = 0.0
            position_m += velocity_mps * model.dt_s
            time_s += model.dt_s
            rows.append(
                {
                    "time_s": time_s,
                    "phase": float(phase_index),
                    "alpha_command_deg": command,
                    "alpha_actual_deg": alpha_actual,
                    "theta_actual_deg": theta,
                    "position_cm": position_m * 100.0,
                    "velocity_cm_s": velocity_mps * 100.0,
                    "acceleration_m_s2": acceleration,
                }
            )
        phase_results.append(dict(rows[-1]))
    return rows, phase_results


def write_csv(path: Path, rows: Iterable[Dict[str, float]]) -> None:
    rows = list(rows)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _points(
    rows: Sequence[Dict[str, float]],
    key: str,
    left: float,
    top: float,
    width: float,
    height: float,
    low: float,
    high: float,
) -> str:
    end_time = rows[-1]["time_s"]
    span = high - low
    return " ".join(
        f"{left + width * row['time_s'] / end_time:.1f},"
        f"{top + height * (high - row[key]) / span:.1f}"
        for row in rows
    )


def write_svg(path: Path, rows: Sequence[Dict[str, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    left, width, height = 70.0, 880.0, 210.0
    pos = _points(rows, "position_cm", left, 55.0, width, height, -9.0, 9.0)
    cmd = _points(rows, "alpha_command_deg", left, 340.0, width, height, -30.0, 45.0)
    actual = _points(rows, "alpha_actual_deg", left, 340.0, width, height, -30.0, 45.0)
    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="600">
<rect width="100%" height="100%" fill="#f8fafc"/>
<style>text{{font-family:Arial,sans-serif;fill:#172033}}</style>
<text x="70" y="30" font-size="21" font-weight="bold">Open-loop ball simulation</text>
<rect x="{left}" y="55" width="{width}" height="{height}" fill="white" stroke="#94a3b8"/>
<line x1="{left}" y1="148.3" x2="{left+width}" y2="148.3" stroke="#94a3b8"/>
<polyline points="{pos}" fill="none" stroke="#2563eb" stroke-width="3"/>
<text x="8" y="70">position</text><text x="15" y="88">(cm)</text>
<rect x="{left}" y="340" width="{width}" height="{height}" fill="white" stroke="#94a3b8"/>
<polyline points="{cmd}" fill="none" stroke="#ef4444" stroke-width="2"/>
<polyline points="{actual}" fill="none" stroke="#16a34a" stroke-width="2"/>
<text x="15" y="355">motor</text><text x="15" y="373">(deg)</text>
<text x="710" y="30" fill="#ef4444">red: command</text>
<text x="825" y="30" fill="#16a34a">green: actual</text>
<text x="{left}" y="580">0 s</text><text x="905" y="580">{rows[-1]['time_s']:.3f} s</text>
</svg>"""
    path.write_text(svg, encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--angles", nargs=7, type=float, default=DEFAULT_ANGLES_DEG)
    parser.add_argument("--times-ms", nargs=7, type=int, default=DEFAULT_TIMES_MS)
    parser.add_argument("--rolling-gain", type=float, default=0.70)
    parser.add_argument("--damping", type=float, default=0.55)
    parser.add_argument("--coulomb", type=float, default=0.025)
    parser.add_argument("--motor-tau", type=float, default=0.070)
    parser.add_argument("--output-dir", type=Path, default=Path("simulation/open_loop_ball/output"))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    model = Model(
        rolling_gain=args.rolling_gain,
        viscous_damping=args.damping,
        coulomb_decel=args.coulomb,
        motor_time_constant_s=args.motor_tau,
    )
    rows, phases = run_simulation(args.angles, args.times_ms, model)
    write_csv(args.output_dir / "open_loop_trace.csv", rows)
    write_svg(args.output_dir / "open_loop_trace.svg", rows)
    for name, state in zip(PHASE_NAMES, phases):
        print(
            f"{name:16s} t={state['time_s']:.3f}s "
            f"x={state['position_cm']:+.4f}cm "
            f"v={state['velocity_cm_s']:+.4f}cm/s "
            f"alpha={state['alpha_actual_deg']:+.3f}deg"
        )
    positions = [row["position_cm"] for row in rows]
    print(f"range={min(positions):+.4f}..{max(positions):+.4f} cm")
    print(f"files={args.output_dir.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
