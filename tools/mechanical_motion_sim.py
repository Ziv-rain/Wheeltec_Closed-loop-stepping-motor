#!/usr/bin/env python3
"""Tunable steel-ball motion simulation and open-loop profile fitter.

The coordinate origin is the centre of A'-D'.  Positive position points to D'.
The default task is 0 cm -> +5 cm -> -5 cm.
"""

from __future__ import annotations

import argparse
import csv
import math
import random
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple


@dataclass(frozen=True)
class Model:
    gravity: float = 9.80665
    rolling_friction: float = 0.012
    viscous_damping: float = 0.55
    angle_ratio: float = 1.0
    motor_rate_deg_s: float = 100.0
    motor_time_constant_s: float = 0.06
    min_angle_deg: float = -30.0
    max_angle_deg: float = 45.0
    dt_s: float = 0.005


@dataclass(frozen=True)
class Step:
    name: str
    angle_deg: float
    duration_s: float
    waypoint: int = 0


@dataclass
class State:
    time_s: float = 0.0
    position_m: float = 0.0
    velocity_mps: float = 0.0
    motor_angle_deg: float = 0.0


@dataclass(frozen=True)
class Sample:
    time_s: float
    position_cm: float
    velocity_cm_s: float
    command_angle_deg: float
    motor_angle_deg: float
    acceleration_mps2: float
    step: str
    waypoint: int


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def acceleration(model: Model, state: State) -> float:
    """Solid sphere rolling without slip, with Coulomb and viscous losses."""
    theta = math.radians(state.motor_angle_deg * model.angle_ratio)
    drive = (5.0 / 7.0) * model.gravity * math.sin(theta)
    friction_limit = model.rolling_friction * model.gravity * math.cos(theta)

    if abs(state.velocity_mps) < 1e-5:
        if abs(drive) <= friction_limit:
            return 0.0
        friction_sign = math.copysign(1.0, drive)
    else:
        friction_sign = math.copysign(1.0, state.velocity_mps)
    return drive - friction_sign * friction_limit - model.viscous_damping * state.velocity_mps


def advance(model: Model, state: State, command_deg: float) -> float:
    command_deg = clamp(command_deg, model.min_angle_deg, model.max_angle_deg)
    desired_rate = (command_deg - state.motor_angle_deg) / max(
        model.motor_time_constant_s, model.dt_s
    )
    rate = clamp(desired_rate, -model.motor_rate_deg_s, model.motor_rate_deg_s)
    state.motor_angle_deg = clamp(
        state.motor_angle_deg + rate * model.dt_s,
        model.min_angle_deg,
        model.max_angle_deg,
    )

    accel = acceleration(model, state)
    old_velocity = state.velocity_mps
    state.velocity_mps += accel * model.dt_s
    if old_velocity * state.velocity_mps < 0.0 and abs(accel) <= (
        model.rolling_friction * model.gravity * 1.05
    ):
        state.velocity_mps = 0.0
    state.position_m += state.velocity_mps * model.dt_s
    state.time_s += model.dt_s
    return accel


def simulate_steps(
    model: Model,
    steps: Sequence[Step],
    initial: State | None = None,
    record: bool = False,
) -> Tuple[State, List[Sample]]:
    state = replace(initial) if initial is not None else State()
    samples: List[Sample] = []
    for step in steps:
        ticks = max(1, round(step.duration_s / model.dt_s))
        command = clamp(step.angle_deg, model.min_angle_deg, model.max_angle_deg)
        for _ in range(ticks):
            accel = advance(model, state, command)
            if record:
                samples.append(
                    Sample(
                        state.time_s,
                        state.position_m * 100.0,
                        state.velocity_mps * 100.0,
                        command,
                        state.motor_angle_deg,
                        accel,
                        step.name,
                        step.waypoint,
                    )
                )
    return state, samples


def candidate_steps(
    direction: int,
    drive_angle: float,
    drive_time: float,
    brake_angle: float,
    brake_time: float,
    settle_time: float,
    waypoint: int,
) -> List[Step]:
    label = f"leg{waypoint}"
    return [
        Step(f"{label}_drive", direction * drive_angle, drive_time),
        Step(f"{label}_brake", -direction * brake_angle, brake_time),
        Step(f"{label}_settle", 0.0, settle_time, waypoint),
    ]


def score_leg(
    model: Model,
    initial: State,
    steps: Sequence[Step],
    target_m: float,
    low_m: float,
    high_m: float,
) -> Tuple[float, State]:
    end, samples = simulate_steps(model, steps, initial, record=True)
    positions = [sample.position_cm / 100.0 for sample in samples]
    boundary_error = max(
        max((low_m - min(positions)), 0.0),
        max((max(positions) - high_m), 0.0),
    )
    position_error = end.position_m - target_m
    return (
        (position_error / 0.0015) ** 2
        + (end.velocity_mps / 0.012) ** 2
        + (boundary_error / 0.002) ** 2
        + 0.015 * sum(step.duration_s for step in steps),
        end,
    )


def fit_leg(
    model: Model,
    initial: State,
    target_m: float,
    waypoint: int,
    rng: random.Random,
    trials: int,
) -> Tuple[List[Step], State, float]:
    direction = 1 if target_m > initial.position_m else -1
    low_m, high_m = sorted((initial.position_m, target_m))
    best: Tuple[float, List[Step], State] | None = None

    def evaluate(params: Tuple[float, float, float, float, float]) -> None:
        nonlocal best
        drive_angle, drive_time, brake_angle, brake_time, settle_time = params
        steps = candidate_steps(
            direction,
            drive_angle,
            drive_time,
            brake_angle,
            brake_time,
            settle_time,
            waypoint,
        )
        value, end = score_leg(model, initial, steps, target_m, low_m, high_m)
        if best is None or value < best[0]:
            best = (value, steps, end)

    for _ in range(max(200, trials)):
        evaluate(
            (
                rng.uniform(1.5, 10.0),
                rng.uniform(0.12, 0.75),
                rng.uniform(1.0, 9.0),
                rng.uniform(0.06, 0.55),
                rng.uniform(0.20, 0.70),
            )
        )

    assert best is not None
    centre = [
        abs(best[1][0].angle_deg),
        best[1][0].duration_s,
        abs(best[1][1].angle_deg),
        best[1][1].duration_s,
        best[1][2].duration_s,
    ]
    spreads = [1.5, 0.08, 1.5, 0.07, 0.08]
    bounds = [(0.5, 15.0), (0.05, 1.2), (0.5, 15.0), (0.03, 1.0), (0.1, 1.0)]
    for generation in range(4):
        for _ in range(max(100, trials // 4)):
            params = tuple(
                clamp(rng.gauss(value, spreads[index]), *bounds[index])
                for index, value in enumerate(centre)
            )
            evaluate(params)  # type: ignore[arg-type]
        centre = [
            abs(best[1][0].angle_deg),
            best[1][0].duration_s,
            abs(best[1][1].angle_deg),
            best[1][1].duration_s,
            best[1][2].duration_s,
        ]
        spreads = [value * 0.5 for value in spreads]
    return best[1], best[2], best[0]


def fit_profile(
    model: Model,
    targets_cm: Sequence[float],
    seed: int = 20260731,
    trials: int = 2500,
) -> Tuple[List[Step], List[State]]:
    rng = random.Random(seed)
    state = State()
    profile: List[Step] = []
    waypoint_states: List[State] = []
    for waypoint, target_cm in enumerate(targets_cm, start=1):
        steps, state, _ = fit_leg(
            model, state, target_cm / 100.0, waypoint, rng, trials
        )
        profile.extend(steps)
        waypoint_states.append(replace(state))
    return profile, waypoint_states


def write_csv(path: Path, samples: Iterable[Sample]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "time_s",
                "position_cm",
                "velocity_cm_s",
                "command_angle_deg",
                "motor_angle_deg",
                "acceleration_mps2",
                "step",
                "waypoint",
            ]
        )
        for row in samples:
            writer.writerow(
                [
                    f"{row.time_s:.4f}",
                    f"{row.position_cm:.5f}",
                    f"{row.velocity_cm_s:.5f}",
                    f"{row.command_angle_deg:.4f}",
                    f"{row.motor_angle_deg:.4f}",
                    f"{row.acceleration_mps2:.6f}",
                    row.step,
                    row.waypoint,
                ]
            )


def write_c_header(path: Path, steps: Sequence[Step]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "#ifndef MECHANICAL_MOTION_PROFILE_H",
        "#define MECHANICAL_MOTION_PROFILE_H",
        "",
        "#include <stdint.h>",
        "",
        "/* Generated by tools/mechanical_motion_sim.py; verify on hardware at low speed. */",
        "typedef struct {",
        "    float angle_deg;",
        "    uint32_t duration_ms;",
        "    uint8_t waypoint;",
        "} MechanicalMotionStep;",
        "",
        "static const MechanicalMotionStep kMechanicalMotionProfile[] = {",
    ]
    for step in steps:
        lines.append(
            f"    {{ {step.angle_deg:.4f}f, {round(step.duration_s * 1000):d}U, "
            f"{step.waypoint}U }}, /* {step.name} */"
        )
    lines.extend(
        [
            "};",
            "",
            "#define MECHANICAL_MOTION_PROFILE_COUNT "
            "(sizeof(kMechanicalMotionProfile) / sizeof(kMechanicalMotionProfile[0]))",
            "",
            "#endif",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def _polyline(
    samples: Sequence[Sample],
    getter,
    left: float,
    top: float,
    width: float,
    height: float,
    low: float,
    high: float,
) -> str:
    end_time = samples[-1].time_s
    span = max(high - low, 1e-9)
    points = []
    for sample in samples:
        x = left + width * sample.time_s / end_time
        y = top + height * (high - getter(sample)) / span
        points.append(f"{x:.1f},{y:.1f}")
    return " ".join(points)


def write_svg(path: Path, samples: Sequence[Sample], targets_cm: Sequence[float]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    width, height = 1000, 620
    left, plot_width = 80, 880
    top1, panel_height, top2 = 60, 220, 350
    pos_values = [sample.position_cm for sample in samples] + list(targets_cm) + [0.0]
    angle_values = [
        value
        for sample in samples
        for value in (sample.command_angle_deg, sample.motor_angle_deg)
    ]
    pos_low, pos_high = min(pos_values) - 1.0, max(pos_values) + 1.0
    angle_low, angle_high = min(angle_values) - 1.0, max(angle_values) + 1.0
    pos_points = _polyline(
        samples, lambda s: s.position_cm, left, top1, plot_width, panel_height, pos_low, pos_high
    )
    command_points = _polyline(
        samples,
        lambda s: s.command_angle_deg,
        left,
        top2,
        plot_width,
        panel_height,
        angle_low,
        angle_high,
    )
    actual_points = _polyline(
        samples,
        lambda s: s.motor_angle_deg,
        left,
        top2,
        plot_width,
        panel_height,
        angle_low,
        angle_high,
    )
    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
<rect width="100%" height="100%" fill="#f8fafc"/>
<style>text{{font-family:Arial,sans-serif;fill:#172033}} .axis{{stroke:#64748b;stroke-width:1}} .grid{{stroke:#dbe3ee;stroke-width:1}}</style>
<text x="80" y="30" font-size="22" font-weight="bold">Mechanical motion: 0 cm → +5 cm → -5 cm</text>
<rect x="{left}" y="{top1}" width="{plot_width}" height="{panel_height}" fill="white" stroke="#cbd5e1"/>
<line class="grid" x1="{left}" y1="{top1 + panel_height * (pos_high)/(pos_high-pos_low):.1f}" x2="{left+plot_width}" y2="{top1 + panel_height * (pos_high)/(pos_high-pos_low):.1f}"/>
<polyline fill="none" stroke="#1463ff" stroke-width="3" points="{pos_points}"/>
<text x="15" y="{top1+15}" font-size="14">position (cm)</text>
<text x="15" y="{top1+panel_height}" font-size="12">{pos_low:.1f}</text>
<text x="20" y="{top1+12}" font-size="12">{pos_high:.1f}</text>
<rect x="{left}" y="{top2}" width="{plot_width}" height="{panel_height}" fill="white" stroke="#cbd5e1"/>
<polyline fill="none" stroke="#ef4444" stroke-width="2" points="{command_points}"/>
<polyline fill="none" stroke="#16a34a" stroke-width="2" points="{actual_points}"/>
<text x="15" y="{top2+15}" font-size="14">angle (deg)</text>
<text x="{left}" y="{height-20}" font-size="13">0 s</text>
<text x="{left+plot_width-45}" y="{height-20}" font-size="13">{samples[-1].time_s:.2f} s</text>
<line x1="680" y1="28" x2="710" y2="28" stroke="#ef4444" stroke-width="3"/><text x="715" y="33" font-size="13">command</text>
<line x1="800" y1="28" x2="830" y2="28" stroke="#16a34a" stroke-width="3"/><text x="835" y="33" font-size="13">motor actual</text>
</svg>"""
    path.write_text(svg, encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("simulation"))
    parser.add_argument("--targets-cm", type=float, nargs="+", default=[5.0, -5.0])
    parser.add_argument("--seed", type=int, default=20260731)
    parser.add_argument("--trials", type=int, default=2500)
    parser.add_argument("--friction", type=float, default=0.012)
    parser.add_argument("--damping", type=float, default=0.55)
    parser.add_argument("--angle-ratio", type=float, default=1.0)
    parser.add_argument("--motor-rate", type=float, default=100.0)
    parser.add_argument("--motor-tau", type=float, default=0.06)
    parser.add_argument("--min-angle", type=float, default=-30.0)
    parser.add_argument("--max-angle", type=float, default=45.0)
    parser.add_argument("--dt", type=float, default=0.005)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.dt <= 0.0 or args.friction < 0.0 or args.min_angle >= args.max_angle:
        raise SystemExit("invalid model parameters")
    model = Model(
        rolling_friction=args.friction,
        viscous_damping=args.damping,
        angle_ratio=args.angle_ratio,
        motor_rate_deg_s=args.motor_rate,
        motor_time_constant_s=args.motor_tau,
        min_angle_deg=args.min_angle,
        max_angle_deg=args.max_angle,
        dt_s=args.dt,
    )
    profile, waypoint_states = fit_profile(
        model, args.targets_cm, seed=args.seed, trials=args.trials
    )
    final, samples = simulate_steps(model, profile, record=True)
    write_csv(args.output_dir / "mechanical_motion_profile.csv", samples)
    write_c_header(args.output_dir / "mechanical_motion_profile.h", profile)
    write_svg(args.output_dir / "mechanical_motion_profile.svg", samples, args.targets_cm)

    print("Fitted motion profile")
    print("  coordinate: A' side negative, D' side positive")
    for index, step in enumerate(profile, start=1):
        print(
            f"  {index}: {step.name:12s} angle={step.angle_deg:+7.3f} deg "
            f"duration={step.duration_s:6.3f} s"
        )
    for index, (target, state) in enumerate(
        zip(args.targets_cm, waypoint_states), start=1
    ):
        print(
            f"  waypoint {index}: target={target:+.3f} cm, "
            f"result={state.position_m*100:+.3f} cm, "
            f"velocity={state.velocity_mps*100:+.3f} cm/s"
        )
    print(
        f"  final: t={final.time_s:.3f} s, x={final.position_m*100:+.3f} cm, "
        f"v={final.velocity_mps*100:+.3f} cm/s"
    )
    print(f"  files: {args.output_dir.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
