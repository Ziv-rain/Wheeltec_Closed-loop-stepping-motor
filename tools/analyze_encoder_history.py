#!/usr/bin/env python3
"""Convert terminal RAM-log dumps into samples and a 50/100 ms profile."""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


WHEEL_METERS_PER_COUNT = 0.0002618
SAMPLE_PERIOD_MS = 50


@dataclass
class CarSample:
    export: int
    run_id: int
    task: int
    index: int
    tick_ms: int
    delta_left: int
    delta_right: int
    pwm_left: int
    pwm_right: int
    base_speed: int
    line_mask: int


@dataclass
class BallSample:
    export: int
    session_id: int
    index: int
    tick_ms: int
    ball_cm: float
    ball_velocity_cm_s: float
    accel_mps2: float
    pid_out_deg: float
    ff_angle_deg: float
    target_angle_deg: float
    actual_angle_deg: float
    flags: int
    task: int


def _numbers(line: str) -> list[int]:
    return [int(part.strip(), 0) for part in line.strip().split(",")[1:]]


def parse_log(lines: Iterable[str]) -> tuple[list[CarSample], list[BallSample]]:
    cars: list[CarSample] = []
    balls: list[BallSample] = []
    car_export = 0
    ball_export = 0
    car_task = 0

    for raw in lines:
        line = raw.strip()
        try:
            if line.startswith("CARLOG_BEGIN,"):
                values = _numbers(line)
                if len(values) >= 4:
                    car_export += 1
                    car_task = values[2]
            elif line.startswith("CARLOG,"):
                values = _numbers(line)
                if len(values) == 9 and car_export != 0:
                    cars.append(
                        CarSample(
                            car_export,
                            values[0],
                            car_task,
                            values[1],
                            values[2],
                            values[3],
                            values[4],
                            values[5],
                            values[6],
                            values[7],
                            values[8],
                        )
                    )
            elif line.startswith("BALLLOG_BEGIN,"):
                values = _numbers(line)
                if len(values) >= 4:
                    ball_export += 1
            elif line.startswith("BALLLOG,"):
                values = _numbers(line)
                if len(values) == 12 and ball_export != 0:
                    balls.append(
                        BallSample(
                            ball_export,
                            values[0],
                            values[1],
                            values[2],
                            values[3] / 100.0,
                            values[4] / 100.0,
                            values[5] / 1000.0,
                            values[6] / 100.0,
                            values[7] / 100.0,
                            values[8] / 100.0,
                            values[9] / 100.0,
                            values[10],
                            values[11],
                        )
                    )
        except ValueError:
            # Terminal status/help lines and truncated lines are intentionally ignored.
            continue
    return cars, balls


def _median_window(values: list[float], index: int, radius: int = 2) -> float:
    start = max(0, index - radius)
    end = min(len(values), index + radius + 1)
    return statistics.median(values[start:end])


def build_rows(cars: list[CarSample], balls: list[BallSample]) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    car_groups: dict[int, list[CarSample]] = {}
    ball_groups: dict[int, list[BallSample]] = {}
    for sample in cars:
        car_groups.setdefault(sample.export, []).append(sample)
    for sample in balls:
        ball_groups.setdefault(sample.export, []).append(sample)

    for export, group in sorted(car_groups.items()):
        group.sort(key=lambda item: item.index)
        raw_speeds: list[float] = []
        previous_tick: int | None = None
        for sample in group:
            dt_s = (
                SAMPLE_PERIOD_MS / 1000.0
                if previous_tick is None
                else max(0.001, (sample.tick_ms - previous_tick) / 1000.0)
            )
            distance = (sample.delta_left + sample.delta_right) * 0.5 * WHEEL_METERS_PER_COUNT
            raw_speeds.append(distance / dt_s)
            previous_tick = sample.tick_ms
        speeds = [_median_window(raw_speeds, i) for i in range(len(raw_speeds))]
        accelerations: list[float] = [0.0] * len(group)
        for i in range(1, len(group)):
            dt_s = max(0.001, (group[i].tick_ms - group[i - 1].tick_ms) / 1000.0)
            accelerations[i] = (speeds[i] - speeds[i - 1]) / dt_s

        matching_balls = sorted(ball_groups.get(export, []), key=lambda item: item.tick_ms)
        by_tick = {sample.tick_ms: sample for sample in matching_balls}
        first_tick = group[0].tick_ms
        cumulative_left = 0
        cumulative_right = 0
        for i, sample in enumerate(group):
            cumulative_left += sample.delta_left
            cumulative_right += sample.delta_right
            ball = by_tick.get(sample.tick_ms)
            future_50 = accelerations[i + 1] if i + 1 < len(group) else math.nan
            future_100 = accelerations[i + 2] if i + 2 < len(group) else math.nan
            rows.append(
                {
                    "export": export,
                    "run_id": sample.run_id,
                    "task": sample.task,
                    "sample_index": sample.index,
                    "tick_ms": sample.tick_ms,
                    "elapsed_ms": sample.tick_ms - first_tick,
                    "delta_left": sample.delta_left,
                    "delta_right": sample.delta_right,
                    "distance_left_m": cumulative_left * WHEEL_METERS_PER_COUNT,
                    "distance_right_m": cumulative_right * WHEEL_METERS_PER_COUNT,
                    "speed_raw_mps": raw_speeds[i],
                    "speed_median_mps": speeds[i],
                    "accel_mps2": accelerations[i],
                    "future_accel_50ms_mps2": future_50,
                    "future_accel_100ms_mps2": future_100,
                    "pwm_left_pct": sample.pwm_left,
                    "pwm_right_pct": sample.pwm_right,
                    "base_speed_pct": sample.base_speed,
                    "line_mask": sample.line_mask,
                    "ball_cm": "" if ball is None else ball.ball_cm,
                    "ball_velocity_cm_s": "" if ball is None else ball.ball_velocity_cm_s,
                    "controller_accel_mps2": "" if ball is None else ball.accel_mps2,
                    "pid_out_deg": "" if ball is None else ball.pid_out_deg,
                    "ff_angle_deg": "" if ball is None else ball.ff_angle_deg,
                    "target_angle_deg": "" if ball is None else ball.target_angle_deg,
                    "actual_angle_deg": "" if ball is None else ball.actual_angle_deg,
                    "sample_flags": "" if ball is None else ball.flags,
                }
            )
    return rows


def build_profile(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    buckets: dict[tuple[int, int], list[dict[str, object]]] = {}
    for row in rows:
        bucket_ms = int(row["elapsed_ms"]) // SAMPLE_PERIOD_MS * SAMPLE_PERIOD_MS
        buckets.setdefault((int(row["task"]), bucket_ms), []).append(row)

    profile: list[dict[str, object]] = []
    for (task, bucket_ms), values in sorted(buckets.items()):
        def med(key: str) -> float:
            nums = [float(value[key]) for value in values if not math.isnan(float(value[key]))]
            return statistics.median(nums) if nums else math.nan

        profile.append(
            {
                "task": task,
                "elapsed_ms": bucket_ms,
                "run_count": len({int(value["export"]) for value in values}),
                "sample_count": len(values),
                "speed_median_mps": med("speed_median_mps"),
                "accel_median_mps2": med("accel_mps2"),
                "future_accel_50ms_median_mps2": med("future_accel_50ms_mps2"),
                "future_accel_100ms_median_mps2": med("future_accel_100ms_mps2"),
            }
        )
    return profile


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        raise ValueError(f"no rows available for {path.name}")
    with path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "logs", type=Path, nargs="+", help="one or more terminal texts captured after pressing H"
    )
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--prefix", default=None, help="output filename prefix")
    args = parser.parse_args()

    cars: list[CarSample] = []
    balls: list[BallSample] = []
    export_offset = 0
    for log_path in args.logs:
        new_cars, new_balls = parse_log(
            log_path.read_text(encoding="utf-8", errors="ignore").splitlines()
        )
        for sample in new_cars:
            sample.export += export_offset
        for sample in new_balls:
            sample.export += export_offset
        cars.extend(new_cars)
        balls.extend(new_balls)
        export_offset += max(
            [sample.export - export_offset for sample in new_cars] + [0]
        )
    if not cars:
        parser.error("no complete CARLOG records found")
    output_dir = args.output_dir or args.logs[0].parent
    output_dir.mkdir(parents=True, exist_ok=True)
    rows = build_rows(cars, balls)
    profile = build_profile(rows)
    prefix = args.prefix or (args.logs[0].stem if len(args.logs) == 1 else "combined-history")
    samples_path = output_dir / f"{prefix}-samples.csv"
    profile_path = output_dir / f"{prefix}-profile.csv"
    write_csv(samples_path, rows)
    write_csv(profile_path, profile)
    print(f"car samples: {len(cars)}, ball samples: {len(balls)}, exports: {len(set(s.export for s in cars))}")
    print(samples_path)
    print(profile_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
