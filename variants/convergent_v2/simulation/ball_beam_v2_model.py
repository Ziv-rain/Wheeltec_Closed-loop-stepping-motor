"""Conservative delayed-vision model used to bench convergent_v2 settings.

This is not a substitute for hardware. It sweeps plant gain and camera delay
so a parameter set is rejected if it only works for one guessed mechanism.
The sign follows the measured rig: positive beam command moves the ball in
the negative coordinate direction.
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import math
import random


DT = 0.005


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass
class Plant:
    gain: float
    position: float = 8.0
    velocity: float = 0.0
    angle: float = 0.0
    damping: float = 0.20
    angle_tau: float = 0.060

    def step(self, angle_target: float) -> None:
        self.angle += (angle_target - self.angle) * DT / self.angle_tau
        acceleration = -self.gain * self.angle - self.damping * self.velocity
        self.velocity += acceleration * DT
        self.position += self.velocity * DT


@dataclass
class V2ReferenceController:
    plant_gain_estimate: float = 2.4
    position_to_speed: float = 0.35
    maximum_speed: float = 2.5
    maximum_reference_accel: float = 2.0
    velocity_gain: float = 0.55
    feedforward_limit: float = 0.60
    normal_angle_limit: float = 1.5
    rate_limit_deg_s: float = 20.0
    velocity_reference: float = 0.0
    command: float = 0.0

    def update(self, position: float, velocity: float) -> float:
        desired_velocity = clamp(
            -self.position_to_speed * position,
            -self.maximum_speed,
            self.maximum_speed,
        )
        previous_reference = self.velocity_reference
        maximum_change = self.maximum_reference_accel * 0.050
        self.velocity_reference = clamp(
            desired_velocity,
            previous_reference - maximum_change,
            previous_reference + maximum_change,
        )
        reference_acceleration = (
            self.velocity_reference - previous_reference
        ) / 0.050
        feedforward = clamp(
            -reference_acceleration / self.plant_gain_estimate,
            -self.feedforward_limit,
            self.feedforward_limit,
        )
        feedback = self.velocity_gain * (velocity - self.velocity_reference)
        desired_angle = clamp(
            feedforward + feedback,
            -self.normal_angle_limit,
            self.normal_angle_limit,
        )
        maximum_angle_change = self.rate_limit_deg_s * 0.050
        self.command = clamp(
            desired_angle,
            self.command - maximum_angle_change,
            self.command + maximum_angle_change,
        )
        return self.command


def simulate(gain: float, delay_ms: int, seconds: float = 18.0,
             seed: int = 7) -> dict[str, float]:
    random.seed(seed)
    plant = Plant(gain=gain)
    controller = V2ReferenceController()
    delay_steps = max(1, round(delay_ms / (DT * 1000)))
    history = deque([plant.position] * (delay_steps + 1),
                    maxlen=delay_steps + 1)
    previous_measurement = None
    filtered_velocity = 0.0
    target_angle = 0.0
    positions: list[float] = []

    for step in range(round(seconds / DT)):
        history.append(plant.position)
        if step % round(0.050 / DT) == 0:
            measurement = history[0] + random.gauss(0.0, 0.05)
            if previous_measurement is not None:
                raw_velocity = (measurement - previous_measurement) / 0.050
                filtered_velocity = 0.35 * filtered_velocity + 0.65 * raw_velocity
            previous_measurement = measurement

            # Fixed-delay state replay approximation: propagate the delayed
            # measurement to now using the measured velocity and current model.
            horizon = delay_ms / 1000.0
            acceleration = -controller.plant_gain_estimate * plant.angle
            predicted_position = (
                measurement + filtered_velocity * horizon +
                0.5 * acceleration * horizon * horizon
            )
            predicted_velocity = filtered_velocity + acceleration * horizon
            target_angle = controller.update(
                predicted_position, predicted_velocity)

        plant.step(target_angle)
        positions.append(plant.position)

    tail = positions[round(12.0 / DT):]
    return {
        "gain": gain,
        "delay_ms": float(delay_ms),
        "final_cm": positions[-1],
        "tail_amplitude_cm": (max(tail) - min(tail)) / 2,
        "maximum_abs_cm": max(abs(value) for value in positions),
    }


def run_sweep() -> int:
    failed = False
    print("gain  delay  final_cm  tail_amp  max_abs")
    for gain in (1.2, 2.4, 6.0):
        for delay_ms in (50, 100, 150):
            result = simulate(gain, delay_ms)
            print(
                f"{gain:4.1f}  {delay_ms:4d}  "
                f"{result['final_cm']:+8.3f}  "
                f"{result['tail_amplitude_cm']:8.3f}  "
                f"{result['maximum_abs_cm']:7.3f}"
            )
            failed |= result["maximum_abs_cm"] >= 9.0
            failed |= abs(result["final_cm"]) >= 1.0
            failed |= result["tail_amplitude_cm"] >= 1.0
    return 1 if failed else 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sweep", action="store_true",
                        help="run the required delay/gain robustness sweep")
    parser.add_argument("--gain", type=float, default=2.4)
    parser.add_argument("--delay-ms", type=int, default=100)
    args = parser.parse_args()
    if args.sweep:
        return run_sweep()
    result = simulate(args.gain, args.delay_ms)
    for key, value in result.items():
        print(f"{key}: {value}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
