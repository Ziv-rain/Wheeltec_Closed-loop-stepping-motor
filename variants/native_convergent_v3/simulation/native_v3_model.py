#!/usr/bin/env python3
"""Deterministic robustness model for the native-parameter V3 controller.

This is not a hardware-identification claim.  It deliberately sweeps a wide
unknown plant-gain range while preserving project facts: 20 Hz vision, the
existing camera EMA, positive angle -> negative ball motion, +3/-5 degree
static-friction asymmetry and the V3 controller values.
"""

from collections import deque
from dataclasses import dataclass
from typing import Optional


DT = 0.005
FRAME_DT = 0.050
MODEL_KP = 0.80
MODEL_KD = 0.50
MODEL_KI = 0.02
MODEL_PREDICTION_S = 0.10
MODEL_OUTPUT_LIMIT = 6.0


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


@dataclass
class Result:
    scenario: str
    gain: float
    delay_ms: int
    final_cm: float
    tail_span_cm: float
    maximum_abs_cm: float
    settled_s: Optional[float]


class NativeV3Model:
    def __init__(self, setpoint):
        self.setpoint = setpoint
        self.kp = MODEL_KP
        self.kd = MODEL_KD
        self.ki = MODEL_KI
        self.integral = 0.0
        self.velocity = 0.0
        self.history = deque(maxlen=3)
        self.stationary_s = 0.0
        self.settled_s = 0.0
        self.breakaway_s = 0.0
        self.breakaway_start_cm = 0.0
        self.mode = "hold"
        self.desired = 0.0
        self.command = 0.0
        self.good_frames = 0

    def observe(self, now_s, position_cm):
        self.good_frames += 1
        self.history.append((now_s, position_cm))
        if len(self.history) >= 2:
            elapsed = self.history[-1][0] - self.history[0][0]
            raw_velocity = (
                (self.history[-1][1] - self.history[0][1]) / elapsed
                if elapsed >= 0.020
                else 0.0
            )
            self.velocity = 0.50 * self.velocity + 0.50 * raw_velocity
        else:
            self.velocity = 0.0

        if self.good_frames < 2:
            self.desired = 0.0
            self.mode = "hold"
            return

        error = position_cm - self.setpoint
        predicted_error = error + clamp(self.velocity * MODEL_PREDICTION_S, -1.0, 1.0)
        approaching = error * self.velocity < 0.0
        distance = abs(error)
        if not approaching or distance <= 1.0:
            brake_scale = 1.0
        elif distance >= 3.0:
            brake_scale = 0.35
        else:
            brake_scale = 0.35 + 0.65 * (3.0 - distance) / 2.0

        p_error = 0.0 if distance <= 0.20 and abs(self.velocity) <= 0.60 else predicted_error
        p_term = self.kp * p_error
        d_term = self.kd * self.velocity * brake_scale

        if distance <= 1.0 and abs(self.velocity) <= 1.0 and self.mode not in (
            "breakaway",
            "settled",
        ):
            candidate = clamp(self.integral + error * FRAME_DT, -3.0, 3.0)
            old_raw = p_term + d_term + self.ki * self.integral
            new_raw = p_term + d_term + self.ki * candidate
            if not ((old_raw >= MODEL_OUTPUT_LIMIT and new_raw > old_raw) or (old_raw <= -MODEL_OUTPUT_LIMIT and new_raw < old_raw)):
                self.integral = candidate

        output = p_term + d_term + self.ki * self.integral

        if distance <= 0.30 and abs(self.velocity) <= 0.60:
            self.settled_s += FRAME_DT
        else:
            self.settled_s = 0.0

        if self.mode == "settled":
            if distance <= 0.60 and abs(self.velocity) <= 1.20:
                output = 0.0
                self.integral = 0.0
            else:
                self.mode = "rolling"

        if self.settled_s >= 0.50:
            self.mode = "settled"
            self.integral = 0.0
            output = 0.0
        elif self.mode != "settled":
            if distance >= 0.35 and abs(self.velocity) <= 0.30:
                self.stationary_s += FRAME_DT
            else:
                self.stationary_s = 0.0

            moving_toward = approaching and abs(self.velocity) >= 0.25
            moved_toward = (
                (position_cm - self.breakaway_start_cm) * error < 0.0
                and abs(position_cm - self.breakaway_start_cm) >= 0.10
            )
            if self.mode == "breakaway":
                self.breakaway_s += FRAME_DT
                if moving_toward or moved_toward or self.breakaway_s >= 0.20 or distance < 0.35:
                    self.mode = "rolling"
                    self.stationary_s = 0.0
                    self.breakaway_s = 0.0
            elif self.stationary_s >= 0.30:
                self.mode = "breakaway"
                self.breakaway_s = 0.0
                self.breakaway_start_cm = position_cm
            else:
                self.mode = "rolling"

            if self.mode == "breakaway":
                if error > 0.0:
                    output = max(output, 3.20)
                elif error < 0.0:
                    output = min(output, -5.20)

        self.desired = clamp(output, -MODEL_OUTPUT_LIMIT, MODEL_OUTPUT_LIMIT)

    def tick(self):
        maximum_delta = 80.0 * DT
        self.command = clamp(
            self.desired,
            self.command - maximum_delta,
            self.command + maximum_delta,
        )
        return self.command


def simulate(initial_cm, setpoint_cm, gain, delay_ms, duration_s=20.0):
    controller = NativeV3Model(setpoint_cm)
    position = initial_cm
    velocity = 0.0
    motor_angle = 0.0
    camera_ema = initial_cm
    state_history = deque([(0.0, initial_cm)])
    trace = []
    next_frame = 0.0
    settled_at = None
    steps = int(duration_s / DT)

    for step in range(steps):
        now_s = step * DT
        state_history.append((now_s, position))
        while state_history and now_s - state_history[0][0] > 0.50:
            state_history.popleft()

        if now_s + 1e-9 >= next_frame:
            capture_s = max(0.0, now_s - delay_ms / 1000.0)
            delayed_position = state_history[0][1]
            for timestamp, saved_position in state_history:
                if timestamp <= capture_s:
                    delayed_position = saved_position
                else:
                    break
            camera_ema = 0.35 * delayed_position + 0.65 * camera_ema
            quantized = round(camera_ema, 2)
            controller.observe(now_s, quantized)
            next_frame += FRAME_DT

        target_angle = controller.tick()
        motor_angle += clamp(target_angle - motor_angle, -120.0 * DT, 120.0 * DT)

        positive_static = 3.0
        negative_static = 5.0
        if abs(velocity) < 0.03 and -negative_static < motor_angle < positive_static:
            acceleration = 0.0
            velocity = 0.0
        else:
            if motor_angle >= 0.0:
                effective_angle = max(0.0, motor_angle - 1.20)
            else:
                effective_angle = min(0.0, motor_angle + 2.00)
            acceleration = -gain * effective_angle - 0.55 * velocity
            velocity += acceleration * DT
            position += velocity * DT

        position = clamp(position, -9.5, 9.5)
        if abs(position) >= 9.5 and position * velocity > 0.0:
            velocity = 0.0

        trace.append(position)
        if (
            settled_at is None
            and now_s >= 1.0
            and abs(position - setpoint_cm) <= 0.50
            and abs(velocity) <= 1.0
        ):
            settled_at = now_s

    tail = trace[-int(3.0 / DT) :]
    scenario = f"{initial_cm:+.0f}->{setpoint_cm:+.0f}"
    return Result(
        scenario=scenario,
        gain=gain,
        delay_ms=delay_ms,
        final_cm=position,
        tail_span_cm=max(tail) - min(tail),
        maximum_abs_cm=max(abs(value) for value in trace),
        settled_s=settled_at,
    )


def run_sweep():
    results = []
    scenarios = ((8.0, 0.0), (-8.0, 0.0), (5.0, -5.0))
    for initial_cm, setpoint_cm in scenarios:
        for gain in (1.2, 2.4, 3.0):
            for delay_ms in (50, 100, 150):
                results.append(simulate(initial_cm, setpoint_cm, gain, delay_ms))
    return results


def main():
    results = run_sweep()
    print("scenario gain delay final_cm tail_span max_abs settled_s")
    for result in results:
        settled = "-" if result.settled_s is None else f"{result.settled_s:.2f}"
        print(
            f"{result.scenario:>7} {result.gain:>4.1f} {result.delay_ms:>5d} "
            f"{result.final_cm:>+8.3f} {result.tail_span_cm:>9.3f} "
            f"{result.maximum_abs_cm:>7.3f} {settled:>9}"
        )

    failures = [
        result
        for result in results
        if abs(result.final_cm - float(result.scenario.split("->")[1])) > 0.65
        or result.tail_span_cm > 1.20
        or result.maximum_abs_cm > 9.50
    ]
    if failures:
        raise SystemExit(f"robustness failures: {len(failures)}")


if __name__ == "__main__":
    main()
