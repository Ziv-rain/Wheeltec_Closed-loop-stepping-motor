#include "PID_speed.h"

float None_flag;
uint8_t sensor_data[GRAYSCALE_SENSOR_CHANNELS];
float sensor_weight[GRAYSCALE_SENSOR_CHANNELS] = {7, 4, 2, 0, 0, -2, -4, -7};
struct tPid EncoderLPid;
struct tPid EncoderRPid;
struct tPid GxPid;
struct tPid TurnErrorPid;
struct tPid jiaoduPid;
float Pre;
int Left, Right, count;

uint8_t g_in_pivot = 0;

static float filtered_sensor = 0.0f;
static float drive_speed_target = 0.0f;
static float drive_steer_target = 0.0f;
static float drive_speed_current = 0.0f;
static float drive_steer_current = 0.0f;
static uint8_t sensor_filter_ready = 0;
static uint8_t line_lost_count = 0;

static float slew_to(float current, float target, float step)
{
    if (target > current + step) {
        return current + step;
    }
    if (target < current - step) {
        return current - step;
    }
    return target;
}

void Motor_Smooth_Update(void)
{
    float left_duty;
    float right_duty;

    /*
     * Ramp chassis speed and steering separately.  Both wheels are then
     * derived from the same two states, so an old left/right correction cannot
     * remain behind and push the car into the next correction.
     */
    if (drive_speed_target < drive_speed_current) {
        drive_speed_current = slew_to(
            drive_speed_current, drive_speed_target, SPEED_DECEL_STEP);
    } else {
        drive_speed_current = slew_to(
            drive_speed_current, drive_speed_target, SPEED_ACCEL_STEP);
    }
    drive_steer_current = slew_to(
        drive_steer_current, drive_steer_target, STEER_SLEW_STEP);

    left_duty = drive_speed_current + drive_steer_current;
    right_duty = drive_speed_current - drive_steer_current;

    if (left_duty < 0.0f) left_duty = 0.0f;
    if (right_duty < 0.0f) right_duty = 0.0f;
    if (left_duty > (float)base_speed) left_duty = (float)base_speed;
    if (right_duty > (float)base_speed) right_duty = (float)base_speed;

    motor_left_forward();
    motor_right_forward();

    set_motor_speed(left_duty, right_duty);
}

void FollowLine_Reset(void)
{
    TurnErrorPid.err = 0;
    TurnErrorPid.err_last = 0;
    TurnErrorPid.err_sum = 0;
    TurnErrorPid.output = 0;

    filtered_sensor = 0.0f;
    drive_speed_target = 0.0f;
    drive_steer_target = 0.0f;
    drive_speed_current = 0.0f;
    drive_steer_current = 0.0f;
    sensor_filter_ready = 0;
    line_lost_count = 0;
    g_in_pivot = 0;
    Pre = 0.0f;
}

void Pid_Init(void)
{
    EncoderLPid.Kp = 40.455;
    EncoderLPid.Ki = 0;
    EncoderLPid.Kd = 0;
    EncoderRPid.Kp = 40.455;
    EncoderRPid.Ki = 0;
    EncoderRPid.Kd = 0;

    TurnErrorPid.Kp = 0;
    TurnErrorPid.Ki = 0;
    TurnErrorPid.Kd = 0;

    FollowLine_Reset();
}

void PID_caculate(struct tPid *pid, double actual_val, double target_val)
{
    pid->actual_val = actual_val;
    pid->target_val = target_val;

    pid->err = pid->target_val - pid->actual_val;

    pid->err_sum += pid->err;

    pid->output = pid->Kp * pid->err
                + pid->Ki * pid->err_sum
                + pid->Kd * (pid->err - pid->err_last);

    pid->err_last = pid->err;
}

void I_limit(struct tPid *pid, double low, double high)
{
    if (pid->err_sum < low)  pid->err_sum = low;
    if (pid->err_sum > high) pid->err_sum = high;
}

int constrain_double(double amt, double low, double high)
{
    return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

float get_sensor_actual(void)
{
    float numerator = 0;
    float denominator = 0;
    Grayscale_Sensor_Read_All();

    for (int i = 0; i < 8; i++) {
        if (sensor_data[i] == 1) {
            numerator   += sensor_weight[i];
            denominator += 1.0;
        }
    }

    if (denominator == 0) {
        None_flag = 1;
        return 0;
    }

    None_flag = 0;
    return numerator / denominator;
}

void xunji_pid(void)
{
    float sensor_actual = get_sensor_actual();
    float abs_err;
    float spd = (float)base_speed;

    if (None_flag == 1) {
        /*
         * Keep the previous arc through isolated read gaps.  A confirmed loss
         * requests a forward-only search arc; no motor direction reversal is
         * used, even while searching.
         */
        if (line_lost_count < LINE_LOST_CONFIRM) {
            line_lost_count++;
        }
        if (line_lost_count < LINE_LOST_CONFIRM) {
            return;
        }

        if (Pre > 0.1f) {
            drive_speed_target = spd * LOST_SPEED_RATIO;
            drive_steer_target = -spd * LOST_STEER_RATIO;
        } else if (Pre < -0.1f) {
            drive_speed_target = spd * LOST_SPEED_RATIO;
            drive_steer_target = spd * LOST_STEER_RATIO;
        } else {
            drive_speed_target = spd * MIN_CRUISE_RATIO;
            drive_steer_target = 0.0f;
        }
        g_in_pivot = 1;
        return;
    }

    line_lost_count = 0;
    g_in_pivot = 0;

    /* Low-pass the discrete 8-channel position before feeding the PD loop. */
    if (!sensor_filter_ready) {
        filtered_sensor = sensor_actual;
        sensor_filter_ready = 1;
    } else {
        filtered_sensor += SENSOR_FILTER_ALPHA *
                           (sensor_actual - filtered_sensor);
    }
    sensor_actual = filtered_sensor;

    /* The two centre sensors both have zero weight; suppress tiny edge jitter. */
    if (sensor_actual > -SENSOR_DEADBAND &&
        sensor_actual < SENSOR_DEADBAND) {
        sensor_actual = 0.0f;
    }

    if (sensor_actual > 0.1f) {
        Pre = 1.0f;
    } else if (sensor_actual < -0.1f) {
        Pre = -1.0f;
    }

    abs_err = (sensor_actual > 0) ? sensor_actual : -sensor_actual;

    /*
     * Pure proportional line control:
     *
     *   speed = base - gain * |error|
     *   steer = -gain * error
     *   left  = speed + steer
     *   right = speed - steer
     *
     * Therefore the outside wheel stays at base speed while only the inside
     * wheel slows.  There is no D kick, mode boundary, or alternating wheel
     * acceleration to excite chassis oscillation.
     */
    drive_speed_target = spd - TRACK_STEER_GAIN * abs_err;
    if (drive_speed_target < spd * MIN_CRUISE_RATIO) {
        drive_speed_target = spd * MIN_CRUISE_RATIO;
    }

    drive_steer_target = -TRACK_STEER_GAIN * sensor_actual;
    if (drive_steer_target > spd * LOST_STEER_RATIO) {
        drive_steer_target = spd * LOST_STEER_RATIO;
    }
    if (drive_steer_target < -spd * LOST_STEER_RATIO) {
        drive_steer_target = -spd * LOST_STEER_RATIO;
    }
}

void TIMER_0_INST_IRQHandler(void)
{
    /* Reading IIDX acknowledges the timer event on MSPM0. */
    if (DL_TimerG_getPendingInterrupt(TIMER_0_INST) != DL_TIMER_IIDX_ZERO) {
        return;
    }

    /*
     * Apply a tiny motor step every 1 ms.  Sensor/PD work remains at 7 ms, but
     * it only changes targets and can no longer kick PWM or direction.
     */
    if (xunji_flag >= 1) {
        Motor_Smooth_Update();
    }

    /* 1 ms timer tick -> one steering target update every 7 ms. */
    if (++Tick.turnerrorpid >= 7) {
        if (xunji_flag >= 1) {
            xunji_pid();
        }
        Tick.turnerrorpid = 0;
    }
}
