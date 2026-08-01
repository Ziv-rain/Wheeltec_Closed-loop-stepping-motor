#include "history_logger.h"

#include "board.h"
#include "closed_loop.h"
#include "mech_balance.h"

#define HISTORY_LOG_VERSION       1U
#define HISTORY_FLAG_OVERFLOW     (1U << 0)
#define HISTORY_SAMPLE_VALID      (1U << 0)
#define HISTORY_SAMPLE_PID_ACTIVE (1U << 1)
#define HISTORY_SAMPLE_FF_MERGED  (1U << 2)
#define HISTORY_SAMPLE_CL_ACTIVE  (1U << 3)

typedef struct {
    uint32_t source_tick_ms;
    int16_t ball_pos_centi_cm;
    int16_t ball_velocity_centi_cm_s;
    int16_t accel_milli_mps2;
    int16_t pid_out_centi_deg;
    int16_t ff_angle_centi_deg;
    int16_t target_angle_centi_deg;
    int16_t actual_angle_centi_deg;
    uint8_t flags;
    uint8_t task_id;
} HistoryRecord_t;

typedef char HistoryRecordMustStay20Bytes[
    (sizeof(HistoryRecord_t) == 20U) ? 1 : -1];

static HistoryRecord_t s_records[HISTORY_LOG_CAPACITY];
static volatile uint16_t s_count;
static volatile uint8_t s_recording;
static volatile uint8_t s_overflow;
static volatile uint8_t s_dumping;
static uint16_t s_dump_index;
static uint16_t s_session_id;
static uint8_t s_task_id;
static uint8_t s_dump_header_pending;

static int16_t scale_i16(float value, float scale)
{
    float scaled;

    /* NaN is the only floating-point value that is not equal to itself. */
    if (!(value == value)) return 0;
    scaled = value * scale;
    if (scaled > 32767.0f) scaled = 32767.0f;
    if (scaled < -32768.0f) scaled = -32768.0f;
    if (scaled >= 0.0f) scaled += 0.5f;
    else scaled -= 0.5f;
    return (int16_t)scaled;
}

static void print_i16(int16_t value)
{
    uart_puti((int32_t)value);
}

void HistoryLogger_Init(void)
{
    s_count = 0U;
    s_recording = 0U;
    s_overflow = 0U;
    s_dumping = 0U;
    s_dump_index = 0U;
    s_session_id = 0U;
    s_task_id = 0U;
    s_dump_header_pending = 0U;
}

void HistoryLogger_Start(uint8_t task_id)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_recording = 0U;
    s_dumping = 0U;
    s_count = 0U;
    s_dump_index = 0U;
    s_overflow = 0U;
    s_dump_header_pending = 0U;
    s_task_id = task_id;
    s_session_id++;
    if (s_session_id == 0U) s_session_id = 1U;
    s_recording = 1U;
    if (!primask) __enable_irq();
}

void HistoryLogger_Stop(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_recording = 0U;
    if (!primask) __enable_irq();
}

void HistoryLogger_Capture(const WheelTelemetry_t *wheel)
{
    VisionStatus_t vision;
    CL_Snapshot_t motor;
    HistoryRecord_t *record;

    if (wheel == 0 || s_recording == 0U) return;
    if (s_count >= HISTORY_LOG_CAPACITY) {
        s_overflow = 1U;
        s_recording = 0U;
        return;
    }

    MechBalance_GetVisionStatus(&vision);
    CL_GetSnapshot(MOTOR_AXIS_X, &motor);

    record = &s_records[s_count];
    record->source_tick_ms = wheel->source_tick_ms;
    record->ball_pos_centi_cm = scale_i16(vision.ball_pos_cm, 100.0f);
    record->ball_velocity_centi_cm_s =
        scale_i16(vision.ball_velocity_cm_s, 100.0f);
    record->accel_milli_mps2 = scale_i16(vision.accel_mps2, 1000.0f);
    record->pid_out_centi_deg = scale_i16(vision.pid_out_deg, 100.0f);
    record->ff_angle_centi_deg = scale_i16(vision.ff_angle_deg, 100.0f);
    record->target_angle_centi_deg =
        scale_i16(motor.target_angle_deg, 100.0f);
    record->actual_angle_centi_deg =
        scale_i16(motor.current_angle_deg, 100.0f);
    record->flags = 0U;
    if (vision.valid != 0U) record->flags |= HISTORY_SAMPLE_VALID;
    if (vision.active != 0U) record->flags |= HISTORY_SAMPLE_PID_ACTIVE;
    if (vision.ff_merged != 0U) record->flags |= HISTORY_SAMPLE_FF_MERGED;
    if (motor.active != 0U) record->flags |= HISTORY_SAMPLE_CL_ACTIVE;
    record->task_id = s_task_id;
    s_count++;
}

uint8_t HistoryLogger_RequestDump(void)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t accepted = 0U;

    __disable_irq();
    if (s_recording == 0U && s_dumping == 0U && s_count != 0U) {
        s_dump_index = 0U;
        s_dump_header_pending = 1U;
        s_dumping = 1U;
        accepted = 1U;
    }
    if (!primask) __enable_irq();
    return accepted;
}

void HistoryLogger_Process(void)
{
    const HistoryRecord_t *record;

    if (s_dumping == 0U) return;

    if (s_dump_header_pending != 0U) {
        s_dump_header_pending = 0U;
        uart_puts("BALLLOG_BEGIN,"); uart_putu(HISTORY_LOG_VERSION);
        uart_putc(','); uart_putu(s_session_id);
        uart_putc(','); uart_putu(s_task_id);
        uart_putc(','); uart_putu(s_count);
        uart_putc(','); uart_putu(s_overflow ? HISTORY_FLAG_OVERFLOW : 0U);
        uart_puts("\r\n");
        return;
    }

    if (s_dump_index < s_count) {
        record = &s_records[s_dump_index];
        uart_puts("BALLLOG,"); uart_putu(s_session_id);
        uart_putc(','); uart_putu(s_dump_index);
        uart_putc(','); uart_putu(record->source_tick_ms);
        uart_putc(','); print_i16(record->ball_pos_centi_cm);
        uart_putc(','); print_i16(record->ball_velocity_centi_cm_s);
        uart_putc(','); print_i16(record->accel_milli_mps2);
        uart_putc(','); print_i16(record->pid_out_centi_deg);
        uart_putc(','); print_i16(record->ff_angle_centi_deg);
        uart_putc(','); print_i16(record->target_angle_centi_deg);
        uart_putc(','); print_i16(record->actual_angle_centi_deg);
        uart_putc(','); uart_putu(record->flags);
        uart_putc(','); uart_putu(record->task_id);
        uart_puts("\r\n");
        s_dump_index++;
        return;
    }

    uart_puts("BALLLOG_END,"); uart_putu(HISTORY_LOG_VERSION);
    uart_putc(','); uart_putu(s_session_id);
    uart_putc(','); uart_putu(s_task_id);
    uart_putc(','); uart_putu(s_count);
    uart_putc(','); uart_putu(s_overflow ? HISTORY_FLAG_OVERFLOW : 0U);
    uart_puts("\r\n");
    s_dumping = 0U;
}

uint8_t HistoryLogger_IsRecording(void)
{
    return s_recording;
}

uint8_t HistoryLogger_IsDumping(void)
{
    return s_dumping;
}
