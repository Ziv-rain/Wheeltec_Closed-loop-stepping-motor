#include "encoder_logger.h"

#include "aa55_proto.h"
#include "Drive.h"
#include "Odometry.h"
#include "grayscale_sensor.h"
#include "ti_msp_dl_config.h"

extern uint8_t base_speed;

#define LOG_PROTOCOL_VERSION 1U
#define LOG_META_BEGIN       1U
#define LOG_META_END         2U
#define LOG_FLAG_OVERFLOW    (1U << 0)

typedef struct {
    uint32_t tick_ms;
    uint16_t delta_left;
    uint16_t delta_right;
    uint8_t  pwm_left;
    uint8_t  pwm_right;
    uint8_t  base_speed;
    uint8_t  line_mask;
} EncoderLogRecord_t;

typedef char EncoderLogRecordMustStay12Bytes[
    (sizeof(EncoderLogRecord_t) == 12U) ? 1 : -1];

static EncoderLogRecord_t s_records[ENCODER_LOG_CAPACITY];
static volatile uint16_t s_count;
static volatile uint8_t s_recording;
static volatile uint8_t s_overflow;
static volatile uint8_t s_dumping;
static uint16_t s_dump_index;
static uint16_t s_run_id;
static uint8_t s_task_id;
static uint8_t s_meta_sent;
static uint32_t s_previous_left;
static uint32_t s_previous_right;
static uint32_t s_last_sample_tick;

static void put_u16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)(value >> 8U);
}

static void put_u32(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8U) & 0xFFU);
    dst[2] = (uint8_t)((value >> 16U) & 0xFFU);
    dst[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static uint8_t read_line_mask(void)
{
    uint8_t mask = 0U;
    uint8_t i;

    for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS && i < 8U; i++) {
        if (sensor_data[i] != 0U) {
            mask |= (uint8_t)(1U << i);
        }
    }
    return mask;
}

static void send_meta(uint8_t kind)
{
    uint8_t data[8];

    data[0] = kind;
    data[1] = LOG_PROTOCOL_VERSION;
    put_u16(&data[2], s_run_id);
    data[4] = s_task_id;
    data[5] = s_overflow ? LOG_FLAG_OVERFLOW : 0U;
    put_u16(&data[6], s_count);
    AA55_SendFrame(AA55_TYPE_LOG_META, data, 8U);
}

static void send_record(uint16_t index, const EncoderLogRecord_t *record)
{
    uint8_t data[8];

    /* 0x44: run, sample index, source timestamp. */
    put_u16(&data[0], s_run_id);
    put_u16(&data[2], index);
    put_u32(&data[4], record->tick_ms);
    AA55_SendFrame(AA55_TYPE_LOG_TIME, data, 8U);

    /* 0x45: run, sample index, left/right count increments. */
    put_u16(&data[0], s_run_id);
    put_u16(&data[2], index);
    put_u16(&data[4], record->delta_left);
    put_u16(&data[6], record->delta_right);
    AA55_SendFrame(AA55_TYPE_LOG_COUNTS, data, 8U);

    /* 0x46: run, sample index, actual drive request and line snapshot. */
    put_u16(&data[0], s_run_id);
    put_u16(&data[2], index);
    data[4] = record->pwm_left;
    data[5] = record->pwm_right;
    data[6] = record->base_speed;
    data[7] = record->line_mask;
    AA55_SendFrame(AA55_TYPE_LOG_DRIVE, data, 8U);
}

void EncoderLogger_Init(void)
{
    s_count = 0U;
    s_recording = 0U;
    s_overflow = 0U;
    s_dumping = 0U;
    s_dump_index = 0U;
    s_run_id = 0U;
    s_task_id = 0U;
    s_meta_sent = 0U;
    s_previous_left = 0U;
    s_previous_right = 0U;
    s_last_sample_tick = 0U;
}

void EncoderLogger_Start(uint8_t task_id, uint32_t tick_ms)
{
    uint32_t primask = __get_PRIMASK();
    (void)tick_ms;

    __disable_irq();
    s_recording = 0U;
    s_dumping = 0U;
    s_count = 0U;
    s_dump_index = 0U;
    s_overflow = 0U;
    s_meta_sent = 0U;
    s_task_id = task_id;
    s_run_id++;
    if (s_run_id == 0U) {
        s_run_id = 1U;
    }
    Odometry_GetSnapshot(&s_previous_left, &s_previous_right);
    s_recording = 1U;
    if (!primask) {
        __enable_irq();
    }
}

void EncoderLogger_Stop(uint32_t tick_ms)
{
    uint32_t primask = __get_PRIMASK();
    (void)tick_ms;

    __disable_irq();
    s_recording = 0U;
    if (!primask) {
        __enable_irq();
    }
}

uint8_t EncoderLogger_RequestDump(void)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t accepted = 0U;

    __disable_irq();
    if (s_recording == 0U && s_dumping == 0U && s_count != 0U) {
        s_dump_index = 0U;
        s_meta_sent = 0U;
        s_dumping = 1U;
        accepted = 1U;
    }
    if (!primask) {
        __enable_irq();
    }
    return accepted;
}

void EncoderLogger_Tick1ms(uint32_t tick_ms)
{
    uint32_t left;
    uint32_t right;
    uint32_t delta_left;
    uint32_t delta_right;
    EncoderLogRecord_t *record;

    if ((tick_ms - s_last_sample_tick) < ENCODER_LOG_SAMPLE_MS) {
        return;
    }
    s_last_sample_tick = tick_ms;

    Odometry_GetSnapshot(&left, &right);

    /* Keep the existing 0x42 frame format, but latch counts and time together. */
    AA55_UpdateEncoderSnapshot((uint16_t)left, (uint16_t)right, tick_ms);

    if (s_recording == 0U) {
        return;
    }

    delta_left = left - s_previous_left;
    delta_right = right - s_previous_right;
    s_previous_left = left;
    s_previous_right = right;

    if (s_count >= ENCODER_LOG_CAPACITY) {
        s_overflow = 1U;
        s_recording = 0U;
        return;
    }

    if (delta_left > 65535U) {
        delta_left = 65535U;
        s_overflow = 1U;
    }
    if (delta_right > 65535U) {
        delta_right = 65535U;
        s_overflow = 1U;
    }

    record = &s_records[s_count];
    record->tick_ms = tick_ms;
    record->delta_left = (uint16_t)delta_left;
    record->delta_right = (uint16_t)delta_right;
    record->pwm_left = Drive_GetLeftDutyPercent();
    record->pwm_right = Drive_GetRightDutyPercent();
    record->base_speed = base_speed;
    record->line_mask = read_line_mask();
    s_count++;
}

void EncoderLogger_Process(void)
{
    if (s_dumping == 0U) {
        return;
    }

    if (s_meta_sent == 0U) {
        send_meta(LOG_META_BEGIN);
        s_meta_sent = 1U;
        return;
    }

    if (s_dump_index < s_count) {
        send_record(s_dump_index, &s_records[s_dump_index]);
        s_dump_index++;
        return;
    }

    send_meta(LOG_META_END);
    s_dumping = 0U;
}

uint8_t EncoderLogger_IsRecording(void)
{
    return s_recording;
}

uint8_t EncoderLogger_IsDumping(void)
{
    return s_dumping;
}
