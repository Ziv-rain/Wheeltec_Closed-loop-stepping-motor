/** task_ctrl.c - task state machine, AA55 link and wheel telemetry */
#include "task_ctrl.h"
#include "ti_msp_dl_config.h"
#include "board.h"

#define TASK3_SETTLE_MS       500U
#define TASK3_SETTLE_TOL      1.0f
#define TARGET_SETTLE_MS      500U
#define TARGET_SETTLE_TOL     1.0f
#define HB_INTERVAL          1000U
#define STATE_INTERVAL        100U
#define TASK3_TOTAL_MS       5000U
#define LINK_TIMEOUT_MS      2500U
#define TX_PENDING_STATE  (1U << 0)
#define TX_PENDING_HB     (1U << 1)

#define CMD_QUEUE_SIZE 8U
#define CMD_QUEUE_MASK (CMD_QUEUE_SIZE - 1U)

#define AA55_TYPE_TASK_CMD   0x40U
#define AA55_TYPE_ENCODER    0x42U
#define AA55_TYPE_LOG_META   0x43U
#define AA55_TYPE_LOG_TIME   0x44U
#define AA55_TYPE_LOG_COUNTS 0x45U
#define AA55_TYPE_LOG_DRIVE  0x46U
#define AA55_CMD_DUMP_LOG    0x05U

#define WENC_SCALE 0.0002618f

#define CAR_LOG_QUEUE_SIZE 16U
#define CAR_LOG_QUEUE_MASK (CAR_LOG_QUEUE_SIZE - 1U)
#define LOG_META_BEGIN 1U
#define LOG_META_END   2U

typedef struct {
    uint8_t cmd;
    uint8_t task;
    int16_t param;
} TCmd_t;

typedef struct {
    uint8_t state;
    uint8_t type;
    uint8_t len;
    uint8_t index;
    uint8_t data[8];
    uint16_t crc;
} AA55_t;

typedef struct {
    uint16_t run_id;
    uint16_t sample_index;
    uint32_t tick_ms;
    uint16_t delta_left;
    uint16_t delta_right;
    uint8_t pwm_left;
    uint8_t pwm_right;
    uint8_t base_speed;
    uint8_t line_mask;
} CarLogRecord_t;

typedef struct {
    uint8_t kind;
    uint8_t version;
    uint16_t run_id;
    uint8_t task;
    uint8_t flags;
    uint16_t sample_count;
} CarLogMeta_t;

static AA55_t s_uart1_parser;
static AA55_t s_debug_parser;
static volatile TCmd_t s_cmd_queue[CMD_QUEUE_SIZE];
static volatile uint8_t s_cmd_head;
static volatile uint8_t s_cmd_tail;

static struct {
    uint8_t task_id;
    uint8_t state;
    float setpoint_cm;
    uint32_t run_time_ms;
    float ball_pos_cm;
    uint8_t ball_valid;
    volatile uint8_t emergency_stop;
    volatile uint8_t queue_overflow;
    volatile uint8_t link_alive;
    volatile uint8_t pending;
    uint32_t settle_ms;
    uint32_t last_rx_ms;
} s_task;

static volatile uint32_t s_tick_ms;
static uint32_t s_last_hb_ms;
static uint32_t s_last_state_ms;

/* Existing 0x42 acceleration path. Its constants and behavior are unchanged. */
static volatile uint32_t s_wheel_left;
static volatile uint32_t s_wheel_right;
static volatile uint32_t s_wheel_tick;
static volatile uint32_t s_wheel_left_previous;
static volatile uint32_t s_wheel_right_previous;
static volatile uint32_t s_wheel_tick_previous;
static volatile float s_wheel_speed_mps;
static volatile float s_wheel_accel_mps2;
static volatile uint8_t s_wheel_accel_valid;
static volatile uint32_t s_wheel_accel_frame;
static volatile uint32_t s_wheel_accel_tick;
static volatile uint32_t s_wheel_accel_rx_frame;
static volatile uint32_t s_wheel_rx_frame;
static volatile uint32_t s_wheel_age_ms;

/* Post-stop car-log receiver. UART1 ISR only assembles and queues records. */
static volatile CarLogRecord_t s_car_log_queue[CAR_LOG_QUEUE_SIZE];
static volatile uint8_t s_car_log_head;
static volatile uint8_t s_car_log_tail;
static volatile uint32_t s_car_log_dropped;
static CarLogRecord_t s_car_log_stage;
static uint8_t s_car_log_stage_bits;
static volatile CarLogMeta_t s_car_log_begin;
static volatile CarLogMeta_t s_car_log_end;
static volatile uint8_t s_car_log_begin_pending;
static volatile uint8_t s_car_log_end_pending;

/* Diagnostics are emitted from the main loop, never from the 5 ms ISR. */
static volatile uint8_t s_diag;
static volatile uint8_t s_diag_task;
static volatile float s_diag_setpoint;

#define DIAG_TASK_OK    1U
#define DIAG_TASK_ERR   2U
#define DIAG_START      3U
#define DIAG_START_T4   4U
#define DIAG_NO_TASK    5U
#define DIAG_STOP       6U
#define DIAG_SP         7U
#define DIAG_NOT_T6     8U

static uint16_t get_u16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static uint32_t get_u32(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

static uint16_t crc16_update(uint16_t crc, uint8_t byte)
{
    uint8_t i;
    crc ^= byte;
    for (i = 0U; i < 8U; i++) {
        crc = (crc & 1U) ? (uint16_t)((crc >> 1U) ^ 0xA001U) :
                           (uint16_t)(crc >> 1U);
    }
    return crc;
}

static uint16_t crc16_frame(uint8_t type, uint8_t len, const uint8_t *data)
{
    uint16_t crc = 0U;
    uint8_t i;
    crc = crc16_update(crc, type);
    crc = crc16_update(crc, len);
    for (i = 0U; i < len; i++) {
        crc = crc16_update(crc, data[i]);
    }
    return crc;
}

static uint8_t aa55_parse(
    AA55_t *parser, uint8_t byte, uint8_t *type, uint8_t *len, uint8_t **data)
{
    uint16_t received_crc;

    switch (parser->state) {
    case 0U:
        if (byte == 0xAAU) parser->state = 1U;
        break;
    case 1U:
        if (byte == 0x55U) parser->state = 2U;
        else parser->state = (byte == 0xAAU) ? 1U : 0U;
        break;
    case 2U:
        parser->type = byte;
        parser->state = 3U;
        break;
    case 3U:
        parser->len = byte;
        parser->index = 0U;
        if (parser->len > sizeof(parser->data)) {
            parser->state = 0U;
        } else {
            parser->state = (parser->len != 0U) ? 4U : 5U;
        }
        break;
    case 4U:
        parser->data[parser->index++] = byte;
        if (parser->index >= parser->len) parser->state = 5U;
        break;
    case 5U:
        parser->crc = byte;
        parser->state = 6U;
        break;
    default:
        received_crc = (uint16_t)parser->crc | ((uint16_t)byte << 8U);
        parser->state = 0U;
        if (received_crc ==
            crc16_frame(parser->type, parser->len, parser->data)) {
            *type = parser->type;
            *len = parser->len;
            *data = parser->data;
            return 1U;
        }
        break;
    }
    return 0U;
}

static void uart1_write(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    for (i = 0U; i < len; i++) {
        while (DL_UART_Main_isTXFIFOFull(UART_1_INST)) {}
        DL_UART_Main_transmitData(UART_1_INST, data[i]);
    }
}

static void send_frame(uint8_t type, const uint8_t *data, uint8_t len)
{
    uint8_t frame[16];
    uint16_t crc;
    uint8_t i;

    if (len > 8U) return;
    frame[0] = 0xAAU;
    frame[1] = 0x55U;
    frame[2] = type;
    frame[3] = len;
    for (i = 0U; i < len; i++) frame[4U + i] = data[i];
    crc = crc16_frame(type, len, data);
    frame[4U + len] = (uint8_t)crc;
    frame[5U + len] = (uint8_t)(crc >> 8U);
    uart1_write(frame, (uint16_t)len + 6U);
}

static void send_state(uint8_t task_id, uint8_t state, float ball_pos_cm)
{
    uint8_t data[6];
    float scaled = ball_pos_cm * 100.0f;
    int16_t ball;

    if (scaled > 32767.0f) scaled = 32767.0f;
    if (scaled < -32768.0f) scaled = -32768.0f;
    ball = (int16_t)scaled;
    data[0] = task_id;
    data[1] = state;
    data[2] = (uint8_t)(uint16_t)ball;
    data[3] = (uint8_t)((uint16_t)ball >> 8U);
    data[4] = 0U;
    data[5] = 0U;
    send_frame(0x41U, data, 6U);
}

static uint8_t command_pop(TCmd_t *command)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t tail;

    __disable_irq();
    tail = s_cmd_tail;
    if (tail == s_cmd_head) {
        if (!primask) __enable_irq();
        return 0U;
    }
    command->cmd = s_cmd_queue[tail].cmd;
    command->task = s_cmd_queue[tail].task;
    command->param = s_cmd_queue[tail].param;
    s_cmd_tail = (uint8_t)((tail + 1U) & CMD_QUEUE_MASK);
    if (!primask) __enable_irq();
    return 1U;
}

static void command_push(uint8_t cmd, uint8_t task, int16_t param)
{
    uint32_t primask;
    uint8_t head;
    uint8_t next;

    if (cmd < 1U || cmd > 4U) return;
    primask = __get_PRIMASK();
    __disable_irq();
    if (cmd == 3U) {
        s_task.emergency_stop = 1U;
        if (!primask) __enable_irq();
        return;
    }
    head = s_cmd_head;
    next = (uint8_t)((head + 1U) & CMD_QUEUE_MASK);
    if (next == s_cmd_tail) {
        s_task.queue_overflow = 1U;
    } else {
        s_cmd_queue[head].cmd = cmd;
        s_cmd_queue[head].task = task;
        s_cmd_queue[head].param = param;
        s_cmd_head = next;
    }
    if (!primask) __enable_irq();
}

static void execute_command(const TCmd_t *command)
{
    switch (command->cmd) {
    case 1U:
        if (command->task >= TASK_2 && command->task <= TASK_6) {
            s_task.task_id = command->task;
            s_task.state = STATE_IDLE;
            s_task.setpoint_cm = 0.0f;
            s_task.run_time_ms = 0U;
            s_task.settle_ms = 0U;
            s_task.ball_valid = 0U;
            s_diag = DIAG_TASK_OK;
            s_diag_task = command->task;
        } else {
            s_diag = DIAG_TASK_ERR;
        }
        break;
    case 2U:
        if (s_task.task_id == TASK_3) {
            s_task.state = STATE_RUNNING;
            s_task.run_time_ms = 0U;
            s_task.settle_ms = 0U;
            s_diag = DIAG_START;
        } else if (s_task.task_id == TASK_4 || s_task.task_id == TASK_5) {
            s_task.state = STATE_RUNNING;
            s_task.run_time_ms = 0U;
            s_task.settle_ms = 0U;
            s_diag = DIAG_START_T4;
            s_diag_task = s_task.task_id;
        } else if (s_task.task_id == TASK_6) {
            s_task.state = STATE_RUNNING;
            s_task.run_time_ms = 0U;
            s_task.settle_ms = 0U;
            s_diag = DIAG_START;
        } else {
            s_diag = DIAG_NO_TASK;
        }
        break;
    case 3U:
        s_task.state = STATE_IDLE;
        s_task.setpoint_cm = 0.0f;
        s_task.run_time_ms = 0U;
        s_task.settle_ms = 0U;
        s_task.ball_valid = 0U;
        s_diag = DIAG_STOP;
        break;
    case 4U:
        if (s_task.task_id == TASK_6) {
            s_task.setpoint_cm = (float)command->param / 100.0f;
            s_task.settle_ms = 0U;
            s_diag = DIAG_SP;
            s_diag_setpoint = s_task.setpoint_cm;
        } else {
            s_diag = DIAG_NOT_T6;
        }
        break;
    default:
        break;
    }
    s_task.pending |= TX_PENDING_STATE;
}

static void execute_queue(void)
{
    TCmd_t command;
    uint32_t primask = __get_PRIMASK();
    uint8_t emergency;
    uint8_t overflow;

    __disable_irq();
    emergency = s_task.emergency_stop;
    overflow = s_task.queue_overflow;
    s_task.emergency_stop = 0U;
    s_task.queue_overflow = 0U;
    if (emergency != 0U || overflow != 0U) s_cmd_tail = s_cmd_head;
    if (!primask) __enable_irq();

    if (emergency != 0U) {
        command.cmd = 3U;
        command.task = s_task.task_id;
        command.param = 0;
        execute_command(&command);
        return;
    }
    if (overflow != 0U) {
        TaskCtrl_ReportFault();
        return;
    }
    while (command_pop(&command) != 0U) execute_command(&command);
}

static float task3_trajectory(uint32_t ms)
{
    if (ms >= TASK3_TOTAL_MS) return -5.0f;
    if (ms < 2000U) return 5.0f * (float)ms / 2000.0f;
    if (ms < 2500U) return 5.0f - 5.0f * (float)(ms - 2000U) / 500.0f;
    if (ms < 4500U) return -5.0f * (float)(ms - 2500U) / 2000.0f;
    return -5.0f;
}

static void check_task3_settled(void)
{
    float error;
    if (s_task.run_time_ms < 4500U) return;
    error = s_task.ball_pos_cm + 5.0f;
    if (error < 0.0f) error = -error;
    if (s_task.ball_valid != 0U && error < TASK3_SETTLE_TOL) {
        s_task.settle_ms += 5U;
        if (s_task.settle_ms >= TASK3_SETTLE_MS) s_task.state = STATE_DONE;
    } else {
        s_task.settle_ms = 0U;
    }
}

static void check_task6_settled(void)
{
    float error = s_task.ball_pos_cm - s_task.setpoint_cm;
    if (error < 0.0f) error = -error;
    if (s_task.ball_valid != 0U && error < TARGET_SETTLE_TOL) {
        s_task.settle_ms += 5U;
        if (s_task.settle_ms >= TARGET_SETTLE_MS) s_task.state = STATE_DONE;
    } else {
        s_task.settle_ms = 0U;
    }
}

static void handle_wheel_frame(const uint8_t *data)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_wheel_left_previous = s_wheel_left;
    s_wheel_right_previous = s_wheel_right;
    s_wheel_tick_previous = s_wheel_tick;
    s_wheel_left = get_u16(&data[0]);
    s_wheel_right = get_u16(&data[2]);
    s_wheel_tick = get_u32(&data[4]);
    s_wheel_rx_frame++;

    if (s_wheel_tick_previous != 0U && s_wheel_tick > s_wheel_tick_previous) {
        float dt = (float)(s_wheel_tick - s_wheel_tick_previous) / 1000.0f;
        int32_t delta_left = (int32_t)(int16_t)
            ((uint16_t)s_wheel_left - (uint16_t)s_wheel_left_previous);
        int32_t delta_right = (int32_t)(int16_t)
            ((uint16_t)s_wheel_right - (uint16_t)s_wheel_right_previous);

        if (dt >= 0.01f && dt <= 0.5f &&
            delta_left > -500 && delta_left < 500 &&
            delta_right > -500 && delta_right < 500) {
            float distance_left = (float)delta_left * WENC_SCALE;
            float distance_right = (float)delta_right * WENC_SCALE;
            float speed = (distance_left + distance_right) * 0.5f / dt;
            float acceleration = (speed - s_wheel_speed_mps) / dt;

            s_wheel_speed_mps = speed;
            s_wheel_age_ms = 0U;
            if (acceleration > -3.0f && acceleration < 3.0f) {
                s_wheel_accel_mps2 = acceleration;
                s_wheel_accel_valid = 1U;
                s_wheel_accel_frame++;
                s_wheel_accel_tick = s_wheel_tick;
                s_wheel_accel_rx_frame = s_wheel_rx_frame;
            }
        }
    }
    if (!primask) __enable_irq();
}

static void car_log_reset_stream(void)
{
    s_car_log_stage_bits = 0U;
    s_car_log_head = 0U;
    s_car_log_tail = 0U;
    s_car_log_dropped = 0U;
}

static void car_log_queue_stage(void)
{
    uint8_t head = s_car_log_head;
    uint8_t next = (uint8_t)((head + 1U) & CAR_LOG_QUEUE_MASK);

    if (next == s_car_log_tail) {
        s_car_log_dropped++;
        return;
    }
    s_car_log_queue[head].run_id = s_car_log_stage.run_id;
    s_car_log_queue[head].sample_index = s_car_log_stage.sample_index;
    s_car_log_queue[head].tick_ms = s_car_log_stage.tick_ms;
    s_car_log_queue[head].delta_left = s_car_log_stage.delta_left;
    s_car_log_queue[head].delta_right = s_car_log_stage.delta_right;
    s_car_log_queue[head].pwm_left = s_car_log_stage.pwm_left;
    s_car_log_queue[head].pwm_right = s_car_log_stage.pwm_right;
    s_car_log_queue[head].base_speed = s_car_log_stage.base_speed;
    s_car_log_queue[head].line_mask = s_car_log_stage.line_mask;
    s_car_log_head = next;
}

static void handle_log_meta(const uint8_t *data)
{
    CarLogMeta_t meta;
    meta.kind = data[0];
    meta.version = data[1];
    meta.run_id = get_u16(&data[2]);
    meta.task = data[4];
    meta.flags = data[5];
    meta.sample_count = get_u16(&data[6]);

    if (meta.kind == LOG_META_BEGIN) {
        car_log_reset_stream();
        s_car_log_begin = meta;
        s_car_log_begin_pending = 1U;
    } else if (meta.kind == LOG_META_END) {
        s_car_log_end = meta;
        s_car_log_end_pending = 1U;
    }
}

static void handle_log_time(const uint8_t *data)
{
    s_car_log_stage.run_id = get_u16(&data[0]);
    s_car_log_stage.sample_index = get_u16(&data[2]);
    s_car_log_stage.tick_ms = get_u32(&data[4]);
    s_car_log_stage_bits = 1U;
}

static void handle_log_counts(const uint8_t *data)
{
    if (s_car_log_stage_bits != 1U ||
        s_car_log_stage.run_id != get_u16(&data[0]) ||
        s_car_log_stage.sample_index != get_u16(&data[2])) {
        s_car_log_stage_bits = 0U;
        s_car_log_dropped++;
        return;
    }
    s_car_log_stage.delta_left = get_u16(&data[4]);
    s_car_log_stage.delta_right = get_u16(&data[6]);
    s_car_log_stage_bits = 3U;
}

static void handle_log_drive(const uint8_t *data)
{
    if (s_car_log_stage_bits != 3U ||
        s_car_log_stage.run_id != get_u16(&data[0]) ||
        s_car_log_stage.sample_index != get_u16(&data[2])) {
        s_car_log_stage_bits = 0U;
        s_car_log_dropped++;
        return;
    }
    s_car_log_stage.pwm_left = data[4];
    s_car_log_stage.pwm_right = data[5];
    s_car_log_stage.base_speed = data[6];
    s_car_log_stage.line_mask = data[7];
    car_log_queue_stage();
    s_car_log_stage_bits = 0U;
}

static void handle_uart1_frame(uint8_t type, uint8_t len, const uint8_t *data)
{
    s_task.last_rx_ms = s_tick_ms;
    s_task.link_alive = 1U;

    if (type == AA55_TYPE_TASK_CMD && len == 4U) {
        command_push(data[0], data[1], (int16_t)get_u16(&data[2]));
    } else if (type == AA55_TYPE_ENCODER && len == 8U) {
        handle_wheel_frame(data);
    } else if (type == AA55_TYPE_LOG_META && len == 8U) {
        handle_log_meta(data);
    } else if (type == AA55_TYPE_LOG_TIME && len == 8U) {
        handle_log_time(data);
    } else if (type == AA55_TYPE_LOG_COUNTS && len == 8U) {
        handle_log_counts(data);
    } else if (type == AA55_TYPE_LOG_DRIVE && len == 8U) {
        handle_log_drive(data);
    }
}

static uint8_t car_log_pop(CarLogRecord_t *record)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t tail;

    __disable_irq();
    tail = s_car_log_tail;
    if (tail == s_car_log_head) {
        if (!primask) __enable_irq();
        return 0U;
    }
    record->run_id = s_car_log_queue[tail].run_id;
    record->sample_index = s_car_log_queue[tail].sample_index;
    record->tick_ms = s_car_log_queue[tail].tick_ms;
    record->delta_left = s_car_log_queue[tail].delta_left;
    record->delta_right = s_car_log_queue[tail].delta_right;
    record->pwm_left = s_car_log_queue[tail].pwm_left;
    record->pwm_right = s_car_log_queue[tail].pwm_right;
    record->base_speed = s_car_log_queue[tail].base_speed;
    record->line_mask = s_car_log_queue[tail].line_mask;
    s_car_log_tail = (uint8_t)((tail + 1U) & CAR_LOG_QUEUE_MASK);
    if (!primask) __enable_irq();
    return 1U;
}

static void print_log_meta(const char *kind, const CarLogMeta_t *meta)
{
    uart_puts("CARLOG_"); uart_puts(kind);
    uart_putc(','); uart_putu(meta->version);
    uart_putc(','); uart_putu(meta->run_id);
    uart_putc(','); uart_putu(meta->task);
    uart_putc(','); uart_putu(meta->sample_count);
    uart_putc(','); uart_putu(meta->flags);
    uart_putc(','); uart_putu(s_car_log_dropped);
    uart_puts("\r\n");
}

static void process_car_log_output(void)
{
    CarLogMeta_t meta;
    CarLogRecord_t record;
    uint32_t primask;
    uint8_t pending;

    primask = __get_PRIMASK();
    __disable_irq();
    pending = s_car_log_begin_pending;
    if (pending != 0U) {
        meta.kind = s_car_log_begin.kind;
        meta.version = s_car_log_begin.version;
        meta.run_id = s_car_log_begin.run_id;
        meta.task = s_car_log_begin.task;
        meta.flags = s_car_log_begin.flags;
        meta.sample_count = s_car_log_begin.sample_count;
        s_car_log_begin_pending = 0U;
    }
    if (!primask) __enable_irq();
    if (pending != 0U) print_log_meta("BEGIN", &meta);

    if (car_log_pop(&record) != 0U) {
        uart_puts("CARLOG,"); uart_putu(record.run_id);
        uart_putc(','); uart_putu(record.sample_index);
        uart_putc(','); uart_putu(record.tick_ms);
        uart_putc(','); uart_putu(record.delta_left);
        uart_putc(','); uart_putu(record.delta_right);
        uart_putc(','); uart_putu(record.pwm_left);
        uart_putc(','); uart_putu(record.pwm_right);
        uart_putc(','); uart_putu(record.base_speed);
        uart_putc(','); uart_putu(record.line_mask);
        uart_puts("\r\n");
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    pending = s_car_log_end_pending;
    if (pending != 0U) {
        meta.kind = s_car_log_end.kind;
        meta.version = s_car_log_end.version;
        meta.run_id = s_car_log_end.run_id;
        meta.task = s_car_log_end.task;
        meta.flags = s_car_log_end.flags;
        meta.sample_count = s_car_log_end.sample_count;
        s_car_log_end_pending = 0U;
    }
    if (!primask) __enable_irq();
    if (pending != 0U) print_log_meta("END", &meta);
}

void TaskCtrl_Init(void)
{
    s_uart1_parser.state = 0U;
    s_debug_parser.state = 0U;
    s_task.task_id = 0U;
    s_task.state = STATE_IDLE;
    s_task.setpoint_cm = 0.0f;
    s_task.run_time_ms = 0U;
    s_task.ball_pos_cm = 0.0f;
    s_task.ball_valid = 0U;
    s_task.emergency_stop = 0U;
    s_task.queue_overflow = 0U;
    s_task.link_alive = 0U;
    s_task.pending = 0U;
    s_task.settle_ms = 0U;
    s_task.last_rx_ms = 0U;
    s_cmd_head = 0U;
    s_cmd_tail = 0U;
    s_tick_ms = 0U;
    s_last_hb_ms = 0U;
    s_last_state_ms = 0U;
    s_wheel_left = 0U;
    s_wheel_right = 0U;
    s_wheel_tick = 0U;
    s_wheel_left_previous = 0U;
    s_wheel_right_previous = 0U;
    s_wheel_tick_previous = 0U;
    s_wheel_speed_mps = 0.0f;
    s_wheel_accel_mps2 = 0.0f;
    s_wheel_accel_valid = 0U;
    s_wheel_accel_frame = 0U;
    s_wheel_accel_tick = 0U;
    s_wheel_accel_rx_frame = 0U;
    s_wheel_rx_frame = 0U;
    s_wheel_age_ms = 0xFFFFFFFFU;
    car_log_reset_stream();
    s_car_log_begin_pending = 0U;
    s_car_log_end_pending = 0U;
    s_diag = 0U;
    DL_UART_Main_enableInterrupt(UART_1_INST, DL_UART_MAIN_INTERRUPT_RX);
}

void TaskCtrl_Tick5ms(void)
{
    s_tick_ms += 5U;
    if (s_wheel_age_ms <= 0xFFFFFFFFU - 5U) s_wheel_age_ms += 5U;
    execute_queue();

    if (s_task.state == STATE_RUNNING && s_task.link_alive != 0U &&
        (s_tick_ms - s_task.last_rx_ms) > LINK_TIMEOUT_MS) {
        s_task.state = STATE_FAULT;
        s_task.settle_ms = 0U;
        s_task.pending |= TX_PENDING_STATE;
    }
    if (s_task.state == STATE_RUNNING) s_task.run_time_ms += 5U;
    if (s_task.state == STATE_RUNNING && s_task.task_id == TASK_3) {
        s_task.setpoint_cm = task3_trajectory(s_task.run_time_ms);
        check_task3_settled();
    } else if (s_task.state == STATE_RUNNING && s_task.task_id == TASK_6) {
        check_task6_settled();
    }
    if (s_tick_ms - s_last_hb_ms >= HB_INTERVAL) {
        s_task.pending |= TX_PENDING_HB;
        s_last_hb_ms = s_tick_ms;
    }
    if (s_tick_ms - s_last_state_ms >= STATE_INTERVAL) {
        s_task.pending |= TX_PENDING_STATE;
        s_last_state_ms = s_tick_ms;
    }
}

void TaskCtrl_Process(void)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t pending;
    uint8_t task_id;
    uint8_t state;
    float ball_pos;

    __disable_irq();
    pending = s_task.pending;
    s_task.pending = 0U;
    task_id = s_task.task_id;
    state = s_task.state;
    ball_pos = s_task.ball_pos_cm;
    if (!primask) __enable_irq();

    if ((pending & TX_PENDING_STATE) != 0U) send_state(task_id, state, ball_pos);
    if ((pending & TX_PENDING_HB) != 0U) {
        uint8_t data = 0U;
        send_frame(0xFFU, &data, 1U);
    }

    if (s_diag != 0U) {
        uint8_t diag = s_diag;
        s_diag = 0U;
        switch (diag) {
        case DIAG_TASK_OK:
            uart_puts("[OK] Task "); uart_putu(s_diag_task); uart_puts("\r\n");
            break;
        case DIAG_TASK_ERR: uart_puts("[ERR] Bad task\r\n"); break;
        case DIAG_START: uart_puts("[OK] Start\r\n"); break;
        case DIAG_START_T4:
            uart_puts("[OK] Start T"); uart_putu(s_diag_task);
            uart_puts(" (sp=0)\r\n");
            break;
        case DIAG_NO_TASK: uart_puts("[ERR] No task\r\n"); break;
        case DIAG_STOP: uart_puts("[OK] Stop\r\n"); break;
        case DIAG_SP:
            uart_puts("[OK] SP="); uart_putf(s_diag_setpoint, 2);
            uart_puts("\r\n");
            break;
        case DIAG_NOT_T6: uart_puts("[ERR] Not T6\r\n"); break;
        default: break;
        }
    }

    process_car_log_output();
}

void TaskCtrl_FeedByte(uint8_t byte)
{
    uint8_t type;
    uint8_t len;
    uint8_t *data;
    if (aa55_parse(&s_debug_parser, byte, &type, &len, &data) != 0U &&
        type == AA55_TYPE_TASK_CMD && len == 4U) {
        command_push(data[0], data[1], (int16_t)get_u16(&data[2]));
    }
}

void TaskCtrl_ReportBallPos(float cm)
{
    s_task.ball_pos_cm = cm;
    s_task.ball_valid = 1U;
}

void TaskCtrl_ReportBallInvalid(void)
{
    s_task.ball_valid = 0U;
    s_task.settle_ms = 0U;
}

void TaskCtrl_ReportFault(void)
{
    if (s_task.state != STATE_FAULT) {
        s_task.state = STATE_FAULT;
        s_task.settle_ms = 0U;
        s_task.pending |= TX_PENDING_STATE;
    }
}

uint8_t TaskCtrl_GetInfo(TaskInfo_t *info)
{
    if (info == 0) return 0U;
    info->task_id = s_task.task_id;
    info->state = s_task.state;
    info->setpoint_cm = s_task.setpoint_cm;
    info->run_time_ms = s_task.run_time_ms;
    return 1U;
}

uint32_t TaskCtrl_GetWheelFrame(void)
{
    return s_wheel_accel_frame;
}

uint8_t TaskCtrl_GetWheelAccel(float *accel_mps2)
{
    if (accel_mps2 == 0 || s_wheel_accel_valid == 0U) return 0U;
    if (s_wheel_age_ms > 500U) return 0U;
    *accel_mps2 = s_wheel_accel_mps2;
    return 1U;
}

uint8_t TaskCtrl_GetWheelAccelSample(WheelAccelSample_t *sample)
{
    uint32_t primask;
    uint8_t valid;

    if (sample == 0) return 0U;
    primask = __get_PRIMASK();
    __disable_irq();
    valid = (s_wheel_accel_valid != 0U && s_wheel_age_ms <= 500U) ? 1U : 0U;
    if (valid != 0U) {
        sample->accel_mps2 = s_wheel_accel_mps2;
        sample->source_tick_ms = s_wheel_accel_tick;
        sample->frame = s_wheel_accel_frame;
        sample->telemetry_frame = s_wheel_accel_rx_frame;
    }
    if (!primask) __enable_irq();
    return valid;
}

uint8_t TaskCtrl_GetWheelTelemetry(WheelTelemetry_t *telemetry)
{
    uint32_t primask;
    if (telemetry == 0 || s_wheel_rx_frame == 0U) return 0U;
    primask = __get_PRIMASK();
    __disable_irq();
    telemetry->left_counts = (uint16_t)s_wheel_left;
    telemetry->right_counts = (uint16_t)s_wheel_right;
    telemetry->source_tick_ms = s_wheel_tick;
    telemetry->frame = s_wheel_rx_frame;
    if (!primask) __enable_irq();
    return 1U;
}

uint8_t TaskCtrl_RequestHistoryDump(void)
{
    uint8_t data[4];
    if (s_task.state == STATE_RUNNING) return 0U;
    data[0] = AA55_CMD_DUMP_LOG;
    data[1] = s_task.task_id;
    data[2] = 0U;
    data[3] = 0U;
    send_frame(AA55_TYPE_TASK_CMD, data, 4U);
    return 1U;
}

void UART1_IRQHandler(void)
{
    while (!DL_UART_Main_isRXFIFOEmpty(UART_1_INST)) {
        uint8_t byte = (uint8_t)DL_UART_Main_receiveData(UART_1_INST);
        uint8_t type;
        uint8_t len;
        uint8_t *data;
        if (aa55_parse(&s_uart1_parser, byte, &type, &len, &data) != 0U) {
            handle_uart1_frame(type, len, data);
        }
    }
}
