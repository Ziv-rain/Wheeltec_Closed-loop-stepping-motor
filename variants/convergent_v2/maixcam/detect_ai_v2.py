"""
MaixCAM ball detector for convergent_v2.

The first four payload bytes are compatible with the existing 0x30 packet:
    int16 position_centi_cm, uint8 confidence, uint8 status

The four formerly reserved bytes now contain:
    uint16 frame_seq, uint16 capture_to_send_ms

An old receiver can still ignore the last four bytes. The V2 receiver uses
them to compensate camera/inference delay instead of controlling an old frame.
"""

from maix import app, camera, display, image, nn, pinmap, time, uart
import struct


CAM_W, CAM_H = 640, 480
MODEL = "/root/models/maixhub/313010/model_313010.mud"
CONF_TH = 0.50
IOU_TH = 0.45

# Keep the verified physical calibration from the existing detector.
PX_CENTER = 295
SCALE_CM_PER_PX = 0.05747

UART_PORT = "/dev/ttyS0"
UART_BAUD = 115200
SEND_PERIOD_MS = 50          # 20 Hz control measurement stream
LOST_CONFIRM_FRAMES = 5
LOST_SEND_PERIOD_MS = 150

# Disable the hidden one-frame double-buffer pipeline for lower latency.
# Throughput only needs to remain above the 20 Hz UART output rate.
DUAL_BUFFER = False

SYNC0, SYNC1 = 0xAA, 0x55
TYPE_BALL = 0x30
TYPE_HEARTBEAT = 0xFF
BALL_LOST, BALL_DETECTED, BALL_TRACKING = 0, 1, 2


def crc16_ibm(data: bytes) -> int:
    crc = 0x0000
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def build_frame(packet_type: int, payload: bytes) -> bytes:
    header = bytes([packet_type, len(payload)])
    crc = crc16_ibm(header + payload)
    return (bytes([SYNC0, SYNC1]) + header + payload +
            bytes([crc & 0xFF, (crc >> 8) & 0xFF]))


def pack_ball_v2(position_cm: float, confidence: int, status: int,
                 frame_seq: int, processing_ms: int) -> bytes:
    position_centi_cm = max(-32768, min(32767, int(position_cm * 100)))
    payload = struct.pack(
        "<hBBHH",
        position_centi_cm,
        confidence & 0xFF,
        status & 0xFF,
        frame_seq & 0xFFFF,
        processing_ms & 0xFFFF,
    )
    return build_frame(TYPE_BALL, payload)


def pack_heartbeat(frame_count: int) -> bytes:
    return build_frame(TYPE_HEARTBEAT, bytes([frame_count & 0xFF]))


def pixel_to_cm(pixel_x: int) -> float:
    return (pixel_x - PX_CENTER) * SCALE_CM_PER_PX


def elapsed_ms(now: int, before: int) -> int:
    # MaixPy on this target does not provide ticks_diff(). A negative value
    # is only possible at the long ticks_ms wrap, where zero is safest.
    delta = now - before
    return delta if delta >= 0 else 0


def next_sequence(sequence: int) -> int:
    sequence = (sequence + 1) & 0xFFFF
    return sequence if sequence != 0 else 1


def main():
    print("=" * 48)
    print("  convergent_v2 MaixCAM detector + latency packet")
    print("=" * 48)

    detector = nn.YOLOv5(MODEL, dual_buff=DUAL_BUFFER)
    model_w = detector.input_width()
    model_h = detector.input_height()
    cam = camera.Camera(CAM_W, CAM_H)
    screen = display.Display()

    pinmap.set_pin_function("A19", "UART1_TX")
    pinmap.set_pin_function("A18", "UART1_RX")
    serial = uart.UART(UART_PORT, UART_BAUD)

    last_send_ms = 0
    last_lost_send_ms = 0
    last_heartbeat_ms = 0
    frame_seq = 0
    transmitted_frames = 0
    lost_count = 0
    last_position_cm = 0.0
    fps_start_ms = time.ticks_ms()
    fps_frames = 0

    print("[MODEL] {}x{} dual_buffer={}".format(
        model_w, model_h, DUAL_BUFFER))
    print("[UART] {} @ {} baud, output={} Hz".format(
        UART_PORT, UART_BAUD, 1000 // SEND_PERIOD_MS))

    while not app.need_exit():
        capture_start_ms = time.ticks_ms()
        frame = cam.read()
        if frame is None:
            continue

        resized = frame.resize(model_w, model_h)
        objects = detector.detect(resized, conf_th=CONF_TH, iou_th=IOU_TH)
        now_ms = time.ticks_ms()
        processing_ms = min(65535, elapsed_ms(now_ms, capture_start_ms))

        scale_x = CAM_W / model_w
        scale_y = CAM_H / model_h
        found = False
        best_score = 0.0
        best_pixel_x = 0
        position_cm = 0.0

        for obj in objects:
            if obj.score <= best_score:
                continue
            box_x = int(obj.x * scale_x)
            box_y = int(obj.y * scale_y)
            box_w = int(obj.w * scale_x)
            box_h = int(obj.h * scale_y)
            best_pixel_x = box_x + box_w // 2
            position_cm = pixel_to_cm(best_pixel_x)
            best_score = obj.score
            found = True

            frame.draw_rect(box_x, box_y, box_w, box_h,
                            image.COLOR_RED, 2)
            frame.draw_cross(best_pixel_x, box_y + box_h // 2,
                             image.COLOR_RED, 4, 1)

        if found:
            lost_count = 0
            last_position_cm = position_cm
            if elapsed_ms(now_ms, last_send_ms) >= SEND_PERIOD_MS:
                frame_seq = next_sequence(frame_seq)
                status = (BALL_TRACKING if best_score > 0.80
                          else BALL_DETECTED)
                packet = pack_ball_v2(
                    position_cm,
                    int(best_score * 100),
                    status,
                    frame_seq,
                    processing_ms,
                )
                serial.write(packet)
                last_send_ms = now_ms
                transmitted_frames += 1
        else:
            lost_count += 1
            if (lost_count >= LOST_CONFIRM_FRAMES and
                    elapsed_ms(now_ms, last_lost_send_ms) >=
                    LOST_SEND_PERIOD_MS):
                frame_seq = next_sequence(frame_seq)
                serial.write(pack_ball_v2(
                    0.0, 0, BALL_LOST, frame_seq, processing_ms))
                last_lost_send_ms = now_ms

        if elapsed_ms(now_ms, last_heartbeat_ms) >= 1000:
            serial.write(pack_heartbeat(transmitted_frames))
            last_heartbeat_ms = now_ms

        state_text = ("BALL {:.2f}cm".format(last_position_cm)
                      if found else "LOST {}".format(lost_count))
        state_color = image.COLOR_GREEN if found else image.COLOR_RED
        frame.draw_string(5, 5, state_text, state_color, 1.4)
        frame.draw_string(
            5, 24,
            "seq:{} proc:{}ms tx:{}".format(
                frame_seq, processing_ms, transmitted_frames),
            image.COLOR_WHITE, 1.0,
        )
        screen.show(frame)

        fps_frames += 1
        if elapsed_ms(now_ms, fps_start_ms) >= 3000:
            fps = fps_frames * 1000 // max(1, elapsed_ms(now_ms, fps_start_ms))
            print("[{}fps] x={:.2f}cm proc={}ms seq={} tx={}".format(
                fps, last_position_cm, processing_ms,
                frame_seq, transmitted_frames))
            fps_frames = 0
            fps_start_ms = now_ms

        time.sleep_ms(5)

    print("[INFO] convergent_v2 detector stopped")


if __name__ == "__main__":
    main()
