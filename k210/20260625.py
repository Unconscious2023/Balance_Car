import sensor
import time
import lcd

try:
    import touchscreen as ts
except ImportError:
    ts = None

try:
    from modules import ybserial, ybkey
except ImportError:
    ybserial = None
    ybkey = None


# ================= Protocol =================
# STM32 continuous control frame: A5 speed turn mode checksum 5A.
CMD_HEAD = 0xA5
CMD_TAIL = 0x5A
CTRL_MODE_STOP = 0
CTRL_MODE_RUN = 1


# ================= Camera =================
IMG_W = 320
IMG_H = 240
IMG_CENTER_X = IMG_W // 2


# ================= User tuning area =================
# Black threshold. Smaller BLACK_CONTRAST_MARGIN detects lighter black lines.
BLACK_L_FALLBACK_MAX = 55
BLACK_L_MIN_MAX = 24
BLACK_L_MAX_LIMIT = 88
BLACK_CONTRAST_MARGIN = 18
BLACK_AB_LIMIT = 50
GROUND_STATS_ROI = (0, 0, 320, 240)

# Three-layer path tracking. Each band outputs one black-line center.
# Format: (x, y, w, h, weight)
NEAR_ROIS = [
    (12, 176, 296, 24, 0.35),
    (12, 206, 296, 24, 0.45),
]
MID_ROIS = [
    (12, 116, 296, 24, 0.35),
    (12, 146, 296, 24, 0.45),
]
FAR_ROIS = [
    (12, 56, 296, 24, 0.30),
    (12, 86, 296, 24, 0.40),
]
VERY_FAR_ROIS = [
    (12, 12, 296, 20, 0.30),
    (12, 34, 296, 18, 0.40),
]

MIN_PIXELS = 120
MIN_AREA = 400
MAX_BLOB_AREA_RATIO = 0.60
MIN_BLOB_DENSITY = 0.08

TARGET_SMOOTH_NUM = 2
LANE_LOOKAHEAD_GAIN = 0.60
CURVE_SLOWDOWN_GAIN = 0.045
CURVE_DETECT_HEADING = 20
CURVE_HOLD_FRAMES = 8
CURVE_HOLD_ERROR = 80
CURVE_HOLD_SPEED = 5
CURVE_TURN_GAIN = 0.22

# Control. Only tune these first.
BASE_SPEED = 8
MIN_SPEED = 5
TURN_KP = 0.32
TURN_KD = 0.25
TURN_MAX = 60
TURN_ARC_RATIO = 4.0
TURN_DEAD_ZONE = 15
SLOWDOWN_GAIN = 0.035

# Search when line is lost.
SEARCH_SPEED = 6
SEARCH_TURN = 30
LOST_CONFIRM_FRAMES = 2

CMD_PERIOD_MS = 50
IDLE_STOP_PERIOD_MS = 100
KEY_DEBOUNCE_MS = 250
# =============== End user tuning area ===============


PARAM_FILE = "/sd/line_params.txt"
PARAMS = {
    "BASE_SPEED": BASE_SPEED,
    "MIN_SPEED": MIN_SPEED,
    "TURN_KP": TURN_KP,
    "TURN_KD": TURN_KD,
    "TURN_MAX": TURN_MAX,
    "TURN_ARC_RATIO": TURN_ARC_RATIO,
    "TURN_DEAD_ZONE": TURN_DEAD_ZONE,
    "SLOWDOWN_GAIN": SLOWDOWN_GAIN,
    "LANE_LOOKAHEAD_GAIN": LANE_LOOKAHEAD_GAIN,
    "CURVE_SLOWDOWN_GAIN": CURVE_SLOWDOWN_GAIN,
    "CURVE_DETECT_HEADING": CURVE_DETECT_HEADING,
    "CURVE_HOLD_FRAMES": CURVE_HOLD_FRAMES,
    "CURVE_HOLD_ERROR": CURVE_HOLD_ERROR,
    "CURVE_HOLD_SPEED": CURVE_HOLD_SPEED,
    "CURVE_TURN_GAIN": CURVE_TURN_GAIN,
}

BUTTONS = [
    ("SLOW", 0, 184, 52, 28),
    ("FAST", 54, 184, 52, 28),
    ("SOFT", 108, 184, 52, 28),
    ("SHARP", 162, 184, 52, 28),
    ("SAVE", 0, 214, 78, 26),
]


WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
YELLOW = (255, 220, 0)
BLUE = (0, 100, 255)


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def load_params(params):
    try:
        with open(PARAM_FILE, "r") as f:
            for line in f:
                if "=" not in line:
                    continue
                key, value = line.strip().split("=", 1)
                if key in params:
                    params[key] = float(value)
    except Exception:
        try:
            with open("line_params.txt", "r") as f:
                for line in f:
                    if "=" not in line:
                        continue
                    key, value = line.strip().split("=", 1)
                    if key in params:
                        params[key] = float(value)
        except Exception:
            pass


def save_params(params):
    text = ""
    for key in sorted(params.keys()):
        text += "%s=%s\n" % (key, params[key])

    try:
        with open(PARAM_FILE, "w") as f:
            f.write(text)
        return True
    except Exception:
        try:
            with open("line_params.txt", "w") as f:
                f.write(text)
            return True
        except Exception:
            return False


def adjust_params(params, action):
    if action == "SLOW":
        params["BASE_SPEED"] = clamp(params["BASE_SPEED"] - 1, 1, 20)
    elif action == "FAST":
        params["BASE_SPEED"] = clamp(params["BASE_SPEED"] + 1, 1, 20)
    elif action == "SOFT":
        params["TURN_KP"] = clamp(params["TURN_KP"] - 0.04, 0.10, 2.00)
        params["TURN_KD"] = clamp(params["TURN_KD"] + 0.03, 0.00, 1.00)
        params["TURN_MAX"] = clamp(params["TURN_MAX"] - 5, 20, 100)
        params["TURN_ARC_RATIO"] = clamp(params["TURN_ARC_RATIO"] - 0.3,
                                         2.0, 8.0)
        params["LANE_LOOKAHEAD_GAIN"] = clamp(
            params["LANE_LOOKAHEAD_GAIN"] - 0.05, 0.00, 1.20)
        params["CURVE_TURN_GAIN"] = clamp(params["CURVE_TURN_GAIN"] - 0.02,
                                          0.00, 1.00)
    elif action == "SHARP":
        params["TURN_KP"] = clamp(params["TURN_KP"] + 0.04, 0.10, 2.00)
        params["TURN_MAX"] = clamp(params["TURN_MAX"] + 5, 20, 100)
        params["TURN_ARC_RATIO"] = clamp(params["TURN_ARC_RATIO"] + 0.3,
                                         2.0, 8.0)
        params["LANE_LOOKAHEAD_GAIN"] = clamp(
            params["LANE_LOOKAHEAD_GAIN"] + 0.05, 0.00, 1.20)
        params["CURVE_TURN_GAIN"] = clamp(params["CURVE_TURN_GAIN"] + 0.02,
                                          0.00, 1.00)


class KeyButton:
    def __init__(self):
        self.last_pressed = False
        self.last_event_ms = 0
        if ybkey:
            self.key = ybkey()
            self.mode = "ybkey"
        else:
            from Maix import GPIO
            from fpioa_manager import fm
            from board import board_info
            fm.register(board_info.BOOT_KEY, fm.fpioa.GPIOHS0)
            self.key = GPIO(GPIO.GPIOHS0, GPIO.IN)
            self.mode = "boot"

    def pressed_event(self):
        if self.mode == "ybkey":
            pressed = bool(self.key.is_press())
        else:
            pressed = self.key.value() == 0

        now = time.ticks_ms()
        event = (pressed and not self.last_pressed and
                 time.ticks_diff(now, self.last_event_ms) > KEY_DEBOUNCE_MS)
        if event:
            self.last_event_ms = now
        self.last_pressed = pressed
        return event


class Stm32Link:
    def __init__(self):
        if ybserial:
            self.serial = ybserial()
            self.mode = "ybserial"
        else:
            from fpioa_manager import fm
            from machine import UART
            fm.register(6, fm.fpioa.UART2_RX)
            fm.register(8, fm.fpioa.UART2_TX)
            self.serial = UART(UART.UART2, 115200, 8, 0, 0,
                               timeout=0, read_buf_len=256)
            self.mode = "uart2"

    def send_control(self, speed, turn, ctrl_mode):
        speed = clamp(int(speed), -100, 100)
        turn = clamp(int(turn), -100, 100)
        ctrl_mode = ctrl_mode & 0xFF
        speed_u = speed & 0xFF
        turn_u = turn & 0xFF
        chk = (speed_u + turn_u + ctrl_mode) & 0xFF
        frame = [CMD_HEAD, speed_u, turn_u, ctrl_mode, chk, CMD_TAIL]
        try:
            if self.mode == "ybserial":
                self.serial.send_bytearray(frame)
            else:
                self.serial.write(bytes(frame))
        except Exception:
            pass


def init_camera():
    lcd.init()
    if ts:
        try:
            ts.init()
        except Exception:
            pass
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    sensor.skip_frames(time=1000)


def touch_action():
    if not ts:
        return None
    try:
        status, x, y = ts.read()
    except Exception:
        return None
    if status != ts.STATUS_PRESS:
        return None
    for name, bx, by, bw, bh in BUTTONS:
        if bx <= x < bx + bw and by <= y < by + bh:
            return name
    return None


def get_black_thresholds(img):
    try:
        stats = img.get_statistics(roi=GROUND_STATS_ROI)
        l_max = int(stats.l_mean() - BLACK_CONTRAST_MARGIN)
    except Exception:
        l_max = BLACK_L_FALLBACK_MAX

    l_max = clamp(l_max, BLACK_L_MIN_MAX, BLACK_L_MAX_LIMIT)
    return [(0, l_max, -BLACK_AB_LIMIT, BLACK_AB_LIMIT,
             -BLACK_AB_LIMIT, BLACK_AB_LIMIT)], l_max


def valid_blob(blob, roi):
    _, _, roi_w, roi_h, _ = roi
    if blob.pixels() < MIN_PIXELS:
        return False
    if blob.area() < MIN_AREA:
        return False
    if blob.area() > roi_w * roi_h * MAX_BLOB_AREA_RATIO:
        return False
    if blob.density() < MIN_BLOB_DENSITY:
        return False
    return True


def best_blob_in_roi(img, roi, thresholds):
    x, y, w, h = roi[:4]
    blobs = img.find_blobs(thresholds,
                           roi=(x, y, w, h),
                           pixels_threshold=MIN_PIXELS,
                           area_threshold=MIN_AREA,
                           merge=True,
                           margin=8)
    best = None
    best_score = 0
    for blob in blobs:
        if not valid_blob(blob, roi):
            continue
        score = blob.pixels()
        if score > best_score:
            best = blob
            best_score = score
    return best


def layer_center(img, rois, thresholds):
    weighted_x = 0
    total_weight = 0
    blobs = []

    for roi in rois:
        blob = best_blob_in_roi(img, roi, thresholds)
        if not blob:
            continue
        weight = roi[4] * blob.pixels()
        weighted_x += blob.cx() * weight
        total_weight += weight
        blobs.append(blob)

    if total_weight <= 0:
        return None, blobs
    return int(weighted_x / total_weight), blobs


def lane_point(xs):
    for x in xs:
        if x is not None:
            return x
    return None


def lane_heading(near_x, mid_x, far_x, very_far_x):
    if near_x is not None and very_far_x is not None:
        return very_far_x - near_x
    if mid_x is not None and very_far_x is not None:
        return very_far_x - mid_x
    if far_x is not None and very_far_x is not None:
        return very_far_x - far_x
    if near_x is not None and far_x is not None:
        return far_x - near_x
    if near_x is not None and mid_x is not None:
        return mid_x - near_x
    if mid_x is not None and far_x is not None:
        return far_x - mid_x
    return 0


def curve_hold_update(near_x, mid_x, far_x, very_far_x, hold_dir,
                      hold_frames, params):
    heading = lane_heading(near_x, mid_x, far_x, very_far_x)
    if abs(heading) >= params["CURVE_DETECT_HEADING"]:
        hold_dir = 1 if heading > 0 else -1
        hold_frames = int(params["CURVE_HOLD_FRAMES"])
    elif hold_frames > 0:
        hold_frames -= 1
    else:
        hold_dir = 0
    return hold_dir, hold_frames, heading


def select_target_x(near_x, mid_x, far_x, very_far_x, last_target_x,
                    params):
    base_x = lane_point([near_x, mid_x, far_x, very_far_x])
    if base_x is None:
        return None

    heading = lane_heading(near_x, mid_x, far_x, very_far_x)
    target_x = int(base_x + heading * params["LANE_LOOKAHEAD_GAIN"])
    target_x = clamp(target_x, 0, IMG_W - 1)

    if last_target_x is None:
        return target_x
    return int((last_target_x * (TARGET_SMOOTH_NUM - 1) + target_x) /
               TARGET_SMOOTH_NUM)


def control_from_target(target_x, last_error, params,
                        near_x, mid_x, far_x, very_far_x):
    error = target_x - IMG_CENTER_X
    delta = error - last_error
    heading = lane_heading(near_x, mid_x, far_x, very_far_x)
    curve_turn = int(heading * params["CURVE_TURN_GAIN"])

    if abs(error) <= params["TURN_DEAD_ZONE"]:
        turn = 0
    else:
        turn = int(error * params["TURN_KP"] + delta * params["TURN_KD"])
    turn += curve_turn
    turn = clamp(turn, -params["TURN_MAX"], params["TURN_MAX"])

    speed = params["BASE_SPEED"] - int(abs(error) * params["SLOWDOWN_GAIN"] +
                                       abs(heading) *
                                       params["CURVE_SLOWDOWN_GAIN"])
    speed = clamp(speed, params["MIN_SPEED"], params["BASE_SPEED"])

    # Keep turning as an arc. If turn is too large compared with speed,
    # one wheel is easily canceled by the steering term and stops rotating.
    arc_turn_max = int(max(1, abs(speed)) * params["TURN_ARC_RATIO"])
    turn = clamp(turn, -arc_turn_max, arc_turn_max)
    return speed, turn, error


def control_from_curve_hold(hold_dir, last_error, params):
    target_x = IMG_CENTER_X + hold_dir * int(params["CURVE_HOLD_ERROR"])
    speed, turn, error = control_from_target(target_x, last_error, params,
                                             None, None, None, None)
    speed = min(speed, int(params["CURVE_HOLD_SPEED"]))
    return speed, turn, error


def draw_tuning_buttons(img, params, save_msg):
    for name, x, y, w, h in BUTTONS:
        color = GREEN if name == "SAVE" else BLUE
        img.draw_rectangle((x, y, w, h), color=color, thickness=1)
        img.draw_string(x + 4, y + 8, name, color=WHITE, scale=1)

    img.draw_string(128, 214, "V:%d KP:%1.2f KD:%1.2f" %
                    (int(params["BASE_SPEED"]), params["TURN_KP"],
                     params["TURN_KD"]), color=WHITE, scale=1)
    img.draw_string(128, 226, "LA:%1.2f ARC:%1.1f DZ:%d %s" %
                    (params["LANE_LOOKAHEAD_GAIN"],
                     params["TURN_ARC_RATIO"],
                     int(params["TURN_DEAD_ZONE"]), save_msg),
                    color=WHITE, scale=1)


def draw_ui(img, running, near_x, mid_x, far_x, very_far_x, target_x,
            blobs, speed, turn, fps, l_max, lost_frames, uart_mode,
            params, save_msg, curve_hold_frames, heading):
    for x, y, w, h, _ in NEAR_ROIS:
        img.draw_rectangle((x, y, w, h), color=GREEN, thickness=1)
    for x, y, w, h, _ in MID_ROIS:
        img.draw_rectangle((x, y, w, h), color=BLUE, thickness=1)
    for x, y, w, h, _ in FAR_ROIS:
        img.draw_rectangle((x, y, w, h), color=WHITE, thickness=1)
    for x, y, w, h, _ in VERY_FAR_ROIS:
        img.draw_rectangle((x, y, w, h), color=(255, 0, 255), thickness=1)

    img.draw_line(IMG_CENTER_X, 0, IMG_CENTER_X, IMG_H - 1,
                  color=WHITE, thickness=1)

    for blob in blobs:
        img.draw_rectangle(blob.rect(), color=YELLOW, thickness=2)
        img.draw_cross(blob.cx(), blob.cy(), color=YELLOW)

    if near_x is not None:
        img.draw_cross(near_x, 222, color=GREEN, thickness=2)
    if mid_x is not None:
        img.draw_cross(mid_x, 166, color=BLUE, thickness=2)
    if far_x is not None:
        img.draw_cross(far_x, 92, color=WHITE, thickness=2)
    if very_far_x is not None:
        img.draw_cross(very_far_x, 36, color=(255, 0, 255), thickness=2)
    if near_x is not None and mid_x is not None:
        img.draw_line(near_x, 222, mid_x, 166, color=YELLOW, thickness=2)
    if mid_x is not None and far_x is not None:
        img.draw_line(mid_x, 166, far_x, 92, color=YELLOW, thickness=2)
    if far_x is not None and very_far_x is not None:
        img.draw_line(far_x, 92, very_far_x, 36, color=(255, 0, 255), thickness=2)
    elif near_x is not None and far_x is not None:
        img.draw_line(near_x, 222, far_x, 92, color=YELLOW, thickness=2)
    if target_x is not None:
        color = GREEN if abs(target_x - IMG_CENTER_X) <= params["TURN_DEAD_ZONE"] else RED
        img.draw_line(target_x, 0, target_x, IMG_H - 1, color=color, thickness=2)

    state = "RUN" if running else "STOP"
    if running and target_x is None:
        state = "HOLD" if curve_hold_frames > 0 else "SEARCH"
    err = 0 if target_x is None else target_x - IMG_CENTER_X
    img.draw_string(0, 0, "%s fps:%2.1f spd:%d turn:%d" %
                    (state, fps, speed, turn), color=GREEN if running else WHITE,
                    scale=2)
    img.draw_string(0, 22, "err:%d h:%d hold:%d L:%02d" %
                    (err, int(heading), curve_hold_frames, l_max),
                    color=WHITE, scale=1)
    if not running:
        draw_tuning_buttons(img, params, save_msg)


def main():
    init_camera()
    key = KeyButton()
    link = Stm32Link()
    clock = time.clock()
    params = PARAMS.copy()
    load_params(params)

    running = False
    last_send = time.ticks_ms()
    last_target_x = None
    last_error = 0
    last_turn_dir = 1
    curve_hold_dir = 0
    curve_hold_frames = 0
    lost_frames = 0
    speed = 0
    turn = 0
    save_msg = ""
    save_msg_until = 0

    while True:
        clock.tick()
        img = sensor.snapshot()
        now = time.ticks_ms()

        if key.pressed_event():
            running = not running
            last_target_x = None
            last_error = 0
            lost_frames = 0
            curve_hold_dir = 0
            curve_hold_frames = 0
            speed = 0
            turn = 0
            last_send = now
            save_msg = ""

        if not running:
            action = touch_action()
            if action:
                if action == "SAVE":
                    save_msg = "SAVED" if save_params(params) else "FAIL"
                    save_msg_until = now + 1500
                else:
                    adjust_params(params, action)
                    save_msg = action
                    save_msg_until = now + 800

        if save_msg and time.ticks_diff(now, save_msg_until) > 0:
            save_msg = ""

        thresholds, l_max = get_black_thresholds(img)
        near_x, near_blobs = layer_center(img, NEAR_ROIS, thresholds)
        mid_x, mid_blobs = layer_center(img, MID_ROIS, thresholds)
        far_x, far_blobs = layer_center(img, FAR_ROIS, thresholds)
        very_far_x, very_far_blobs = layer_center(img, VERY_FAR_ROIS,
                                                  thresholds)
        blobs = near_blobs + mid_blobs + far_blobs + very_far_blobs
        curve_hold_dir, curve_hold_frames, heading = curve_hold_update(
            near_x, mid_x, far_x, very_far_x, curve_hold_dir,
            curve_hold_frames, params)

        target_x = select_target_x(near_x, mid_x, far_x, very_far_x,
                                   last_target_x, params)
        if target_x is None:
            lost_frames += 1
        else:
            lost_frames = 0
            last_target_x = target_x

        if running:
            if target_x is not None:
                speed, turn, last_error = control_from_target(
                    target_x, last_error, params,
                    near_x, mid_x, far_x, very_far_x)
                if curve_hold_frames > 0 and curve_hold_dir != 0:
                    speed = min(speed, int(params["CURVE_HOLD_SPEED"]))
                if turn > params["TURN_DEAD_ZONE"]:
                    last_turn_dir = 1
                elif turn < -params["TURN_DEAD_ZONE"]:
                    last_turn_dir = -1
            elif curve_hold_frames > 0 and curve_hold_dir != 0:
                speed, turn, last_error = control_from_curve_hold(
                    curve_hold_dir, last_error, params)
                last_turn_dir = curve_hold_dir
            elif lost_frames >= LOST_CONFIRM_FRAMES:
                speed = SEARCH_SPEED
                turn = SEARCH_TURN * last_turn_dir
            else:
                speed = MIN_SPEED
                turn = 0
        else:
            speed = 0
            turn = 0

        send_period = CMD_PERIOD_MS if running else IDLE_STOP_PERIOD_MS
        if time.ticks_diff(now, last_send) >= send_period:
            if running:
                link.send_control(speed, turn, CTRL_MODE_RUN)
            else:
                link.send_control(0, 0, CTRL_MODE_STOP)
            last_send = now

        draw_ui(img, running, near_x, mid_x, far_x, very_far_x, target_x,
                blobs, speed, turn, clock.fps(), l_max, lost_frames,
                link.mode, params, save_msg, curve_hold_frames, heading)
        lcd.display(img)


main()
