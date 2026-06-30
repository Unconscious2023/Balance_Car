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


HEAD = 0xA5
TAIL = 0x5A

IMG_W = 320
IMG_H = 240
IMG_CENTER_X = IMG_W // 2

DEBUG_MODE = True
DISPLAY_WHILE_RUNNING = True
DISPLAY_EVERY_N = 2

BLACK_L_MAX = 55
BLACK_AB_LIMIT = 55
MIN_PIXELS = 240
MIN_AREA = 420
MAX_CANDIDATES = 3

CMD_PERIOD_MS = 50
KEY_DEBOUNCE_MS = 250

NEAR_ROI = (16, 188, 288, 26, 0.42)
MID_ROI = (16, 132, 288, 26, 0.36)
FAR_ROI = (16, 76, 288, 24, 0.16)
VERY_FAR_ROI = (16, 26, 288, 22, 0.06)
ROIS = [NEAR_ROI, MID_ROI, FAR_ROI, VERY_FAR_ROI]

PARAM_FILE = "/sd/road_sensor_params.txt"
PARAMS = {
    "BLACK_L_MAX": BLACK_L_MAX,
    "MIN_AREA": MIN_AREA,
    "CENTER_DEAD_ZONE": 5,
    "CONF_MIN_SEND": 15,
}

WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
YELLOW = (255, 220, 0)
BLUE = (0, 100, 255)
PURPLE = (255, 0, 255)


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
        return False


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
                               timeout=0, read_buf_len=128)
            self.mode = "uart2"

    def send_vision(self, error, slope, confidence):
        error = int(clamp(error, -100, 100))
        slope = int(clamp(slope, -100, 100))
        confidence = int(clamp(confidence, 0, 100))
        err_u = error & 0xFF
        slope_u = slope & 0xFF
        chk = (err_u + slope_u + confidence) & 0xFF
        frame = [HEAD, err_u, slope_u, confidence, chk, TAIL]
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


def black_threshold(params):
    l_max = int(clamp(params["BLACK_L_MAX"], 25, 95))
    return [(0, l_max, -BLACK_AB_LIMIT, BLACK_AB_LIMIT,
             -BLACK_AB_LIMIT, BLACK_AB_LIMIT)]


def find_layer_center(img, roi, thresholds, params):
    x, y, w, h, weight = roi
    blobs = img.find_blobs(thresholds,
                           roi=(x, y, w, h),
                           pixels_threshold=MIN_PIXELS,
                           area_threshold=int(params["MIN_AREA"]),
                           merge=True,
                           margin=12)
    candidates = []
    for blob in blobs:
        if blob.area() < params["MIN_AREA"]:
            continue
        if blob.pixels() < MIN_PIXELS:
            continue
        if blob.density() < 0.20:
            continue
        candidates.append(blob)
    candidates.sort(key=lambda b: b.pixels(), reverse=True)
    candidates = candidates[:MAX_CANDIDATES]
    if not candidates:
        return None, []

    total = 0
    weighted = 0
    for blob in candidates:
        p = blob.pixels()
        total += p
        weighted += blob.cx() * p
    return int(weighted / total), candidates


def detect_road(img, params):
    thresholds = black_threshold(params)
    centers = []
    all_blobs = []
    for roi in ROIS:
        cx, blobs = find_layer_center(img, roi, thresholds, params)
        centers.append(cx)
        for blob in blobs:
            all_blobs.append(blob)

    seen = 0
    for cx in centers:
        if cx is not None:
            seen += 1

    if seen == 0:
        return {
            "seen": False, "centers": centers, "blobs": all_blobs,
            "error": 0, "slope": 0, "confidence": 0,
        }

    near = centers[0]
    mid = centers[1]
    far = centers[2]
    very_far = centers[3]

    base = near
    if base is None:
        base = mid if mid is not None else (far if far is not None else very_far)
    look = far
    if look is None:
        look = very_far if very_far is not None else mid
    if look is None:
        look = base

    sum_x = 0
    sum_w = 0
    for cx, roi in zip(centers, ROIS):
        if cx is None:
            continue
        weight = roi[4]
        sum_x += cx * weight
        sum_w += weight
    target_x = int(sum_x / sum_w)

    error_px = target_x - IMG_CENTER_X
    slope_px = look - base

    continuity = 0
    last = None
    for cx in centers:
        if cx is None:
            continue
        if last is not None and abs(cx - last) < 95:
            continuity += 1
        last = cx

    confidence = seen * 18 + continuity * 12
    if centers[0] is not None:
        confidence += 12
    if centers[1] is not None:
        confidence += 10
    confidence = int(clamp(confidence, 0, 100))

    error_i8 = int(error_px * 100 / IMG_CENTER_X)
    slope_i8 = int(slope_px * 100 / IMG_CENTER_X)
    if abs(error_i8) <= params["CENTER_DEAD_ZONE"]:
        error_i8 = 0

    return {
        "seen": confidence >= params["CONF_MIN_SEND"],
        "centers": centers,
        "blobs": all_blobs,
        "target_x": target_x,
        "error": int(clamp(error_i8, -100, 100)),
        "slope": int(clamp(slope_i8, -100, 100)),
        "confidence": confidence,
    }


def touch_params(params, save_msg):
    if not ts:
        return save_msg
    try:
        status, x, y = ts.read()
    except Exception:
        return save_msg
    if status != ts.STATUS_PRESS:
        return save_msg
    if 0 <= y < 34:
        if x < 100:
            params["BLACK_L_MAX"] = clamp(params["BLACK_L_MAX"] - 1, 25, 95)
            return "BL-"
        if x > 220:
            params["BLACK_L_MAX"] = clamp(params["BLACK_L_MAX"] + 1, 25, 95)
            return "BL+"
    if 206 <= y < 240 and x < 100:
        return "SAVED" if save_params(params) else "FAIL"
    return save_msg


def draw_debug(img, road, running, fps, params, save_msg):
    colors = [GREEN, BLUE, WHITE, PURPLE]
    for roi, color in zip(ROIS, colors):
        x, y, w, h, _ = roi
        img.draw_rectangle((x, y, w, h), color=color, thickness=1)
    for blob in road["blobs"]:
        img.draw_rectangle(blob.rect(), color=YELLOW, thickness=2)
    ys = [201, 145, 88, 37]
    for cx, y, color in zip(road["centers"], ys, colors):
        if cx is not None:
            img.draw_cross(int(cx), y, color=color, thickness=2)
    img.draw_line(IMG_CENTER_X, 0, IMG_CENTER_X, IMG_H - 1,
                  color=WHITE, thickness=1)
    if road.get("target_x") is not None:
        img.draw_line(int(road["target_x"]), 0, int(road["target_x"]),
                      IMG_H - 1, color=RED, thickness=2)

    title = "RUN" if running else "STOP"
    img.draw_string(0, 0, "%s fps:%2.1f e:%d s:%d c:%d" %
                    (title, fps, road["error"], road["slope"],
                     road["confidence"]), color=GREEN, scale=1)
    if not running:
        img.draw_rectangle((0, 18, 100, 22), color=BLUE, thickness=1)
        img.draw_string(10, 25, "BL-", color=WHITE, scale=1)
        img.draw_string(116, 25, "BL:%d" % int(params["BLACK_L_MAX"]),
                        color=WHITE, scale=1)
        img.draw_rectangle((220, 18, 100, 22), color=BLUE, thickness=1)
        img.draw_string(250, 25, "BL+", color=WHITE, scale=1)
        img.draw_rectangle((0, 206, 100, 34), color=GREEN, thickness=1)
        img.draw_string(20, 218, "SAVE", color=WHITE, scale=1)
        if save_msg:
            img.draw_string(110, 218, save_msg, color=GREEN, scale=1)


def main():
    init_camera()
    key = KeyButton()
    link = Stm32Link()
    clock = time.clock()
    params = PARAMS.copy()
    load_params(params)

    running = False
    last_send = time.ticks_ms()
    display_count = 0
    save_msg = ""
    save_msg_until = 0
    last_road = {
        "seen": False, "centers": [None, None, None, None], "blobs": [],
        "error": 0, "slope": 0, "confidence": 0,
    }

    while True:
        try:
            clock.tick()
            img = sensor.snapshot()
            now = time.ticks_ms()

            if key.pressed_event():
                running = not running
                if not running:
                    link.send_vision(0, 0, 0)

            if not running:
                msg = touch_params(params, save_msg)
                if msg != save_msg:
                    save_msg = msg
                    save_msg_until = now + 900
            if save_msg and time.ticks_diff(now, save_msg_until) > 0:
                save_msg = ""

            road = detect_road(img, params)
            last_road = road
            if running and time.ticks_diff(now, last_send) >= CMD_PERIOD_MS:
                if road["seen"]:
                    link.send_vision(road["error"], road["slope"],
                                     road["confidence"])
                else:
                    link.send_vision(0, 0, 0)
                last_send = now

            show_display = DEBUG_MODE and (DISPLAY_WHILE_RUNNING or
                                           (not running))
            display_count += 1
            if not show_display:
                display_count = 0
            if show_display and display_count >= DISPLAY_EVERY_N:
                display_count = 0
                draw_debug(img, road, running, clock.fps(), params, save_msg)
                lcd.display(img)
        except Exception as err:
            try:
                link.send_vision(0, 0, 0)
                img.draw_string(0, 44, "ERR:%s" % err, color=RED, scale=1)
                lcd.display(img)
            except Exception:
                pass
            running = False
            last_road["confidence"] = 0


main()
