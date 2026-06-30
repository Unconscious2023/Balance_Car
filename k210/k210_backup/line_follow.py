import sensor
import time
import lcd

try:
    from modules import ybserial, ybkey
except ImportError:
    ybserial = None
    ybkey = None


# ================= Protocol =================
CMD_HEAD  = 0xA5
CMD_TAIL  = 0x5A
DIR_LEFT  = 0x01
DIR_RIGHT = 0x02


# ================= Camera =================
IMG_W = 320
IMG_H = 240
IMG_CENTER_X = IMG_W // 2


# ================= Tuning =================
# Black threshold
BLACK_L_FALLBACK_MAX = 55
BLACK_L_MIN_MAX     = 24
BLACK_L_MAX_LIMIT   = 88
BLACK_CONTRAST_MARGIN = 18
BLACK_AB_LIMIT      = 50
GROUND_STATS_ROI    = (0, 0, 320, 240)

# 7-band horizontal ROIs
LINE_ROIS = [
    (20,  24, 280, 18, 0.08),
    (18,  56, 284, 18, 0.10),
    (16,  88, 288, 20, 0.13),
    (14, 120, 292, 20, 0.17),
    (12, 152, 296, 22, 0.22),
    (10, 184, 300, 22, 0.30),
    (8,  214, 304, 22, 0.38),
]
SIDE_ROIS = [
    (0,   70, 96, 164, DIR_LEFT),
    (224, 70, 96, 164, DIR_RIGHT),
]

MIN_PIXELS           = 80
MIN_AREA             = 100
MAX_BLOB_AREA_RATIO  = 0.55
MIN_BLOB_DENSITY     = 0.10

LINE_SMOOTH_NUM = 3
LINE_SEEN_CONFIRM = 2
LINE_LOST_CONFIRM = 2
SIDE_HINT_MIN  = 160
CROSS_SIDE_MIN = 220
SIDE_RATIO     = 1.35

# Curve hold
CURVE_DETECT_HEADING = 8
CURVE_HOLD_FRAMES    = 8
CURVE_HOLD_ERROR     = 80
CURVE_HOLD_SPEED     = 5

# Control
BASE_SPEED      = 8
MIN_SPEED       = 5
TURN_MAX_HDG    = 5
TURN_ARC_RATIO  = 4.0
TURN_DEAD_ZONE  = 12
SLOWDOWN_GAIN   = 0.035

# Search
SEARCH_SPEED        = 6
SEARCH_TURN         = 5
LOST_CONFIRM_FRAMES = 2

CMD_PERIOD_MS       = 50
IDLE_STOP_PERIOD_MS = 100
KEY_DEBOUNCE_MS     = 250

WHITE  = (255, 255, 255)
GREEN  = (0, 255, 0)
RED    = (255, 0, 0)
YELLOW = (255, 220, 0)
BLUE   = (0, 100, 255)


def clamp(value, low, high):
    if value < low: return low
    if value > high: return high
    return value


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

    def send_control(self, speed, turn):
        speed = clamp(int(speed), 0, 100)
        turn  = clamp(int(turn), -100, 100)
        speed_u = speed & 0xFF
        turn_u  = turn & 0xFF
        chk = (speed_u + turn_u + 1) & 0xFF
        frame = [CMD_HEAD, speed_u, turn_u, 1, chk, CMD_TAIL]
        try:
            if self.mode == "ybserial":
                self.serial.send_bytearray(frame)
            else:
                self.serial.write(bytes(frame))
        except Exception:
            pass


def init_camera():
    lcd.init()
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    sensor.skip_frames(time=1000)


# ================= Detection =================

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
    if blob.pixels() < MIN_PIXELS:       return False
    if blob.area()   < MIN_AREA:         return False
    if blob.area()   > roi_w * roi_h * MAX_BLOB_AREA_RATIO: return False
    if blob.density() < MIN_BLOB_DENSITY: return False
    return True


def best_blob_in_roi(img, roi, thresholds):
    x, y, w, h = roi[:4]
    blobs = img.find_blobs(thresholds, roi=(x, y, w, h),
                           pixels_threshold=MIN_PIXELS,
                           area_threshold=MIN_AREA,
                           merge=True, margin=8)
    best = None
    best_score = 0
    for blob in blobs:
        if not valid_blob(blob, roi): continue
        score = blob.pixels()
        if score > best_score:
            best = blob
            best_score = score
    return best


def black_pixels_in_roi(img, roi, thresholds):
    x, y, w, h = roi[:4]
    blobs = img.find_blobs(thresholds, roi=(x, y, w, h),
                           pixels_threshold=MIN_PIXELS,
                           area_threshold=MIN_AREA,
                           merge=True, margin=8)
    pixels = 0; found = []
    for blob in blobs:
        pixels += blob.pixels()
        found.append(blob)
    return pixels, found


def find_line(img, thresholds):
    w_x = 0; w_t = 0; blobs = []
    for roi in LINE_ROIS:
        blob = best_blob_in_roi(img, roi, thresholds)
        if not blob: continue
        w = roi[4] * blob.pixels()
        w_x += blob.cx() * w
        w_t += w
        blobs.append(blob)
    if w_t <= 0: return None, blobs
    return int(w_x / w_t), blobs


def find_corner_hint(img, thresholds):
    lp, lb = 0, []
    rp, rb = 0, []
    for x, y, w, h, _ in [SIDE_ROIS[0]]:
        px, bx = black_pixels_in_roi(img, (x, y, w, h, 0), thresholds)
        lp += px; lb += bx
    for x, y, w, h, _ in [SIDE_ROIS[1]]:
        px, bx = black_pixels_in_roi(img, (x, y, w, h, 0), thresholds)
        rp += px; rb += bx
    cross = (lp >= CROSS_SIDE_MIN and rp >= CROSS_SIDE_MIN)
    if lp < SIDE_HINT_MIN and rp < SIDE_HINT_MIN: return 0, lb + rb, False
    if cross: return DIR_RIGHT, lb + rb, True
    if lp > rp * SIDE_RATIO: return DIR_LEFT, lb + rb, False
    if rp > lp * SIDE_RATIO: return DIR_RIGHT, lb + rb, False
    return 0, lb + rb, False


def lane_heading_angle(blobs):
    """二次拟合→近端切线方向 ° (x=a*y²+b*y+c, 切向=2a*222+b)"""
    pts = [(b.cx(), b.cy()) for b in blobs]
    if len(pts) < 2: return 0

    n = len(pts)
    S0=n; S1=0; S2=0; S3=0; S4=0; Tx=0; Txy=0; Txy2=0
    for x, y in pts:
        y1=y; y2=y*y; y3=y2*y; y4=y3*y
        S1+=y1; S2+=y2; S3+=y3; S4+=y4
        Tx+=x; Txy+=x*y1; Txy2+=x*y2

    det = S4*(S2*S0-S1*S1) - S3*(S3*S0-S1*S2) + S2*(S3*S1-S2*S2)
    if abs(det) < 1e-6: return 0
    a = (Txy2*(S2*S0-S1*S1) - S3*(Txy*S0-S1*Tx) + S2*(Txy*S1-S2*Tx)) / det
    b = (S4*(Txy*S0-S1*Tx) - Txy2*(S3*S0-S1*S2) + S2*(S3*Tx-Txy*S2)) / det

    tangent = 2.0*a*222.0 + b
    x = tangent * 0.25
    x2 = x * x
    atan_x = x * (1.0 - x2/3.0 + x2*x2/5.0)
    return atan_x * 57.2958


def control_from_target(line_x, blobs):
    error   = line_x - IMG_CENTER_X if line_x is not None else 0
    hdg_deg = lane_heading_angle(blobs)

    turn_hdg = hdg_deg + error * 0.08
    turn_hdg = clamp(turn_hdg, -TURN_MAX_HDG, TURN_MAX_HDG)
    if abs(error) <= TURN_DEAD_ZONE:
        turn_hdg = 0

    speed = BASE_SPEED - int(abs(error) * SLOWDOWN_GAIN)
    speed = clamp(speed, MIN_SPEED, BASE_SPEED)
    arc_max = int(max(1, abs(speed)) * TURN_ARC_RATIO)
    turn_hdg = clamp(turn_hdg, -arc_max, arc_max)

    return speed, turn_hdg


def curve_hold_update(blobs, hold_dir, hold_frames):
    heading = lane_heading_angle(blobs)
    if abs(heading) >= CURVE_DETECT_HEADING:
        hold_dir    = 1 if heading > 0 else -1
        hold_frames = CURVE_HOLD_FRAMES
    elif hold_frames > 0:
        hold_frames -= 1
    else:
        hold_dir = 0
    return hold_dir, hold_frames, heading


# ================= UI =================

def draw_ui(img, running, line_x, blobs, side_blobs, speed, turn,
            fps, l_max, lost_frames, hold_frames, heading):
    for x, y, w, h, _ in LINE_ROIS:
        img.draw_rectangle((x, y, w, h), color=BLUE, thickness=1)
    for x, y, w, h, _ in SIDE_ROIS:
        img.draw_rectangle((x, y, w, h), color=WHITE, thickness=1)
    img.draw_line(IMG_CENTER_X, 0, IMG_CENTER_X, IMG_H-1, color=WHITE, thickness=1)
    for blob in blobs:
        img.draw_rectangle(blob.rect(), color=YELLOW, thickness=2)
        img.draw_cross(blob.cx(), blob.cy(), color=YELLOW)
    for blob in side_blobs:
        img.draw_rectangle(blob.rect(), color=BLUE, thickness=1)
    if line_x is not None:
        color = GREEN if abs(line_x-IMG_CENTER_X) <= TURN_DEAD_ZONE else RED
        img.draw_line(line_x, 150, line_x, IMG_H-1, color=color, thickness=2)
        img.draw_cross(line_x, 205, color=color, thickness=2)
    state = "RUN" if running else "STOP"
    if running and line_x is None:
        state = "HOLD" if hold_frames > 0 else "SEARCH"
    err = 0 if line_x is None else line_x - IMG_CENTER_X
    img.draw_string(0, 0, "%s fps:%2.1f S:%d T:%d" %
                    (state, fps, speed, turn), color=GREEN, scale=2)
    img.draw_string(0, 22, "err:%d h:%d L:%02d" %
                    (err, int(heading), l_max), color=WHITE, scale=1)


# ================= Main =================

def main():
    init_camera()
    key  = KeyButton()
    link = Stm32Link()
    clock = time.clock()

    running        = False
    speed = 0
    turn  = 0
    last_send      = time.ticks_ms()
    last_seen_ms   = time.ticks_ms()
    last_seen_side = DIR_LEFT
    search_dir     = DIR_LEFT
    stable_line_x  = None
    seen_frames    = 0
    lost_frames    = 0
    hold_dir       = 0
    hold_frames    = 0
    last_turn_dir  = 1

    while True:
        clock.tick()
        img = sensor.snapshot()
        now = time.ticks_ms()

        if key.pressed_event():
            running = not running
            stable_line_x = None
            seen_frames   = 0
            lost_frames   = 0
            hold_dir      = 0
            hold_frames   = 0
            speed = 0
            turn  = 0
            last_send = now

        thresholds, l_max = get_black_thresholds(img)
        raw_x, blobs = find_line(img, thresholds)
        corner_hint, side_blobs, crossroad = find_corner_hint(img, thresholds)
        hold_dir, hold_frames, heading = curve_hold_update(
            blobs, hold_dir, hold_frames)

        if raw_x is not None:
            seen_frames += 1
            lost_frames = 0
            if stable_line_x is None:
                stable_line_x = raw_x
            else:
                stable_line_x = (stable_line_x*(LINE_SMOOTH_NUM-1) + raw_x) // LINE_SMOOTH_NUM
        else:
            lost_frames += 1
            seen_frames = 0
            if lost_frames >= LINE_LOST_CONFIRM:
                stable_line_x = None

        line_seen  = (raw_x is not None)
        line_ready = (stable_line_x is not None and seen_frames >= LINE_SEEN_CONFIRM)
        searching  = False

        if stable_line_x is not None:
            if stable_line_x < IMG_CENTER_X - TURN_DEAD_ZONE:
                last_seen_side = DIR_LEFT
            elif stable_line_x > IMG_CENTER_X + TURN_DEAD_ZONE:
                last_seen_side = DIR_RIGHT

        if running:
            if line_ready:
                speed, turn = control_from_target(stable_line_x, blobs)
                if hold_frames > 0 and hold_dir != 0:
                    speed = min(speed, CURVE_HOLD_SPEED)
                if turn > 1: last_turn_dir = 1
                elif turn < -1: last_turn_dir = -1
                last_seen_ms = now
            elif line_seen:
                speed = MIN_SPEED
                turn  = 0
            elif corner_hint:
                speed = MIN_SPEED
                turn  = 30 if corner_hint == DIR_RIGHT else -30
                last_turn_dir = 1 if corner_hint == DIR_RIGHT else -1
                last_seen_ms = now
            elif hold_frames > 0 and hold_dir != 0:
                speed = CURVE_HOLD_SPEED
                turn  = 30 if hold_dir == 1 else -30
            elif time.ticks_diff(now, last_seen_ms) <= 420:
                searching = True
                speed = SEARCH_SPEED
                turn  = SEARCH_TURN * last_turn_dir
            elif lost_frames >= LOST_CONFIRM_FRAMES:
                searching = True
                speed = SEARCH_SPEED
                turn  = SEARCH_TURN * last_turn_dir
            else:
                speed = MIN_SPEED
                turn  = 0
        else:
            speed = 0
            turn  = 0

        send_period = CMD_PERIOD_MS if running else IDLE_STOP_PERIOD_MS
        if time.ticks_diff(now, last_send) >= send_period:
            link.send_control(speed, turn)
            last_send = now

        lost_ms = time.ticks_diff(now, last_seen_ms)
        draw_ui(img, running, stable_line_x, blobs, side_blobs,
                speed, turn, clock.fps(), l_max, lost_frames,
                hold_frames, heading)
        lcd.display(img)


main()
