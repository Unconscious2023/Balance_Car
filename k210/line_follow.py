import sensor
import time
import lcd

try:
    from modules import ybserial, ybkey
except ImportError:
    ybserial = None
    ybkey = None


# 帧定界符
CMD_HEAD = 0xA5
CMD_TAIL = 0x5A

# 方向标记 (仅作内部枚举, 不拼帧)
DIR_LEFT  = 0x01
DIR_RIGHT = 0x02


IMG_W = 320
IMG_H = 240
IMG_CENTER_X = IMG_W // 2

# ========== 黑线检测参数 ==========
BLACK_L_FALLBACK_MAX = 55
BLACK_L_MIN_MAX     = 24
BLACK_L_MAX_LIMIT   = 88
BLACK_CONTRAST_MARGIN = 18
BLACK_AB_LIMIT      = 50
GROUND_STATS_ROI    = (0, 0, 320, 240)

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
    (0,   70, 96, 164),
    (224, 70, 96, 164),
]
CORNER_HINT_MIN_PIXELS  = 160
CROSS_SIDE_MIN_PIXELS   = 220
SIDE_HINT_RATIO         = 1.35
CORNER_HINT_CONFIRM_FRAMES = 3

MIN_PIXELS           = 120
MIN_AREA             = 500
MAX_BLOB_AREA_RATIO  = 0.55
MIN_BLOB_DENSITY     = 0.10

# ========== 转向参数 ==========
TURN_ON_DEAD_ZONE_X  = 30
TURN_OFF_DEAD_ZONE_X = 20
FIT_LOOKAHEAD_Y      = 100
FIT_NEAR_Y           = 222
STEER_NEAR_GAIN      = 70
STEER_LOOKAHEAD_GAIN = 45
STEER_HEADING_GAIN   = 150
STEER_ERROR_DIV      = 100
HEADING_TURN_ON_X    = 24
NEAR_POSITION_PRIORITY_X = 36
LINE_SMOOTH_NUM      = 3
LINE_SEEN_CONFIRM_FRAMES = 2
LINE_LOST_CONFIRM_FRAMES = 2

# ========== 搜索 ==========
SEARCH_DEFAULT_TURN  = DIR_LEFT
LOST_KEEP_MS         = 420

# ========== 速度 ==========
START_SPEED_STEPS    = 8
START_SPEED_MAX      = 18
STRAIGHT_SPEED       = 18
CURVE_SPEED          = 10
SEARCH_SPEED         = 2
MIN_LINE_SPEED       = 5
CMD_PERIOD_MS        = 55
KEY_DEBOUNCE_MS      = 250

WHITE  = (255, 255, 255)
GREEN  = (0, 255, 0)
RED    = (255, 0, 0)
YELLOW = (255, 220, 0)
BLUE   = (0, 100, 255)


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

    def send_control(self, speed, turn_s8, direction=1):
        turn_u = turn_s8 & 0xFF
        chk = (speed + turn_u + direction) & 0xFF
        frame = [CMD_HEAD, speed & 0xFF, turn_u, direction, chk, CMD_TAIL]
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


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


# ========== 视觉检测 ==========

def get_black_thresholds(img):
    l_max = BLACK_L_FALLBACK_MAX
    try:
        stats = img.get_statistics(roi=GROUND_STATS_ROI)
        l_max = int(stats.l_mean() - BLACK_CONTRAST_MARGIN)
    except Exception:
        pass
    l_max = clamp(l_max, BLACK_L_MIN_MAX, BLACK_L_MAX_LIMIT)
    return [(0, l_max, -BLACK_AB_LIMIT, BLACK_AB_LIMIT,
             -BLACK_AB_LIMIT, BLACK_AB_LIMIT)], l_max


def best_blob_in_roi(img, roi, thresholds):
    x, y, w, h = roi[:4]
    blobs = img.find_blobs(thresholds, roi=(x, y, w, h),
                           pixels_threshold=MIN_PIXELS,
                           area_threshold=MIN_AREA,
                           merge=True, margin=8)
    best = None
    best_score = 0
    for blob in blobs:
        if blob.pixels() < MIN_PIXELS or blob.area() < MIN_AREA:
            continue
        if blob.area() > w * h * MAX_BLOB_AREA_RATIO:
            continue
        if blob.density() < MIN_BLOB_DENSITY:
            continue
        if blob.pixels() > best_score:
            best = blob
            best_score = blob.pixels()
    return best


def black_pixels_in_roi(img, roi, thresholds):
    x, y, w, h = roi[:4]
    blobs = img.find_blobs(thresholds, roi=(x, y, w, h),
                           pixels_threshold=MIN_PIXELS,
                           area_threshold=MIN_AREA,
                           merge=True, margin=8)
    pixels = 0
    found = []
    for blob in blobs:
        pixels += blob.pixels()
        found.append(blob)
    return pixels, found


def fit_x_at_y(points, target_y):
    n = len(points)
    if n <= 0: return None
    if n == 1: return points[0][0]
    sy = sx = syy = syx = 0
    for x, y, _ in points:
        sy += y; sx += x; syy += y * y; syx += y * x
    denom = n * syy - sy * sy
    if denom == 0: return int(sx / n)
    slope = (n * syx - sy * sx) / denom
    intercept = (sx - slope * sy) / n
    return int(slope * target_y + intercept)


def fit_line_near_slope(points, target_y):
    n = len(points)
    if n <= 0: return None, 0
    if n == 1: return points[0][0], 0
    sy = sx = syy = syx = 0
    for x, y, _ in points:
        sy += y; sx += x; syy += y * y; syx += y * x
    denom = n * syy - sy * sy
    if denom == 0: return int(sx / n), 0
    slope = (n * syx - sy * sx) / denom
    intercept = (sx - slope * sy) / n
    return int(slope * target_y + intercept), slope


def find_line(img, thresholds):
    found_blobs = []
    points = []
    for roi in LINE_ROIS:
        blob = best_blob_in_roi(img, roi, thresholds)
        if blob:
            points.append((blob.cx(), blob.cy(), roi[4] * blob.pixels()))
            found_blobs.append(blob)
    if not points:
        return None, None, 0, 0, 0, found_blobs, points

    wx = wt = 0
    for x, _, w in points:
        wx += x * w; wt += w
    center_x = int(wx / wt)
    near_x, slope = fit_line_near_slope(points, FIT_NEAR_Y)
    look_x = fit_x_at_y(points, FIT_LOOKAHEAD_Y)
    if near_x is None: near_x = center_x
    if look_x is None: look_x = center_x
    near_err = near_x - IMG_CENTER_X
    look_err = look_x - IMG_CENTER_X
    head_err = int(slope * (FIT_NEAR_Y - FIT_LOOKAHEAD_Y))
    pos_err = int((near_err * STEER_NEAR_GAIN + look_err * STEER_LOOKAHEAD_GAIN) / STEER_ERROR_DIV)
    dir_err = int((head_err * STEER_HEADING_GAIN) / STEER_ERROR_DIV)
    if abs(near_err) >= NEAR_POSITION_PRIORITY_X and pos_err * dir_err < 0:
        dir_err = 0
    steer_err = int(pos_err + dir_err)
    return (center_x, near_x, steer_err, pos_err, dir_err, found_blobs, points)


def find_corner_hint(img, thresholds):
    lp, lb = black_pixels_in_roi(img, SIDE_ROIS[0], thresholds)
    rp, rb = black_pixels_in_roi(img, SIDE_ROIS[1], thresholds)
    cross = (lp >= CROSS_SIDE_MIN_PIXELS and rp >= CROSS_SIDE_MIN_PIXELS)
    if lp < CORNER_HINT_MIN_PIXELS and rp < CORNER_HINT_MIN_PIXELS:
        return 0, lb + rb, False
    if cross:
        return DIR_RIGHT, lb + rb, True
    if lp > rp * SIDE_HINT_RATIO:
        return DIR_LEFT, lb + rb, False
    if rp > lp * SIDE_HINT_RATIO:
        return DIR_RIGHT, lb + rb, False
    return 0, lb + rb, False


# ========== UI ==========

def draw_ui(img, running, line_x, raw_x, steer_err, fit_pts, blobs, side_blobs,
            flags, speed, turn, fps, lost_ms, searching, l_max):
    state_color = GREEN if running else WHITE
    for x, y, w, h, _ in LINE_ROIS:
        img.draw_rectangle((x, y, w, h), color=BLUE, thickness=1)
    for x, y, w, h in SIDE_ROIS:
        img.draw_rectangle((x, y, w, h), color=WHITE, thickness=1)
    img.draw_line(IMG_CENTER_X, 150, IMG_CENTER_X, IMG_H - 1, color=WHITE, thickness=1)
    for blob in blobs:
        img.draw_rectangle(blob.rect(), color=YELLOW, thickness=2)
        img.draw_cross(blob.cx(), blob.cy(), color=YELLOW)
    for x, y, _ in fit_pts:
        img.draw_cross(x, y, color=GREEN, thickness=1)
    for blob in side_blobs:
        img.draw_rectangle(blob.rect(), color=BLUE, thickness=1)
    if line_x is not None:
        color = GREEN if abs(line_x - IMG_CENTER_X) <= TURN_OFF_DEAD_ZONE_X else RED
        img.draw_line(line_x, 150, line_x, IMG_H - 1, color=color, thickness=2)
        img.draw_cross(line_x, 205, color=color, thickness=2)
    if raw_x is not None:
        img.draw_line(raw_x, 150, raw_x, IMG_H - 1, color=YELLOW, thickness=1)
    state = "RUN" if running else "STOP"
    if searching: state = "SEARCH"
    err = 0 if line_x is None else steer_err
    img.draw_string(0, 0, "%s fps:%2.1f S:%d T:%d L:%02d" %
                    (state, fps, speed, turn, l_max), color=state_color, scale=2)
    img.draw_string(0, 22, "frame:%s err:%d lost:%d" %
                    ("NO" if line_x is None else "OK", err, lost_ms),
                    color=GREEN if line_x is not None else RED, scale=1)
    img.draw_string(0, 36, "A5 %02X %02X 01 %02X 5A" %
                    (speed, turn & 0xFF, (speed + (turn & 0xFF) + 1) & 0xFF),
                    color=WHITE, scale=1)


# ========== 主循环 ==========

def main():
    init_camera()
    key = KeyButton()
    link = Stm32Link()
    clock = time.clock()

    running     = False
    speed       = 0
    turn        = 0
    last_seen_ms = time.ticks_ms()
    last_seen_side = SEARCH_DEFAULT_TURN
    search_dir     = SEARCH_DEFAULT_TURN
    stable_line_x  = None
    stable_steer   = 0
    stable_dir_err = 0
    seen_frames    = 0
    lost_frames    = 0
    startup_left   = 0
    corner_frames  = 0
    corner_last    = 0

    while True:
        clock.tick()
        img = sensor.snapshot()
        now = time.ticks_ms()

        if key.pressed_event():
            running = not running
            stable_line_x = None
            stable_steer = 0
            stable_dir_err = 0
            seen_frames = 0
            lost_frames = 0
            last_seen_ms = now
            search_dir = SEARCH_DEFAULT_TURN
            last_seen_side = SEARCH_DEFAULT_TURN
            corner_frames = 0
            corner_last = 0
            if running:
                startup_left = START_SPEED_STEPS

        # ---- 检测 ----
        thresholds, l_max = get_black_thresholds(img)
        (raw_x, raw_near_x, raw_steer, raw_pos, raw_dir,
         blobs, fit_pts) = find_line(img, thresholds)
        corner_hint, side_blobs, crossroad = find_corner_hint(img, thresholds)

        # ---- 拐角确认 ----
        if corner_hint and corner_hint == corner_last:
            corner_frames += 1
        elif corner_hint:
            corner_last = corner_hint
            corner_frames = 1
        else:
            corner_last = 0
            corner_frames = 0

        # ---- 平滑 ----
        if raw_x is not None:
            seen_frames += 1
            lost_frames = 0
            if stable_line_x is None:
                stable_line_x = raw_x
                stable_steer = raw_steer
                stable_dir_err = raw_dir
            else:
                stable_line_x = (stable_line_x * (LINE_SMOOTH_NUM - 1) + raw_x) // LINE_SMOOTH_NUM
                stable_steer  = (stable_steer  * (LINE_SMOOTH_NUM - 1) + raw_steer) // LINE_SMOOTH_NUM
                stable_dir_err = (stable_dir_err * (LINE_SMOOTH_NUM - 1) + raw_dir) // LINE_SMOOTH_NUM
        else:
            lost_frames += 1
            seen_frames = 0
            if lost_frames >= LINE_LOST_CONFIRM_FRAMES:
                stable_line_x = None
                stable_steer = 0
                stable_dir_err = 0

        line_seen  = (raw_x is not None)
        line_ready = (stable_line_x is not None and seen_frames >= LINE_SEEN_CONFIRM_FRAMES)
        steer_now  = 0 if stable_line_x is None else stable_steer
        searching  = False

        if steer_now < -TURN_OFF_DEAD_ZONE_X:
            last_seen_side = DIR_LEFT
        elif steer_now > TURN_OFF_DEAD_ZONE_X:
            last_seen_side = DIR_RIGHT

        corner_ok   = crossroad and corner_frames >= CORNER_HINT_CONFIRM_FRAMES
        corner_hint = corner_hint if corner_frames >= CORNER_HINT_CONFIRM_FRAMES else 0

        # ---- 决策 ----
        if running:
            if startup_left > 0:
                speed = (START_SPEED_STEPS - startup_left + 1) * 2
                turn  = 0
                startup_left -= 1
            elif line_ready:
                turn  = clamp(steer_now * 3, -127, 127)
                speed = STRAIGHT_SPEED if abs(steer_now) < 20 else CURVE_SPEED
                if corner_ok:
                    turn = 80
                elif corner_hint:
                    turn = 80 if corner_hint == DIR_RIGHT else -80
                last_seen_ms = now
            elif line_seen:
                speed = MIN_LINE_SPEED
                turn  = 0
            elif time.ticks_diff(now, last_seen_ms) <= LOST_KEEP_MS:
                searching = True
                speed = SEARCH_SPEED
                turn  = 80 if last_seen_side == DIR_RIGHT else -80
            else:
                searching = True
                speed = SEARCH_SPEED
                turn  = 80 if search_dir == DIR_RIGHT else -80
        else:
            speed = 0
            turn  = 0

        link.send_control(speed, turn)

        lost_ms = time.ticks_diff(now, last_seen_ms)
        draw_ui(img, running, stable_line_x, raw_near_x, steer_now,
                fit_pts, blobs, side_blobs, 0, speed, turn,
                clock.fps(), lost_ms, searching, l_max)
        lcd.display(img)


main()
