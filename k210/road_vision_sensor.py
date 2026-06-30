import sensor
import time
import lcd

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

# ---- 黑线检测 ----
BLACK_L_MAX = 40
BLACK_AB_LIMIT = 45

# ---- 7层ROI ----
ROI_H = 22
ROI_W = 290
ROI_X = 15
ROIS = [
    (ROI_X, 208, ROI_W, ROI_H),   # NEAR  (最近)
    (ROI_X, 182, ROI_W, ROI_H),
    (ROI_X, 156, ROI_W, ROI_H),
    (ROI_X, 130, ROI_W, ROI_H),
    (ROI_X, 104, ROI_W, ROI_H),
    (ROI_X, 78,  ROI_W, ROI_H),
    (ROI_X, 52,  ROI_W, ROI_H),   # FAR   (最远)
]
N_LAYERS = 7
WEIGHTS = [1.0, 0.85, 0.70, 0.55, 0.40, 0.25, 0.15]

# ---- 判定 ----
DEADZONE = 80          # |error| < 80px 不纠
SLOPE_THRESH = 20      # 斜率阈值, 越过说明弯道
MAX_INCREMENT = 50     # 单次最大增量 (°)
MIN_INCREMENT = 3      # 单次最小增量 (°)
TURN_POWER = 2         # 非线性幂指数 (1=线性, 越大越非线性)

# ---- 速度 ----
STRAIGHT_SPEED = 8
TURN_SPEED = 3
MAX_SPEED = 15

# ---- 发送 ----

# ---- 丢线 ----
LOST_ROTATE = 10      # 向消失方向旋转的角度

# ---- 显示 ----
DEBUG_MODE = True

# 黑线阈值
TH_BLACK  = [(0, BLACK_L_MAX, -BLACK_AB_LIMIT, BLACK_AB_LIMIT,
             -BLACK_AB_LIMIT, BLACK_AB_LIMIT)]
# 白线/红线阈值 (白=高L, 红=高A)
TH_WHITE  = [(60, 255, -45, 45, -45, 45)]    # 白色
TH_RED    = [(0, 100, 25, 128, -45, 45)]     # 红色 (低L+高A)
TH_WHITERED = TH_WHITE + TH_RED


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
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
                 time.ticks_diff(now, self.last_event_ms) > 250)
        if event:
            self.last_event_ms = now
        self.last_pressed = pressed
        return event


class Stm32Link:
    def __init__(self):
        self.actual_heading = 0
        self.actual_speed = 0
        self._rbuf = bytearray(16)
        self._ridx = 0
        try:
            from fpioa_manager import fm
            from machine import UART
            fm.register(6, fm.fpioa.UART2_RX)
            fm.register(8, fm.fpioa.UART2_TX)
            self.serial = UART(UART.UART2, 115200, 8, 0, 0,
                               timeout=0, read_buf_len=128)
            self.mode = "uart2"
        except Exception:
            if ybserial:
                self.serial = ybserial()
                self.mode = "ybserial"
            else:
                self.serial = None
                self.mode = "none"

    def send_request(self):
        """非阻塞: 发请求帧 A5 FF 00 00 00 5A"""
        if self.serial is None:
            return
        try:
            self.serial.write(bytes([HEAD, 0xFF, 0, 0, 0,
                                     (0xFF + 0 + 0 + 0) & 0xFF, TAIL]))
        except Exception:
            pass

    def read_response(self, timeout_ms=5):
        """非阻塞+超时: poll缓冲, 超时返回False"""
        if self.serial is None:
            return False
        t0 = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
            try:
                if self.serial.any():
                    b = self.serial.read(1)[0]
                    if b == HEAD:
                        self._ridx = 0
                    if self._ridx < 16:
                        self._rbuf[self._ridx] = b
                        self._ridx += 1
                    if self._ridx == 7 and self._rbuf[6] == TAIL:
                        chk = (self._rbuf[1] + self._rbuf[2] +
                               self._rbuf[3] + self._rbuf[4]) & 0xFF
                        if chk == self._rbuf[5]:
                            self.actual_heading = (self._rbuf[2]
                                if self._rbuf[2] < 128
                                else self._rbuf[2] - 256)
                            self.actual_speed = self._rbuf[1]
                            return True
                # 无数据, 继续等超时
            except Exception:
                pass
        return False

    def send_cmd(self, target, speed):
        """7字节 A5 target 0 conf speed chk 5A"""
        target = int(clamp(target, -100, 100))
        speed = int(clamp(speed, 0, 30))
        t_u = target & 0xFF
        s_u = speed & 0xFF
        chk = (t_u + 0 + 100 + s_u) & 0xFF
        frame = [HEAD, t_u, 0, 100, s_u, chk, TAIL]
        try:
            if self.mode == "ybserial":
                self.serial.send_bytearray(frame)
            elif self.mode == "uart2":
                self.serial.write(bytes(frame))
        except Exception:
            pass


def init_camera():
    lcd.init()
    try:
        import touchscreen as ts
        ts.init()
    except Exception:
        pass
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    sensor.skip_frames(time=600)


def fit_quadratic(pts):
    """7点二次拟合 x=a*y²+b*y+c, 返回(a,b,c)或None"""
    n = len(pts)
    if n < 3:
        return None
    s_y4 = s_y3 = s_y2 = s_y1 = 0.0
    s_xy2 = s_xy = s_x = 0.0
    for x, y, *_ in pts:
        y2 = y * y
        s_y4 += y2 * y2
        s_y3 += y2 * y
        s_y2 += y2
        s_y1 += y
        s_xy2 += x * y2
        s_xy += x * y
        s_x += x
    det = (s_y4 * (s_y2 * n - s_y1 * s_y1) -
           s_y3 * (s_y3 * n - s_y1 * s_y2) +
           s_y2 * (s_y3 * s_y1 - s_y2 * s_y2))
    if abs(det) < 0.001:
        return None
    a = (s_xy2 * (s_y2 * n - s_y1 * s_y1) -
         s_y3 * (s_xy * n - s_x * s_y1) +
         s_y2 * (s_xy * s_y1 - s_y2 * s_x)) / det
    b = (s_y4 * (s_xy * n - s_x * s_y1) -
         s_xy2 * (s_y3 * n - s_y1 * s_y2) +
         s_y2 * (s_y3 * s_x - s_y2 * s_xy)) / det
    c = (s_y4 * (s_y2 * s_x - s_xy * s_y1) -
         s_y3 * (s_y3 * s_x - s_xy * s_y2) +
         s_xy2 * (s_y3 * s_y1 - s_y2 * s_y2)) / det
    return a, b, c


def detect_centers(img, mode):
    """7层ROI找黑线中心 → 二次拟合 + 单噪点剔除 → slope(tangent)"""
    th = TH_BLACK if mode == 0 else TH_WHITERED
    raw = []
    raw_blobs = []  # 每层的 blob 矩形列表
    for i, roi in enumerate(ROIS):
        blobs = img.find_blobs(th, roi=roi,
                               pixels_threshold=50,
                               area_threshold=120,
                               merge=True, margin=10)
        valid = []
        for b in blobs:
            if b.w() < 10:
                continue
            valid.append(b)
        valid.sort(key=lambda b: b.pixels(), reverse=True)

        raw_blobs.append(valid)
        if not valid:
            raw.append(None)
        elif len(valid) == 1:
            raw.append(valid[0].cx())
        else:
            if valid[1].pixels() < valid[0].pixels() * 0.25:
                raw.append(valid[0].cx())
            else:
                cx = (valid[0].cx() + valid[1].cx()) // 2
                raw.append(cx)

    # 有效点列表 (x, y, weight, index)
    pts = []
    for i, cx in enumerate(raw):
        if cx is not None:
            y = ROIS[i][1] + ROI_H // 2
            pts.append((cx, y, WEIGHTS[i], i))

    n_valid = len(pts)
    if n_valid < 3:
        return raw, raw_blobs, None, 0, 0

    # 二次拟合
    ret = fit_quadratic(pts)
    if ret is None:
        # 线性拟合
        sum_x = 0
        sum_w = 0
        for cx, y, w, _ in pts:
            sum_x += cx * w
            sum_w += w
        return raw, raw_blobs, int(sum_x / sum_w), 0, 0
    a, b, c = ret

    # 残差检测: 剔除最大残差
    residuals = []
    for cx, y, w, i in pts:
        pred = a * y * y + b * y + c
        residuals.append((abs(cx - pred) / (w + 0.01), w, i))
    residuals.sort(reverse=True)
    # 若最坏残差 > 2×中位残差 → 剔除
    med = residuals[len(residuals)//2][0]
    if residuals[0][0] > med * 2.5 and n_valid > 3:
        # 剔除最坏点
        bad_i = residuals[0][2]
        pts2 = [(cx, y, w, i) for cx, y, w, i in pts if i != bad_i]
        ret2 = fit_quadratic(pts2)
        if ret2 is not None:
            a, b, c = ret2
            pts = pts2

    # 加权平均 → target_x
    sum_x = 0
    sum_w = 0
    for cx, y, w, _ in pts:
        sum_x += cx * w
        sum_w += w
    target_x = int(sum_x / sum_w)
    error = target_x - IMG_CENTER_X

    # 近端切线 = 2a*y_near + b  (这就是 slope)
    y_near = pts[0][1]
    slope = 2.0 * a * y_near + b

    return raw, raw_blobs, target_x, error, slope


def main():
    init_camera()
    key = KeyButton()
    link = Stm32Link()
    clock = time.clock()

    running = False
    last_turn_dir = 1
    target_hdg = 0
    speed = 0
    detect_mode = 0
    turn_power = TURN_POWER
    centers = [None] * N_LAYERS
    was_turning = False
    brake_cnt = 0

    while True:
        try:
            clock.tick()
            img = sensor.snapshot()
            now = time.ticks_ms()

            if key.pressed_event():
                running = not running
                if not running:
                    link.send_cmd(0, 0)

            # 触摸
            if not running:
                try:
                    import touchscreen as ts
                    status, tx, ty = ts.read()
                    if status == ts.STATUS_PRESS:
                        if ty < 40:
                            detect_mode = 1 - detect_mode
                        elif ty >= 120:
                            if tx < 160:
                                turn_power = clamp(turn_power - 1, 1, 10)
                            else:
                                turn_power = clamp(turn_power + 1, 1, 10)
                except Exception:
                    pass
            # 流水线: 发请求 → 并行处理图像 → 读回传 → 决策
            if running:
                link.send_request()                       # ① 先发请求
            centers, raw_blobs, target_x, error, slope = detect_centers(img, detect_mode)  # ② 处理图像
            if running:
                link.read_response()                      # ③ 读STM32回传(图像处理完已就绪)
                # ④ 决策
                if target_x is not None:
                    is_turning = (abs(error) >= DEADZONE or
                                  abs(slope) >= SLOPE_THRESH)
                    if not is_turning:
                        target_hdg = link.actual_heading
                        speed = STRAIGHT_SPEED
                    else:
                        inc_e = 0
                        ss = 1 if slope > 0 else -1
                        sn = abs(slope)
                        inc_s = ss * ((sn * 3) ** turn_power) * 0.8
                        inc = inc_e + inc_s
                        inc = clamp(int(inc), -MAX_INCREMENT, MAX_INCREMENT)
                        if 0 < inc < MIN_INCREMENT:
                            inc = MIN_INCREMENT
                        elif -MIN_INCREMENT < inc < 0:
                            inc = -MIN_INCREMENT
                        target_hdg = link.actual_heading + inc
                        target_hdg = clamp(target_hdg, -127, 127)
                        last_turn_dir = 1 if inc > 0 else -1
                        if not was_turning:
                            brake_cnt = 2
                        speed = 1 if brake_cnt > 0 else TURN_SPEED
                        if brake_cnt > 0:
                            brake_cnt -= 1
                    was_turning = is_turning
                else:
                    target_hdg = link.actual_heading + LOST_ROTATE * last_turn_dir
                    target_hdg = clamp(target_hdg, -127, 127)
                    speed = 0
                    was_turning = False

                speed = clamp(speed, 0, MAX_SPEED)
                link.send_cmd(target_hdg, speed)          # ⑤ 发指令
            fps = clock.fps()
            # ROI + 每个黑块中心 + 最终中心
            for i, roi in enumerate(ROIS):
                img.draw_rectangle(roi, color=(0,180,0), thickness=1)
                cy = roi[1] + roi[3] // 2
                # 绿色* = 每个检测到的黑块中心
                if i < len(raw_blobs):
                    for b in raw_blobs[i][:2]:
                        img.draw_cross(int(b.cx()), cy, color=(0,255,0), size=4, thickness=1)
                # 红色+ = 最终计算中心
                cx = centers[i] if i < len(centers) else None
                if cx is not None:
                    img.draw_cross(int(cx), cy, color=(255,0,0), size=6, thickness=2)

            mstr = "BLK" if detect_mode == 0 else "WHT+RED"
            if not running:
                img.draw_string(0, 0, "STOP", color=(255,0,0), scale=2)
                img.draw_rectangle(230, 0, 90, 24, color=(0,0,200), thickness=2)
                img.draw_string(238, 4, mstr, color=(255,255,0), scale=2)
                img.draw_string(0, 26, "P:%d" % turn_power, color=(255,255,0), scale=2)
                img.draw_string(0, 52, "FPS:%2.1f" % fps, color=(255,255,0), scale=2)
                img.draw_rectangle(0, 120, 160, 120, color=(0,150,0), thickness=3)
                img.draw_string(30, 168, "P-1", color=(0,255,0), scale=3)
                img.draw_rectangle(160, 120, 160, 120, color=(0,150,0), thickness=3)
                img.draw_string(190, 168, "P+1", color=(0,255,0), scale=3)
            else:
                img.draw_string(0, 0, "RUN", color=(0,255,0), scale=2)
                img.draw_string(0, 26, "ang:%d spd:%d" % (target_hdg, speed), color=(0,255,0), scale=2)
                img.draw_string(0, 52, "cur:%d %d" % (link.actual_heading, link.actual_speed), color=(255,255,0), scale=2)
                img.draw_string(0, 78, "FPS:%2.1f" % fps, color=(255,255,0), scale=2)
            lcd.display(img)

        except Exception as err:
            running = False
            try:
                link.send_cmd(0, 0)
                img.draw_string(0, 0, "ERR:" + str(err), color=(255,0,0), scale=2)
                lcd.display(img)
            except Exception:
                pass


main()
