# Chaseline — K210 视觉巡线 (4线检测 + 优先级 + 丢线自转)
#
# 4条检测线, 由远及近优先级递减:
#   ROI_FAR: 仅预测减速
#   ROI_1~3: 转弯判断, 优先用最远的有效线
#   全部丢线 → 向最后偏离方向旋转找回
#
# 赛道黑块: 白色虚线可见时左右分两块 → 取中点
#           虚线不可见时一块 → 取质心

import sensor, image, time, lcd
from modules import ybserial

ser = ybserial()
lcd.init()
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=100)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
clock = time.clock()

# ==================== ROI 定义 ====================
# (name, x, y, w, h, dead, role)
ROIS = [
    {"name": "FAR",  "x": 30, "y": 20,  "w": 260, "h": 40, "dead": 14, "role": "predict"},
    {"name": "R1",   "x": 30, "y": 65,  "w": 260, "h": 40, "dead": 8,  "role": "turn"},
    {"name": "R2",   "x": 30, "y": 110, "w": 260, "h": 40, "dead": 8,  "role": "turn"},
    {"name": "R3",   "x": 40, "y": 155, "w": 240, "h": 40, "dead": 8,  "role": "turn"},
]

# ==================== 阈值 ====================
black_threshold = (0, 35, -20, 20, -20, 20)
MIN_AREA = 600       # 提高最小面积, 过滤噪声碎块
MERGE_MARGIN = 15    # 加大合并间距, 把碎块合并成完整路面

ALPHA = 0.5          # 低通 (降低响应更快)
last_x = {}           # 每线各自平滑

# 丢线自转
lost_dir = 0          # 最后偏离方向: +1=右, -1=左
LOST_TURN_SPEED = 4   # 丢线时自转的 X 偏移量(约25像素)


# ==================== 核心函数 ====================
def find_track_center(roi_img, roi):
    """找路面中心。≥2块黑→中点, 1块→质心, 0→None"""
    blobs = roi_img.find_blobs([black_threshold],
                               pixels_threshold=MIN_AREA,
                               area_threshold=MIN_AREA,
                               merge=True, margin=MERGE_MARGIN)
    if not blobs:
        return None, []

    big = [b for b in blobs if b.area() >= MIN_AREA]
    if not big:
        return None, []

    if len(big) >= 2:
        big.sort(key=lambda b: b.cx())
        cx = (big[0].cx() + big[-1].cx()) / 2.0
    else:
        cx = big[0].cx()

    return cx + roi["x"], big


def draw_blobs(img, blobs, roi, color):
    for b in blobs:
        img.draw_rectangle(roi["x"] + b.x(), roi["y"] + b.y(),
                           b.w(), b.h(), color=color, thickness=1)


# ==================== 主循环 ====================
for r in ROIS:
    last_x[r["name"]] = 160

while True:
    clock.tick()
    img = sensor.snapshot()

    # ---- 每条线独立检测 ----
    results = {}
    for r in ROIS:
        roi_img = img.copy(roi=(r["x"], r["y"], r["w"], r["h"]))
        cx, blobs = find_track_center(roi_img, r)
        name = r["name"]

        if cx is not None:
            last_x[name] = ALPHA * last_x[name] + (1.0 - ALPHA) * cx
            results[name] = {"x": int(last_x[name]), "blobs": blobs, "valid": True}
        else:
            results[name] = {"x": int(last_x[name]), "blobs": [], "valid": False}

        # 画框和色块
        box_color = (0, 0, 255) if r["role"] == "predict" else (255, 0, 0)
        img.draw_rectangle(r["x"], r["y"], r["w"], r["h"], color=box_color, thickness=1)
        blob_color = (0, 200, 200) if r["role"] == "predict" else (255, 100, 100)
        draw_blobs(img, blobs, r, blob_color)
        if results[name]["valid"]:
            img.draw_cross(results[name]["x"], r["y"] + r["h"] // 2,
                           color=(0, 255, 0), size=6, thickness=1)

    # ---- 优先级决策 ----
    # 预测: 看 FAR
    far = results["FAR"]

    # 转弯: R1 > R2 > R3
    turn_line = None
    for name in ["R1", "R2", "R3"]:
        if results[name]["valid"]:
            turn_line = results[name]
            break

    if turn_line:
        turn_x = turn_line["x"]
        err = turn_x - 160
        lost_dir = 1 if err > 0 else (-1 if err < 0 else 0)

        if abs(err) > ROIS[1]["dead"]:  # 用 R1 的死区
            y = 0       # 转弯
        elif not far["valid"] or abs(far["x"] - 160) > far_dead:
            y = 200     # 预测减速
        else:
            y = 120     # 直行
        out_x = turn_x
    else:
        # 全部丢线 → 自转找回
        out_x = 160 + lost_dir * LOST_TURN_SPEED
        y = 0

    far_dead = ROIS[0]["dead"]
    ser.send("$%03d%03d#" % (out_x if turn_line else out_x, y))

    # ---- OLED 调试 ----
    fps = clock.fps()
    img.draw_string(0, 0,  "FPS:%.1f" % fps, color=(255,255,255), scale=1.3)
    valid_str = "".join(["1" if results[r["name"]]["valid"] else "0" for r in ROIS])
    img.draw_string(0, 12, "V:" + valid_str + " X:%d" % (out_x if turn_line else out_x),
                    color=(0,255,0), scale=1.3)
    mode_str = "LOST" if not turn_line else ({0:"TURN",200:"SLOW"}.get(y,"GO"))
    img.draw_string(0, 24, mode_str, color=(255,255,0), scale=1.3)
    lcd.display(img)

