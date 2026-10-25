#!/usr/bin/env python3
"""生成 16:9 PPT 用架构图位图（3840x2160）。运行: python3 make_slides.py"""
from PIL import Image, ImageDraw, ImageFont
import math, os

W, H = 3840, 2160
OUT = os.path.dirname(os.path.abspath(__file__))
FR = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
FB = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"

BG = (247, 249, 252); INK = (16, 35, 63); SUB = (90, 106, 133)
WHITE = (255, 255, 255); BORDER = (205, 215, 232)
BLUE = (37, 99, 235); TEAL = (14, 154, 139); ORANGE = (224, 122, 26)
RED = (214, 69, 69); PURPLE = (124, 92, 252); GREEN = (38, 152, 91)
GRAYB = (235, 240, 248)

def F(sz, bold=False):
    return ImageFont.truetype(FB if bold else FR, sz)

def rr(d, xy, fill, outline=BORDER, width=4, r=28):
    d.rounded_rectangle(xy, radius=r, fill=fill, outline=outline, width=width)

def center(d, cx, y, text, font, fill=INK):
    d.text((cx, y), text, font=font, fill=fill, anchor="ma")

def block(d, xy, title, lines, accent=BLUE, title_sz=54, body_sz=39, fill=WHITE, tcol=INK):
    x1, y1, x2, y2 = xy
    rr(d, xy, fill, outline=accent, width=5, r=30)
    d.rounded_rectangle((x1, y1, x2, y1 + 14), radius=7, fill=accent)
    cx = (x1 + x2) / 2
    y = y1 + 26
    center(d, cx, y, title, F(title_sz, True), tcol); y += title_sz + 16
    for ln in lines:
        center(d, cx, y, ln, F(body_sz), SUB if tcol == INK else tcol); y += body_sz + 10

def arrow(d, pts, color=INK, width=7, label=None, lx=0, ly=0, lsz=34):
    for a, b in zip(pts, pts[1:]):
        d.line([a, b], fill=color, width=width)
    (x1, y1), (x2, y2) = pts[-2], pts[-1]
    ang = math.atan2(y2 - y1, x2 - x1); s = 34
    for da in (2.65, -2.65):
        d.line([(x2, y2), (x2 + s * math.cos(ang + da), y2 + s * math.sin(ang + da))], fill=color, width=width)
    if label:
        tw = d.textlength(label, font=F(lsz, True))
        d.rounded_rectangle((lx - tw/2 - 16, ly - 30, lx + tw/2 + 16, ly + 34), radius=14, fill=BG)
        d.text((lx, ly), label, font=F(lsz, True), fill=color, anchor="mm")

def header(d, num, title, subtitle):
    d.rectangle((0, 0, W, 240), fill=INK)
    d.rectangle((0, 240, W, 256), fill=BLUE)
    d.text((110, 52), title, font=F(92, True), fill=WHITE, anchor="la")
    d.text((112, 158), subtitle, font=F(42), fill=(178, 198, 232), anchor="la")
    d.text((W - 110, 78), num, font=F(56, True), fill=(120, 160, 235), anchor="ra")
    d.text((W - 110, 150), "CyberDuck  ·  16:9", font=F(36), fill=(150, 175, 215), anchor="ra")

def footer(d, text):
    d.text((110, H - 78), text, font=F(34), fill=SUB, anchor="la")
    d.text((W - 110, H - 78), "Radxa Dragon Q6A · QCS6490", font=F(34), fill=SUB, anchor="ra")

# ---------------- Slide 1: 电路 / 电源 ----------------
def slide1():
    img = Image.new("RGB", (W, H), BG); d = ImageDraw.Draw(img)
    header(d, "01 / 03", "电路与电源架构", "4S 电池包（自带 BMS）· 双开关分域供电：主控电源 / 关节电源")
    # 电池 -> 保险 -> 分电
    block(d, (110, 420, 760, 800), "4S 电池包", ["标称 14.8 V · 满电 16.8 V", "自带 BMS", "过充 / 过放 / 过流 / 短路保护"], GREEN, 56, 40)
    block(d, (110, 920, 760, 1150), "总输入保护", ["主保险 · 防反接（建议）"], ORANGE, 50, 38)
    arrow(d, [(435, 800), (435, 920)], GREEN)
    # 上支路 主控
    block(d, (900, 400, 1560, 640), "开关 1 · 主控电源", ["SW_MAIN"], BLUE, 50, 44)
    block(d, (1700, 400, 2360, 640), "主控 DC-DC", ["降压至 Q6A 额定输入", "电压待按规格核对"], BLUE, 50, 38)
    arrow(d, [(760, 1040), (835, 1040), (835, 520), (900, 520)], BLUE, label="分路 1", lx=835, ly=760)
    arrow(d, [(1560, 520), (1700, 520)], BLUE)
    arrow(d, [(2360, 520), (2540, 520)], BLUE)
    # 下支路 关节
    block(d, (900, 920, 1560, 1160), "开关 2 · 关节电源", ["SW_JOINT"], RED, 50, 44)
    block(d, (1700, 920, 2360, 1160), "关节动力母线", ["VBUS_JOINT", "直供或 DC-DC，待核对电机额定电压"], RED, 50, 36)
    arrow(d, [(760, 1040), (900, 1040)], RED, label="分路 2", lx=830, ly=990)
    d.ellipse((822, 1027, 848, 1053), fill=INK)
    arrow(d, [(1560, 1040), (1700, 1040)], RED)
    arrow(d, [(2360, 1040), (2540, 1040)], RED)
    # 主控电源域
    rr(d, (2540, 330, 3730, 1210), (238, 244, 255), BLUE, 6, 36)
    center(d, 3135, 352, "主控电源域（SW_MAIN）", F(56, True), BLUE)
    block(d, (2620, 450, 3650, 690), "Radxa Dragon Q6A 主控", ["Linux · 摄像头 / 传感器 / 电机总线控制中心"], BLUE, 52, 38)
    block(d, (2620, 730, 3120, 1010), "摄像头 IMX219", ["CAM3 · MIPI CSI-2 图像", "CCI I2C 控制"], TEAL, 46, 36)
    block(d, (3180, 730, 3650, 1010), "麦克风 + 扬声器", ["I2C 配置寄存器", "I2S 音频数据"], PURPLE, 46, 36)
    block(d, (2620, 1040, 3650, 1180), "灯环 · 触摸 · 蓝牙 / WiFi", ["低压外设"], BLUE, 42, 34)
    # I2C 总线框
    rr(d, (110, 1330, 2360, 1960), WHITE, TEAL, 6, 36)
    center(d, 1235, 1360, "一路 I2C 总线 · I2C6（GPIO24 SDA / GPIO25 SCL · 软件 I2C）", F(54, True), TEAL)
    block(d, (170, 1460, 830, 1760), "IMU #1", ["LSM6DSV16X", "地址 0x6A"], TEAL, 52, 42)
    block(d, (910, 1460, 1570, 1760), "IMU #2", ["LSM6DSV16X", "地址 0x6B"], TEAL, 52, 42)
    block(d, (1650, 1460, 2310, 1760), "8×8 TOF", ["VL53L5CX", "地址 0x29"], ORANGE, 52, 42)
    center(d, 1235, 1820, "主控 Q6A 为总线主机 · 三设备地址不冲突 · 当前测试程序独占总线，不可同时运行", F(38), SUB)
    arrow(d, [(2850, 1210), (2850, 1300), (1235, 1300), (1235, 1330)], TEAL, label="I2C6", lx=2050, ly=1270)
    # 关节电源域
    rr(d, (2540, 1330, 3730, 1960), (255, 241, 241), RED, 6, 36)
    center(d, 3135, 1360, "关节电源域（SW_JOINT）", F(56, True), RED)
    block(d, (2620, 1460, 3120, 1760), "腿部电机", ["右腿 ID 1~5", "左腿 ID 6~10"], RED, 46, 38)
    block(d, (3180, 1460, 3650, 1760), "头颈 / 嘴", ["头颈 ID 11~14", "嘴 ID 15"], RED, 46, 38)
    center(d, 3135, 1820, "PCAN-USB → CAN_H / CAN_L · 1 Mbps · 两端 120Ω 终端", F(38, True), RED)
    # CAN 线：从主控域右侧绕下到关节域
    arrow(d, [(3650, 900), (3780, 900), (3780, 1610), (3650, 1610)], BLUE, label="CAN 总线", lx=3135, ly=1272)
    center(d, 1920, 2012, "上电顺序：先主控 → 拉起 can0 → 再开关节    ·    下电反序    ·    动力地与信号地共地、星形汇聚", F(40, True), INK)
    footer(d, "注：DC-DC 输出电压与关节母线电压需按 Q6A 与电机规格书最终核对")
    img.save(os.path.join(OUT, "slide1_power_circuit.png"))

# ---------------- Slide 2: 软件架构 ----------------
def slide2():
    img = Image.new("RGB", (W, H), BG); d = ImageDraw.Draw(img)
    header(d, "02 / 03", "软件架构", "分层设计：应用行为 → 算法中间层 → 驱动封装 → Linux HAL → 硬件")
    layers = [
        ("应用 /\n行为层", BLUE, [
            ("行走控制", ["locomotion", "wheel_drive"]), ("动作姿态", ["pose_contrl", "teach 示教"]),
            ("人脸跟随", ["FaceLocate", "face_detect"]), ("语音行为", ["duck · synth", "voices · TTS"]),
            ("蓝牙配网", ["BT/app", "BLE · WiFi · 遥测"]), ("视频推流", ["stream_camera", "HTTP 8088 MJPEG"])]),
        ("算法 /\n中间层", PURPLE, [
            ("ONNX 策略引擎", ["engine.cpp", "CPU / QNN HTP"]), ("轮驱分配", ["差速 · 自转", "机身升降插值"]),
            ("人脸检测", ["YuNet ONNX", "Haar 级联"]), ("安全限幅", ["力矩上限 · 缩放", "超时保护"]), ("姿态估计", ["IMU 外参校准", "imu_mount.json"])]),
        ("驱动 /\n封装层", TEAL, [
            ("电机总线", ["ht_can.py", "MotorBus / Motor"]), ("摄像头", ["camera_csi.py", "open_camera()"]),
            ("IMU 驱动", ["lsm6dsv16x", "四元数 · 六轴"]), ("TOF 驱动", ["VL53L5CX ULD", "8×8 连续测距"]), ("音频播放", ["play.py", "ALSA aplay"])]),
        ("系统 /\nHAL", ORANGE, [
            ("SocketCAN", ["can0 · 1 Mbps", "setup_can0.sh"]), ("libcamera", ["GStreamer", "libcamerasrc"]),
            ("GPIO 软件 I2C", ["i2c_gpio.c", "GPIO24 / 25"]), ("ALSA / PipeWire", ["当前 USB 声卡", "目标 I2S Codec"]), ("BlueZ / 输入", ["BLE · js0 手柄"])]),
        ("硬件层", GREEN, [
            ("关节电机", ["PCAN-USB", "ID 1~15"]), ("IMX219", ["CAM3", "CSI-2"]),
            ("双 IMU + TOF", ["0x6A · 0x6B", "0x29"]), ("音频模块", ["麦克风 + 扬声器", "I2C + I2S"]), ("手柄 / 触摸 / 灯环", ["GPIO", ""])]),
    ]
    y = 330
    for name, col, boxes in layers:
        h = 330
        rr(d, (110, y, 560, y + h), col, col, 4, 30)
        lines = name.split("\n")
        for i, ln in enumerate(lines):
            center(d, 335, y + h/2 - 60 + i * 72 - 30, ln, F(56, True), WHITE)
        n = len(boxes); x = 620; gap = 24; bw = (3730 - x - gap * (n - 1)) / n
        for t, ls in boxes:
            block(d, (x, y + 18, x + bw, y + h - 18), t, [x_ for x_ in ls if x_], col, 44, 36)
            x += bw + gap
        if y > 330:
            arrow(d, [(1920, y - 14), (1920, y + 16)], SUB, 5)
        y += h + 28
    footer(d, "注：音频当前走 USB 声卡，I2C+I2S Codec 为目标接入方案")
    img.save(os.path.join(OUT, "slide2_software.png"))

# ---------------- Slide 3: 控制架构 ----------------
def slide3():
    img = Image.new("RGB", (W, H), BG); d = ImageDraw.Draw(img)
    header(d, "03 / 03", "控制架构", "三层闭环：电机内环伺服 · 主控 50 Hz 策略环 · 低频行为层（视觉 / 语音 / 避障）")
    # 指令来源
    rr(d, (110, 340, 900, 1990), WHITE, BLUE, 5, 32)
    center(d, 505, 370, "指令来源", F(56, True), BLUE)
    block(d, (160, 480, 850, 760), "手柄", ["/dev/input/js0", "摇杆 · 按键"], BLUE, 50, 38)
    block(d, (160, 830, 850, 1110), "蓝牙 / 上位机", ["BLE 指令", "WiFi 遥测"], BLUE, 50, 38)
    block(d, (160, 1180, 850, 1460), "自主行为", ["人脸跟随", "TOF 避障 · 语音"], PURPLE, 50, 38)
    block(d, (160, 1530, 850, 1810), "感知输入", ["双 IMU 姿态", "相机 · TOF · 麦克风"], TEAL, 50, 38)
    # 控制核心
    rr(d, (1020, 340, 2640, 1990), (238, 244, 255), BLUE, 6, 32)
    center(d, 1830, 370, "控制核心 · Radxa Dragon Q6A", F(56, True), BLUE)
    block(d, (1100, 480, 2560, 740), "状态整理", ["IMU 四元数 / 角速度 + 关节位置·速度·力矩反馈 → 策略观测"], TEAL, 48, 36)
    block(d, (1100, 810, 2560, 1120), "ONNX 行走策略（50 Hz）", ["vx / vy / vyaw 限幅 · 指令斜坡 ramp 2 s", "ONNX Runtime CPU / QNN HTP 推理"], PURPLE, 50, 38)
    block(d, (1100, 1190, 2560, 1450), "轮驱控制", ["左右轮差速（左 6 / 右 1）· 原地自转 · 机身升降插值"], BLUE, 48, 36)
    block(d, (1100, 1520, 2560, 1810), "安全监督", ["电机超时 100 ms · 腿力矩 ≤ 0.5 N·m", "轮驱力矩保护 1 N·m · 死区 0.12"], RED, 48, 38, (255, 241, 241))
    # 执行层
    rr(d, (2760, 340, 3730, 1990), WHITE, GREEN, 5, 32)
    center(d, 3245, 370, "执行层", F(56, True), GREEN)
    block(d, (2820, 480, 3670, 760), "电机总线 MotorBus", ["ht_can.py · SocketCAN", "29 位扩展帧 · 1 Mbps"], GREEN, 46, 36)
    block(d, (2820, 830, 3670, 1110), "电机内置伺服环", ["位置 0x0A · 速度 0x0B", "力矩 0x0C"], GREEN, 46, 36)
    block(d, (2820, 1180, 3670, 1520), "关节电机 1~14", ["右腿 1~5 · 左腿 6~10", "头颈 11~14（嘴 15 独立）"], RED, 48, 38)
    block(d, (2820, 1590, 3670, 1810), "扬声器", ["TTS / 鸭叫合成播放"], PURPLE, 46, 36)
    arrow(d, [(3245, 760), (3245, 830)], GREEN, 6)
    arrow(d, [(3245, 1110), (3245, 1180)], GREEN, 6)
    # 横向箭头
    arrow(d, [(900, 900), (1020, 900)], BLUE, label="速度 / 动作指令", lx=960, ly=840)
    arrow(d, [(2640, 900), (2760, 900)], GREEN, label="关节目标", lx=2700, ly=840)
    arrow(d, [(900, 1650), (1020, 1650)], TEAL, label="姿态 / 环境", lx=960, ly=1590)
    # 反馈回路：电机 -> 底部总线 -> 左侧上行 -> 状态整理
    arrow(d, [(2820, 1350), (2790, 1350), (2790, 1905), (975, 1905), (975, 610), (1100, 610)], RED, 6)
    d.rounded_rectangle((1450, 1868, 3050, 1942), radius=16, fill=BG)
    center(d, 2250, 1888, "CAN 状态反馈：位置 · 速度 · 力矩 → 状态整理（闭环）", F(36, True), RED)
    center(d, 1920, 2045, "行为层只给速度 / 动作指令，不直接驱动电机；安全限幅始终位于策略环内", F(42, True), INK)
    footer(d, "轮驱模式：左轮 ID 6 · 右轮 ID 1（反装取反）· 全程力矩保护 1 N·m")
    img.save(os.path.join(OUT, "slide3_control.png"))

if __name__ == "__main__":
    slide1(); slide2(); slide3()
    print("done:", os.listdir(OUT))
