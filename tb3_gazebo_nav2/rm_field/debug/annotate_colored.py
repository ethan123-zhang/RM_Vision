#!/usr/bin/env python3
"""给"颜色标注"出图加图例与尺寸标注（Windows 侧跑，PIL + 微软雅黑）。

输入（debug/render_colored.sh 产出）:
  shots/debug_colored_hero.png        3/4 低机位
  shots/debug_colored_wall.png        正立面近景（橙带 = 0.503 m 侧壁）
  shots/debug_colored_elevation.png   正立面全景

输出: shots/debug_colored_{hero,wall,elevation}_annotated.png
同时打印"像素实测 ↔ 几何换算"对照，作为尺度的独立证据。
"""
import math
import os

from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
RM = os.path.dirname(HERE)
SHOTS = os.path.join(RM, "shots")
FONT_PATH = r"C:\Windows\Fonts\msyh.ttc"

ORANGE = (242, 107, 13)
BLUE = (31, 97, 235)
CYAN = (13, 217, 242)
LIGHT = (22, 158, 46)
GROUND_GREY = (150, 150, 155)
WHITE = (255, 255, 255)
TEXT = (226, 230, 238)
WARM = (255, 232, 190)
PANEL = (16, 18, 24)


def font(size):
    return ImageFont.truetype(FONT_PATH, size)


def classify(rgb):
    r, g, b = rgb[:3]
    if g > 170 and b > 170 and r < 130:
        return "cyan"
    if b > 130 and b > r + 40 and r < 130:
        return "blue"
    if r > 110 and b < 100 and r > b + 50:
        return "orange"
    return ("sky" if 0.299 * r + 0.587 * g + 0.114 * b < 80 else "ground")


def column_runs(img, x):
    runs, prev = [], None
    for y in range(img.size[1]):
        c = classify(img.getpixel((x, y)))
        if c != prev:
            runs.append((y, c))
            prev = c
    return runs


def find_band(img, x, color):
    runs = column_runs(img, x)
    h = img.size[1]
    best = None
    for i, (y0, c) in enumerate(runs):
        if c != color:
            continue
        y1 = runs[i + 1][0] if i + 1 < len(runs) else h
        if best is None or (y1 - y0) > (best[1] - best[0]):
            best = (y0, y1)
    return best


def dashed(draw, x0, x1, y, color=WHITE, width=2, dash=18, gap=12):
    x = x0
    while x < x1:
        draw.line([(x, y), (min(x + dash, x1), y)], fill=color, width=width)
        x += dash + gap


def vee_arrow(draw, x, y0, y1, color, width=3, head=14):
    draw.line([(x, y0), (x, y1)], fill=color, width=width)
    for y, s in ((y0, 1), (y1, -1)):
        draw.line([(x, y), (x - head, y + s * head)], fill=color, width=width)
        draw.line([(x, y), (x + head, y + s * head)], fill=color, width=width)


def box(img, rect, lines, size=26, fill=TEXT, alpha=205, pad=18, lh=None):
    """在 img 上画半透明圆角面板 + 多行文字，返回新的 ImageDraw。"""
    d = ImageDraw.Draw(img)
    image_draw = d
    layer = Image.new("RGBA", img.size, (0, 0, 0, 0))
    ld = ImageDraw.Draw(layer)
    ld.rounded_rectangle(rect, 14, fill=PANEL + (alpha,))
    img.alpha_composite(layer)
    d = ImageDraw.Draw(img)
    lh = lh or int(size * 1.45)
    y = rect[1] + pad
    for line in lines:
        d.text((rect[0] + pad, y), line, font=font(size), fill=fill)
        y += lh
    return d


def with_caption(img, height, lines, size=26, fill=TEXT):
    """在图片底部追加一条说明带，避免压在画面内容上。"""
    w, h = img.size
    canvas = Image.new("RGBA", (w, h + height), PANEL + (255,))
    canvas.paste(img, (0, 0))
    d = ImageDraw.Draw(canvas)
    y = h + 14
    for line in lines:
        d.text((24, y), line, font=font(size), fill=fill)
        y += int(size * 1.5)
    return canvas


def annotate_hero():
    img = Image.open(os.path.join(SHOTS, "debug_colored_hero.png")).convert("RGBA")
    d = box(img, (40, 35, 900, 330), [
        "同一场地的颜色标注（几何一点没改，只改颜色）",
    ], size=30, fill=WHITE)
    rows = [
        (ORANGE, "橙色 = 场地侧壁：台面比周围地面高出的 0.503 m"),
        (LIGHT, "绿色 = 台面（z = 0.000 m，机器人站的地方）"),
        (BLUE, "蓝色 = 围栏（0.30 m 高，图纸里没有、为仿真补的）"),
        (CYAN, "青色 = 围栏顶沿（让墙顶这条线一眼看得出）"),
        (GROUND_GREY, "灰 = 场地外地面（z = -0.503 m，比台面低）"),
    ]
    d = ImageDraw.Draw(img)
    y = 120
    for color, text in rows:
        d.rectangle([66, y + 8, 122, y + 34], fill=color, outline=WHITE, width=1)
        d.text((140, y), text, font=font(26), fill=TEXT)
        y += 42
    out = with_caption(img, 92, [
        "橙色带的下边缘就是周围地面；橙带在它上面 0.503 m —— 场地在地面之上，不是在地底下。",
    ], size=27, fill=WARM)
    dst = os.path.join(SHOTS, "debug_colored_hero_annotated.png")
    out.convert("RGB").save(dst)
    print("saved", dst)


def annotate_wall():
    img = Image.open(os.path.join(SHOTS, "debug_colored_wall.png")).convert("RGBA")
    w, h = img.size
    x_mid = w // 2

    blue = find_band(img, x_mid, "blue")
    orange = find_band(img, x_mid, "orange")
    ground = find_band(img, x_mid, "ground")

    cam_z, dist = -0.2515, 9.0 - 4.3
    f = (w / 2) / math.tan(0.30 / 2)
    top_row, bot_row = orange
    h_px = bot_row - top_row
    h_m = h_px * dist / f
    print(f"[wall] 橙色带 rows {top_row}..{bot_row} = {h_px} px -> {h_m:.4f} m（模型 0.5030）")
    print(f"[wall] 预测行 台面 z=0 -> {h/2 - f*(0-cam_z)/dist:.1f}；地面 z=-0.503 -> {h/2 - f*(-0.503-cam_z)/dist:.1f}")
    print(f"[wall] 蓝带 {blue}（围栏，上边缘出画）；地面起始行 {ground[0]}")

    d = ImageDraw.Draw(img)
    dashed(d, 0, w, top_row)
    dashed(d, 0, w, bot_row)

    d = box(img, (20, 20, 640, 92), ["蓝色 = 围栏（0.30 m 高，图纸外补）"], size=25, fill=(200, 225, 255))
    d = box(img, (670, top_row - 66, 1060, top_row - 8), ["台面 z = 0.000 m"], size=27, fill=WHITE)
    d.line([(866, top_row - 8), (866, top_row)], fill=WHITE, width=2)
    d = box(img, (60, 250, 800, 300), ["橙色 = 场地侧壁：0.503 m 高的凸起部分"], size=28, fill=WARM)
    d = box(img, (670, bot_row + 10, 1110, bot_row + 68), ["周围地面 z = -0.503 m"], size=27, fill=WHITE)
    d.line([(890, bot_row + 10), (890, bot_row)], fill=WHITE, width=2)

    xa = w - 150
    vee_arrow(d, xa, top_row, bot_row, WHITE, width=4)
    d = box(img, (xa - 250, (top_row + bot_row) // 2 - 46, xa - 40, (top_row + bot_row) // 2 + 46),
            ["0.503 m"], size=38, fill=WHITE)

    out = with_caption(img, 150, [
        f"像素实测：橙带 {h_px} px x 4.7 m / {f:.0f} px = {h_m:.3f} m（模型定义 0.503 m；差 {abs(h_m - 0.503) * 1000:.0f} mm 来自边缘抗锯齿 1~2 px）",
        "橙色整条都在地面之上 —— 场地是凸出地面的高台，不是陷在地底下。",
    ], size=25)
    out = out.convert("RGBA")
    d = ImageDraw.Draw(out)
    d.text((24, h + 96), "", font=font(10))
    out = out.convert("RGB")
    out.save(os.path.join(SHOTS, "debug_colored_wall_annotated.png"))
    print("saved", os.path.join(SHOTS, "debug_colored_wall_annotated.png"))


def annotate_elevation():
    img = Image.open(os.path.join(SHOTS, "debug_colored_elevation.png")).convert("RGBA")
    w, h = img.size
    x_mid = w // 2

    blue = find_band(img, x_mid, "blue")
    orange = find_band(img, x_mid, "orange")

    cam_z = -0.2515
    f = (w / 2) / math.tan(0.65 / 2)
    d_field, d_fence = 25.0 - 4.3, 25.0 - 4.275

    def row_of(z, dist):
        return h / 2 - f * (z - cam_z) / dist

    print(f"[elev] f={f:.1f}px 预测行: 围栏顶 {row_of(0.30, d_fence):.1f} / 台面 {row_of(0, d_field):.1f} / 地面 {row_of(-0.503, d_field):.1f}")
    print(f"[elev] 实测: 蓝带 {blue} 橙带 {orange}")
    h_px = orange[1] - orange[0]
    h_m = h_px * d_field / f
    print(f"[elev] 橙带 {h_px} px -> {h_m:.4f} m（模型 0.5030），1 px = {d_field / f * 1000:.1f} mm")

    d = ImageDraw.Draw(img)
    labels = [
        (blue[0], "围栏顶 z = +0.300 m", 120),
        (orange[0], "台面 z = 0.000 m", 700),
        (orange[1], "周围地面 z = -0.503 m", 1280),
    ]
    for row, text, lx in labels:
        dashed(d, 0, w, row)
        d = box(img, (lx - 10, row - 92, lx + 560, row - 34), [text], size=27, fill=WHITE)
        d.line([(lx + 250, row - 34), (lx + 250, row)], fill=WHITE, width=2)

    d = box(img, (60, 60, 1180, 232), [
        "正立面：台面（橙上边缘）比周围地面（橙下边缘）高 0.503 m",
        f"像素实测：橙带 {h_px} px x {d_field} m / {f:.0f} px = {h_m:.3f} m（模型 0.503 m；此处 1 px = {d_field / f * 1000:.1f} mm）",
        "蓝色带 = 围栏 0.30 m。三层都在地面之上。",
    ], size=26, fill=WHITE)

    dst = os.path.join(SHOTS, "debug_colored_elevation_annotated.png")
    img.convert("RGB").save(dst)
    print("saved", dst)


if __name__ == "__main__":
    annotate_hero()
    annotate_wall()
    annotate_elevation()