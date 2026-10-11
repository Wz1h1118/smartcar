# OpenMV 颜色+形状+罗马数字识别
import sensor, image, time, math

# 初始化摄像头
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 320x240

# 使用自动曝光、自动增益和自动白平衡，不要求启动时对白纸/白墙。
# 增益设上限，避免暗处自动放大过度，把画面和颜色一起冲白。
sensor.set_auto_exposure(True)
sensor.set_auto_gain(True, gain_db_ceiling=12)
sensor.set_auto_whitebal(True)
sensor.skip_frames(time=2500)


from machine import UART
uart = UART(3, 9600) 
def send(msg):
    uart.write(msg+"\r")
# ============ 颜色识别 ============
#
# ---- 2026-10-11：red 这条原来是 (0,54,-22,78,30,70)，把红圆卡认废了 ----
# 症状：屏幕上 `RED-REV / Triangle-none-CD / px2690 box5198 d0.52`。
# 明明是红圆卡，却报 Triangle，而且 d=0.52（三角形才该有的数）。
# 拿那帧逐像素量了一遍，红卡实际是：
#     L 18~46（中位 31）   A 31~52（中位 44）   B 9~33（中位 24）
# 对照老阈值：A、L 都在范围里，**只有 B ≥ 30 这一条在杀人** ——
# 卡上 84% 的红像素 B 都不到 30，直接被判成"不是红"。
#
# 为什么 B 这么低还是红的：这条卡的体积颜色本来就偏暗偏洋红（L 才 31），
# 不是那种鲜红。老阈值是照"鲜红 A≈60 B≈45"定的，对这张卡从根上就不对。
#
# 连环反应：色块只剩零散一小片时，几何特征也会失真，所以先用阈值编辑器确认
# 屏幕上的色块是完整图形，再看 d/r 数值调形状门限。
#
# 新范围按实测的 2%~98% 分位取，两头都留了余量：
#   L 0~60   （实测 18~46，远一点会更暗，亮一点会更亮）
#   A 25~80  （实测 31~52。**下限 25 是关键**：白纸 A≈-13、黄卡 A≈-14、
#             桌面/黑胶布 A≈0，全被这条挡在外面，红黄的分家其实靠是 A 不是 B）
#   B 0~60   （实测 9~33。下限不再卡 30 —— 它原来就是那个凶手）
# 换灯、换地方、换打印机之后认不出红卡，照这个流程重新量一遍，
# 或者用 IDE 的 Tools → Machine Vision → Threshold Editor 拖框看遮罩。
#
# ★ 顺带一句：这一帧里白纸读成了 rgb(191,240,244) —— 明显偏青。
#   2026-10-01 那帧白纸还是 rgb(144,157,157)（基本中性）。
#
# ---- 2026-10-11（同一晚，第6帧）：黄卡量了，量不出来，根因还是白平衡 ----
# 那一帧黄卡占了大半个画面，屏幕上却一个框都没有。不是"阈值太窄"，
# 是这张卡和白纸在 LAB 里**完全重叠**了：
#     黄卡    L 97~99   A -12~-7   B 21~36
#     白纸    L 93~97   A  -9~-4   B 28~32      ← 纸自己被读成了黄的！
#     rgb 采样：白纸 (236,230,170)、卡片 (253,255,206) —— R>G>B，整幅画面都是暖的。
# 真白纸应当 R≈G≈B（B≈0）。现在 B 有 31，说明白平衡又锁歪了，
# 而且方向和上一次（偏青）**正好相反**：上次锁出来发青，这次发黄。
# 现在改为持续自动白平衡，不再要求开机时对白纸/白墙。
# 自动白平衡稳定后，白纸和黄卡的 LAB 间隔会比固定错误白平衡更可靠；
# 如果换到明显不同的灯光，仍需重新检查 yellow 阈值。
#
# 现象补充：颜色那一路认不出黄卡 → 形状那一路就没有 blob 可算，
# 屏幕上连 "YELLOW-PAR" 都不会出现，看着像"黄卡根本没被看见"。
#
# ---- 2026-10-11 夜（第8、9、10帧）：自动白平衡之后重新量 yellow ----
# 白平衡改成全自动之后，"白"确实中性了（手 B 5.8、背景 B -1.1），
# 但黄卡自己的 B 也跟着掉下来。三帧实测，卡面 B 的**最高值**是：
#     第8帧 51     第9帧 67     第10帧 76
# 也就是说原来那条 B>=49 并不是"卡"和"别的东西"之间的界线 ——
# 它整个压在卡身上：卡亮一点就被切掉一半，暗一点就一块都抓不到
# （第8帧整幅 B 最大才 51，49 的门槛只抓到 98 个杂点）。
# 反过来，不该进来的东西 B 都在 10 以下：
#     第8帧量过，卡以外最大 B = 5.9（白卡/桌面上沿/暗背景/蓝椅子）
# 所以这条线该画在 **6 和 21 之间**的空档里。取 25，留 19 点余量。
# 这跟红色是同一个做法：红色在 A 通道上画线（A>=25），黄色在 B 通道上画线。
#
# L 上限和 B 上限也顺手松开：卡面自己也顶到过 L 87（98%分位）、B 76，
# 原来的 89 和 70 都是切在卡身上的。黄色和"白/灰/蓝"分家全靠 B 下限，
# 上限松一点不会放进别的东西。
thresholds = {
    "red":    (0, 60, 25, 80, 0, 60),
    "yellow": (0, 100, -74, 20, 25, 90),
}

color_to_action = {
    "red": "REV",#倒车
    "yellow": "PAR",#侧方
}

shape_to_edge = {
    "Circle": "BC",
    "Triangle": "CD",
    "Rect": "AD"
    
}
num_to_point={
    1:"A",
    3:"B",
    5:"C",
    7:"D",
    
}

MIN_AREA = 1500   # 小于这个像素数的色块丢掉（太小的是噪点）
OVERLAY_TEXT_COLOR = (0, 0, 0)  # 黑字在黄色、红色和白纸背景上都清楚

# ---- 形状识别门限 ----
# 密度 = 色块像素数 / 最小旋转外接矩形面积：
# 三角形约 0.50，圆形约 0.785，正方形约 1.00。
# 圆度用于处理颜色阈值漏掉一圈后，正方形密度下降的情况。
TRIANGLE_DENSITY_MAX = 0.64
RECT_DENSITY_MIN = 0.90
CIRCLE_ROUNDNESS_MIN = 0.86
RECT_LEAK_DENSITY_MIN = 0.82


def _blob_member(blob, name):
    """兼容不同 OpenMV 固件：成员可能是属性，也可能是方法。"""
    value = getattr(blob, name)
    try:
        return value()
    except TypeError:
        return value


def blob_min_density(blob):
    """用最小外接矩形算密度（不受旋转影响）。"""
    corners = _blob_member(blob, "min_corners")

    # 鞋带公式算四边形面积
    area2 = 0
    for i in range(4):
        x1, y1 = corners[i]
        x2, y2 = corners[(i + 1) % 4]
        area2 += x1 * y2 - x2 * y1
    rect_area = abs(area2) / 2.0

    if rect_area <= 0:
        return 0.0, 0.0
    return _blob_member(blob, "pixels") / rect_area, rect_area


def detect_shape(img, blob):
    """用颜色 blob 的几何特征识别形状，不依赖 Hough/AprilTag 检测器。

    这三张卡的色块本身就是实心图形：最小旋转外接矩形的填充率约为
    三角形 0.50、圆形 0.785、正方形 1.00。这样不受旋转影响，也不会因为
    圆的边缘对比度、距离或 OpenMV 型号不同而整段检测失败。
    """
    density, _ = blob_min_density(blob)
    roundness = -1.0
    try:
        roundness = float(_blob_member(blob, "roundness"))
    except Exception:
        pass

    if density < TRIANGLE_DENSITY_MAX:
        shape = "Triangle"
    elif density < RECT_DENSITY_MIN:
        # 透视会把圆拍成椭圆，圆度可能明显低于 0.86；先保留密度在
        # 0.785 附近的圆。只有密度已经偏高、且圆度像方形时才判方形。
        shape = "Rect" if (
            density >= RECT_LEAK_DENSITY_MIN
            and roundness >= 0.0
            and roundness < CIRCLE_ROUNDNESS_MIN
        ) else "Circle"
    else:
        shape = "Rect"

    return shape, "d%.2f r%.2f" % (density, roundness)


# ============ 罗马数字识别 ============
# 数字墨迹的阈值 —— 深蓝，不是黑。注意是 LAB 六元组，不是灰度二值。
#
# 为什么黑色不行（原来就是黑的，吃过亏）：
#   黑和灰在 LAB 里都是 A=0、B=0，只有 L 不一样。也就是说"是不是笔迹"
#   这件事，黑色阈值实际只剩 L 一个维度在管。阴影、深色桌面、卡片边、
#   打印出来的灰标签、笔画的投影 —— 只要够暗，全都会被当成笔画抓走。
#   现象就是满屏碎块，数字认不出来，而代码报的是"背景太暗→用白纸"
#   （reason 里那句话），看着像纸的问题，其实是阈值选错了维度。
#
#
# ---- 2026-10-01 拿实拍帧逐像素标定，把纸上那套理论推翻了 ----
# 原来这条框是照"打印墨色的理想 LAB"定的：RGB(28,40,140) → (22.5, 32.8, -56.4)，
# 当时以为笔迹是 A≈33、B≈-56。**板子上根本不是这样。** 一帧实拍量下来：
#     笔迹核心 (最蓝的那些像素)  LAB ≈ (18, 12.6, -34.9)   rgb(16,41,95)
#     笔迹中位                    LAB ≈ (23,  1.4, -20.1)
#     白纸                        LAB ≈ (64, -4.4,  -1.9)   rgb(144,157,157)
#     桌面/背景                   LAB ≈ (58, -4.0,  +0.5)
# 两个结论：
#   1) 蓝被压扁了。传感器蓝通道本来就弱，墨又吸蓝，实测 B 只有 -20 上下，
#      理想值是 -56，差了快三倍 —— "离边界还有 20.8 个单位"是纸上算的假数。
#   2) 所以 A≥12 这道关**把笔迹自己挡死了**。笔迹的 A 中位数才 1.4，全部 <12；
#      那一帧里 1136 个笔迹像素只有 2 个过得了 A≥12，raw_n=0，
#      屏幕上显示的就是"无暗块 → 看阈值"。这就是"读不到"的直接原因。
#      A 在真机上不分敌我：笔迹 A 中位 1.4、背景 A 中位 -0.2，两堆是重叠的。
#
# 现在这条框里，谁在干活：
#   L   0 ~ 50   : 够暗。白纸 64、背景 58，进不来
#   B -128 ~ -10 : **唯一真正管用的那道**。笔迹中位 -20、核心 -35；
#                  背景里最蓝的像素只到 -6.4（画面右上角一个偏蓝红的东西），
#                  中间是空的。赛道上那条黑色引导线（电工胶布，宽1.8cm）
#                  是中性色 B≈0，离 -10 还有 10 个单位的余量 ——
#                  "换蓝色就是为了甩掉它"这件事，靠的是 B 不是 A。
#   A -40 ~ 70   : 留个范围而已，实际上不分敌我（见上），别再指望它
#
# B 的闸门为什么放 -10 而不是 -30：放 -30 会连大半个笔迹一起筛掉，那就是故障本身。
# 拿坏掉那帧喂进去验证过：
#     框 (0,50, 12,70,-128,-30)  →  0 块                  ← 改之前，什么都找不到
#     框 (0,50,-40,70,-128,-10)  →  1 块 66x67 aspect0.86 ← 改之后，只有那个 V
# 一改就通，而且没有多出别的块。
#
# 换笔/换纸/换灯之后认不出，先看这一行。重新标定的办法：
# IDE 里 Tools → Machine Vision → Threshold Editor，抓一帧，拖框看遮罩，
# 把框拖到只剩笔迹为止，再把六个数抄回来。
# 注意 cards/make_cards.py 里的 NUMBER_BOX 现在**故意和这里不一样**：
# 那边是"选墨色"用的设计目标（必须挑个足够蓝的墨），这边是"认墨色"用的
# 运行闸门（得迁就传感器的蓝通道）。两边都放宽的话，选出来的墨会偏淡。
DARK_LAB     = (0, 50, -40, 70, -128, -10)
COMP_MIN_PIX = 40     # 组件最小像素数（滤小墨点）
COMP_MIN_LEN = 12     # 组件最小长边（滤小墨点）
# 组件最小长宽比 —— 比这个还细的，当"细丝"丢掉。
# 真竖条 I 的长宽比 = 笔画宽 ÷ 字高 = 0.15（就是 cards/make_cards.py 里的
# BAR_RATIO）。0.1 比它低三分之一，留的是"阈值把笔画边缘啃掉一圈"的余量。
# 为什么必须单独加这一道：细丝不满足"太小"—— 它长边够长，COMP_MIN_LEN
# 拦不住，最后还会被算成一根竖条 I。卡片边缘的一道缝、电缆反光、纸上的
# 一道折痕，都会连成这么一条。
#
# ★ 2026-10-01 拿测试台量过（_备份_2026-10-01/test_openmv_port.py）：
#   照真卡比例（BAR_RATIO=0.15）画的那几张"实拍距离"表，加不加这道闸门
#   **一行都不差** —— 对印出来的卡片，I/III/VII 一个都没被误杀，
#   再小到认不出的时候，不加也认不出。
#   但测试台里另有一批用 3px 签字笔画的用例（长宽比只有 0.025，比真卡细
#   6 倍），加上这道闸门会从 152/176 掉到 109/176。那批是**笔写的**，
#   不代表你那张印的卡。要是哪天拿手写卡当备用，把这里降到 0.01。
# 万一出现"I/III 认成 ?"或者"VII 读成 V"（隔得远、光暗，笔画被啃细），
# 第一件事就是把这个数往下调。
COMP_MIN_ASPECT = 0.10
MIN_REL_LEN  = 0.45   # 长边 < 最大块长边×这个 → 当噪点丢掉
THIN_MAX     = 0.35   # aspect<这个 = 竖条(I)；>=这个 = V形
MERGE_FRAC   = 0.03   # 两块最近距离 < 画面宽度×这个 → 合并成一块

# ---- "笔画得长在白纸上"这道闸门 ----
# 症状：认图形（红圆/黄三角）的时候，串口时不时蹦出一个罗马数字。
# 根子不在图形那一路。罗马数字这一路（下面 detect_roman_numeral）是**每3帧
# 无条件跑一次**的，拿的是**整幅画面、没有 ROI**；而 I/II/III 那一支做的事
# 就是**数细条**——数出1根就是I、2根就是II、3根就是III，中间没有任何一步
# 停下来问"这几条到底是不是数字"。桌子边缘、电线、卡片缝、胶布断口，在暗光
# 下都可能留下1~3条**稳定的**深蓝细条；稳定器连着5帧看到同样的结果，就当成
# 数字发一串 P 出去。所以现象是"认图形的时候冒出数字"——两路各跑各的，
# 数字这一路根本没在看卡片。
#
# 数字是印在白卡上的，笔画四周就该是**亮纸**。所以量一下笔画组外圈那一条框的
# 平均亮度，太暗就说明这堆东西不在白卡上，判"?"。
# 亮度按人眼权重算：(3R+6G+B)/10 —— 绿权重最大、蓝最小，和眼睛的观感一致。
# 实拍参考值（你那间屋子的光）：
#     白卡      ≈ 153
#     深色桌面  ≈  72
# 闸门取 90，正好在两者中间，上下各有 20 以上的余量。
#
# 为什么取样只取外圈、不取里面：笔画自己是暗的，混进平均值会把它拖下去，
# 等于自己把自己判掉。只沿外框四条边取样，那里是纯纸。
# 外扩量为什么按**短边**算：数字"III"很长，按长边比例外扩会伸出去太远，
# 直接跑到卡片外面去。按短边 × 0.10 + 3px，保证这一圈还落在卡片上。
#
# 外扩量是拿真卡的几何算出来的，不是拍的：
#   卡 100mm 见方，PATH_W = 84mm，GLYPH_H = 84/1.62 = 51.85mm（见 make_cards.py）
#   → 字高只占卡高的 0.52，上下各有 0.46×字高 的纸；
#     左右各留 8mm = **0.154×字高** —— 这是整张卡上最窄的一圈纸。
#   最宽的 "VII" 正好把左右那圈纸用光（1.62×字高 ≈ 84mm），所以外圈只要
#   比 0.154×字高 多探出去一点，左右两条边就滑到卡外面去了。
#   第一版取 0.25 → 外圈整个落在卡外；第二版 0.10（真卡约 5.2mm）→
#   正着放刚好压线，**卡一斜就出界**；现在取 0.03（真卡约 1.5mm）+ 3px。
#
# ★ 为什么统计量是"四条边里最差那条的亮比例"，而不是"整圈平均亮度"：
#   实拍第6帧就是被平均值放过去的（临时目录 omvtest\stat_sweep*.py，
#   按真卡比例画、拿**真脚本**跑到那一帧上）：
#       外圈上边 34 个点全在白纸上（244），下边 34 个点全在键盘上（47），
#       平均 144 > 90 → 放行 → 在一堆键盘花纹上判出 II，连着 5 帧就发 P2。
#   换成"整圈里亮点的比例"也分不开：第6帧 50%，而真卡斜 45° 放是 38%，
#   两者叠在一起，中间画不出线。
#   分得开的是**位置**：
#       真卡斜着放 → 每条边中间都是纸，只有两头滑到桌面上 → 没有一条边是整条暗的
#       跨在纸/桌分界上 → 有一条边整条趴在键盘上 → 那条边 0%
#   所以取"四条边里最差的那条"。实测（桌面 72、真卡比例）：
#       真卡 0°  III/VII 100%    15° VII 50%    30° VII 36%
#       45°    III 71%  VII 36%  真第6帧 下边 0%   纯桌面 0%
#   门槛 30 就落在这中间：真卡最坏 36%，误报那侧 0%。
#   余量不大，根源是"外接框"本身：斜着放的笔画，它的**外接框四个角**会伸到
#   卡外面去，任何贴着外接框量的画法都躲不掉，只能靠缩小外扩量把损失压到最小。
#
# 有效采样点少于 PAPER_MIN_PTS（卡片顶满画面，一圈大半在画面外）时返回 -1，
# 意思是"这一帧判不了"，**不拦** —— 宁可不报，也别乱报。
PAPER_L_MIN        = 90    # 单个采样点算不算"纸"的亮度线（0~255）
PAPER_PAD_K        = 0.03  # 外扩量 = 笔画组短边 × 这个（+3px，见上）
PAPER_EDGE_MIN     = 30    # 四条边里最差的那条，亮点比例不能低于这个（%）
PAPER_EDGE_MIN_PTS = 4     # 一条边采到少于这么多点 → 这条边不参与评比
PAPER_MIN_PTS      = 8     # 有效采样点少于这个 → 判不了，放行

def _lum(r, g, b):
    """人眼亮度 0~255（绿权重最大、蓝最小）"""
    return (r * 3 + g * 6 + b) / 10.0

def _edge_bright_frac(img, horizontal, fixed, lo, hi, step):
    """量**一条边**上有多少比例的点是"纸"（亮），返回百分比。

    horizontal=True  → 横边：y 固定在 fixed，x 从 lo 走到 hi（上和下）
    horizontal=False → 竖边：x 固定在 fixed，y 从 lo 走到 hi（左和右）

    返回 -1 = 这条边在画面外 / 采到的点太少 → 这条边**没资格作证**，
    不参加"最差边"的评比（比如卡片顶满画面时，左右两条边整个在画面外）。
    """
    n = 0
    bright = 0
    W = img.width()
    H = img.height()
    for t in range(lo, hi + 1, step):
        if horizontal:
            x = t
            y = fixed
        else:
            x = fixed
            y = t
        # 画面外的点自己先挡掉：OpenMV 的 get_pixel 越界是返回 None，
        # 但老固件上对负坐标不一定老实，宁可在这里判一次。
        if x < 0 or y < 0 or x >= W or y >= H:
            continue
        px = img.get_pixel(x, y)
        if px is None:
            continue
        n += 1
        if _lum(px[0], px[1], px[2]) >= PAPER_L_MIN:
            bright += 1
    if n < PAPER_EDGE_MIN_PTS:
        return -1.0
    return 100.0 * bright / n

def strokes_on_paper(img, comps):
    """量笔画组外圈那一条"纸边"，返回**四条边里最差的那条**的亮点比例（%）。

    返回 -1 = 四条边都量不了，判不了（调用方放行，别拦）。
    只在四条边上取点，不从中间穿 —— 笔画自己是暗的。
    为什么是"最差的一条边"而不是整圈平均，见上面常量注释里第6帧的账。
    """
    # 1) 所有组件外接框的并集
    x0, y0 = 99999, 99999
    x1, y1 = -1, -1
    for c in comps:
        r = c['rect']
        if r[0] < x0: x0 = r[0]
        if r[1] < y0: y0 = r[1]
        if r[0] + r[2] > x1: x1 = r[0] + r[2]
        if r[1] + r[3] > y1: y1 = r[1] + r[3]

    bw = x1 - x0
    bh = y1 - y0
    pad = int(min(bw, bh) * PAPER_PAD_K) + 3   # 0.03×短边 + 3px，理由见常量注释
    left, right = x0 - pad, x1 + pad
    top, bot = y0 - pad, y1 + pad

    # 2) 四条边各量一次。步长取 pad 的一半：pad 小的时候也能取满一圈，
    #    pad 大的时候不至于一帧取上千个点（get_pixel 是逐点读的，慢）。
    step = pad // 2 + 2
    f_top = _edge_bright_frac(img, True, top, left, right, step)
    f_bot = _edge_bright_frac(img, True, bot, left, right, step)
    f_lft = _edge_bright_frac(img, False, left, top, bot, step)
    f_rgt = _edge_bright_frac(img, False, right, top, bot, step)

    # 3) 取最差的那条。量不了的边记 101（比任何真实比例都大 = 不拖后腿）；
    #    四条都量不了 → 最差还是 101 → 返回 -1，交给调用方放行。
    if f_top < 0:
        f_top = 101.0
    if f_bot < 0:
        f_bot = 101.0
    if f_lft < 0:
        f_lft = 101.0
    if f_rgt < 0:
        f_rgt = 101.0
    worst = min(f_top, f_bot, f_lft, f_rgt)
    if worst > 100.0:
        return -1
    return worst

def pts_min_dist(p1, p2):
    """两组点之间的最短距离（纯python，不用numpy）"""
    best = 99999
    for a in p1:
        ax = a[0]
        ay = a[1]
        for b in p2:
            dx = ax - b[0]
            dy = ay - b[1]
            d = math.sqrt(dx * dx + dy * dy)
            if d < best:
                best = d
    return best

def pts_aspect(pts):
    """用一组点算长宽比（PCA主成分分析，旋转不变）

    idea：点沿一条线排开 → 主轴长、次轴短 → 细长（aspect小）
          点铺成一个面   → 两个轴差不多   → 方正（aspect大）

    做法：算这堆点的协方差矩阵，求两个特征值，
          小的开根号 / 大的开根号，就是长短边之比。
    和 cv2.minAreaRect 算出来的结果几乎一样，但纯python就能跑。
    """
    n = len(pts)
    if n < 2:
        return 0.0

    # 1) 重心
    mx = 0.0
    my = 0.0
    for p in pts:
        mx += p[0]
        my += p[1]
    mx /= n
    my /= n

    # 2) 协方差矩阵的3个元素
    sxx = 0.0
    syy = 0.0
    sxy = 0.0
    for p in pts:
        dx = p[0] - mx
        dy = p[1] - my
        sxx += dx * dx
        syy += dy * dy
        sxy += dx * dy
    sxx /= n
    syy /= n
    sxy /= n

    # 3) 2x2对称矩阵的两个特征值（有公式，不用迭代）
    tr = sxx + syy
    det = sxx * syy - sxy * sxy
    disc = tr * tr / 4.0 - det
    if disc < 0:
        disc = 0
    disc = math.sqrt(disc)
    l_big = tr / 2.0 + disc
    l_small = tr / 2.0 - disc

    if l_big <= 0.0001:
        return 0.0
    if l_small < 0:
        l_small = 0
    return math.sqrt(l_small / l_big)

def detect_roman_numeral(img):
    """罗马数字识别 - 按连通块判断（和 test_roman_logic.py 完全同一套逻辑）

    思路：罗马数字由「竖条」和「V形」拼成
      竖条 → 细长（aspect ≈ 0.15 = 笔画宽÷字高，见 BAR_RATIO）
      V形  → 铺开成一个面（aspect约0.7~1.0）
    数个数 + 看左右顺序 → 得出是几

    关键：先合并靠得近的块！
    实拍时笔画常会断开，不合并的话一个V会被拆成"细碎片+V主体"→误判成IV

    用「角点」而不是「外接框」判距离：倾斜的细笔画外接框很宽，
    会和邻居的框重叠，导致本来分开的笔画被误合并。

    返回 (罗马数字, 数值, 组件列表, 调试信息)
    """
    # 找深蓝笔迹的连通块 —— 直接在彩色图上找，用 LAB 六元组。
    # 别退回 to_grayscale：灰度只有一个维度，红只贡献 30% 亮度、蓝只占 11%，
    # 鲜红卡片灰度才 76、鲜蓝才 29 —— 都会掉进"暗"这个区间里。
    # LAB 多出 A/B 两个维度，靠 B 的正负就能把蓝色和灰/黑/红一刀切开。
    # 关键：merge=False！I 和 V 必须是分开的两块，不能被OpenMV自己合并掉
    blobs = img.find_blobs(
        [DARK_LAB],
        pixels_threshold=COMP_MIN_PIX,
        area_threshold=COMP_MIN_PIX,
        merge=False,
        x_stride=2, y_stride=2      # 步长2快4倍；笔画粗，不会漏
    )
    raw_n = len(blobs)

    # ---- 第一步：每个块取最小外接矩形的4个角点 ----
    items = []
    for b in blobs:
        r = b.rect()
        if max(r[2], r[3]) < COMP_MIN_LEN:     # 太小，当噪点
            continue
        c = b.min_corners()
        items.append({'pts': [c[0], c[1], c[2], c[3]], 'rect': r})
    keep_n = len(items)

    # ---- 第二步：把靠得近的块合并（反复合并直到稳定）----
    merge_dist = img.width() * MERGE_FRAC
    changed = True
    while changed:
        changed = False
        for i in range(len(items)):
            if changed:
                break
            for j in range(i + 1, len(items)):
                if pts_min_dist(items[i]['pts'], items[j]['pts']) <= merge_dist:
                    items[i]['pts'] = items[i]['pts'] + items[j]['pts']
                    # 外接框取并集
                    r1 = items[i]['rect']
                    r2 = items[j]['rect']
                    x1 = min(r1[0], r2[0])
                    y1 = min(r1[1], r2[1])
                    x2 = max(r1[0] + r1[2], r2[0] + r2[2])
                    y2 = max(r1[1] + r1[3], r2[1] + r2[3])
                    items[i]['rect'] = (x1, y1, x2 - x1, y2 - y1)
                    items.pop(j)
                    changed = True
                    break
    mrg_n = len(items)

    # ---- 第三步：每个合并后的块算长宽比 + 长边 ----
    comps = []
    for it in items:
        aspect = pts_aspect(it['pts'])
        # 又细又长的当噪点 —— 原来只挡 aspect<=0（退化成一条线段、算不出比例），
        # 结果一条细丝照样能过：长边够长，COMP_MIN_LEN 拦不住它，
        # 到了下面还会被算成竖条 I。现在按长宽比拦，真竖条是 0.15，见常量注释。
        if aspect <= COMP_MIN_ASPECT:
            continue
        r = it['rect']
        sx = 0.0
        for p in it['pts']:
            sx += p[0]
        comps.append({'x': sx / len(it['pts']),
                      'aspect': aspect,
                      'long': max(r[2], r[3]),
                      'rect': r})

    comps.sort(key=lambda c: c['x'])   # 从左到右排序

    # ---- 第四步：按"大小"再筛一次噪点 ----
    # 坑：小墨点的最小外接矩形就是个小正方形 → 长宽比≈1 → 会被当成V形！
    # 真实笔画都差不多高，比最大的那块短很多的，一定是噪点。
    before_n = len(comps)
    if comps:
        biggest = 0
        for c in comps:
            if c['long'] > biggest:
                biggest = c['long']
        keep = []
        for c in comps:
            if c['long'] >= biggest * MIN_REL_LEN:
                keep.append(c)
        comps = keep

    # ---- 第五步：量一下这堆笔画是不是长在白卡上 ----
    # 返回的是"外圈最差那条边上纸点的比例（%）"，门槛和取样规则见上面常量那一段。
    # 返回 -1 = 四条边都量不了，判不了，不拦。
    paper_pct = -1.0
    if comps:
        paper_pct = strokes_on_paper(img, comps)

    thin = [c for c in comps if c['aspect'] < THIN_MAX]    # 竖条
    wide = [c for c in comps if c['aspect'] >= THIN_MAX]   # V形

    n_thin = len(thin)
    n_wide = len(wide)

    # p 后面那个数 = 外圈**最差那条边**上"纸"点的比例（%），调 PAPER_EDGE_MIN 看它。
    # 认卡的时候（卡片放在桌上、正着或斜着）应该是 100 或接近 100；
    # 笔画长在桌面/键盘上 → 会有一条边贴地，那个数直接掉到 0。
    # 显示 "--" = 四条边都量不了（卡片顶满画面，一圈大半在画面外），
    # 这道闸门这一帧没生效。
    if paper_pct < 0:
        p_str = "--"
    else:
        p_str = "%.0f%%" % paper_pct
    debug = "r%d l%d m%d c%d k%d p%s" % (raw_n, keep_n, mrg_n, before_n, len(comps), p_str)

    # 画面里有多少比例的像素是"暗"的
    dark_pct = 0
    if raw_n:
        dark_px = 0
        for b in blobs:
            dark_px += b.pixels()
        dark_pct = dark_px * 100 // (img.width() * img.height())

    # 板子自己诊断：卡在哪一步，就直接告诉你要改什么
    if raw_n == 0:
        reason = "无暗块 → 看阈值"
    elif keep_n == 0:
        reason = "笔画太小 → 摄像头拿近"
    elif len(comps) == 0:
        reason = "太细/太小被当噪点 → 拿近"
    elif dark_pct > 25:
        reason = "背景太暗%d%% → 用白纸" % dark_pct
    else:
        reason = "%d细%d方 暗%d%%" % (n_thin, n_wide, dark_pct)

    # ---- 闸门：这几条笔画，四周得是白纸 ----
    # 位置故意放在**判数字之前** —— 只要"是不是长在白卡上"这一问没通过，
    # 后面数出几根细条都不作数。这条闸门专治"认图形的时候冒出罗马数字"：
    # 那些误报的细条来自桌面/电线/胶布，四周是暗的，在这里就被拦成 "?"。
    # paper < 0（四条边都量不了）时整段跳过 —— 卡片顶满画面那一帧照常识别。
    if comps and paper_pct >= 0 and paper_pct < PAPER_EDGE_MIN:
        return "?", 0, comps, debug, "外圈最差那条边只有%.0f%%是纸 → 不是白卡" % paper_pct

    # ---- 只有竖条：I / II / III ----
    if n_wide == 0:
        if n_thin == 1:
            return "I", 1, comps, debug, reason
        elif n_thin == 2:
            return "II", 2, comps, debug, reason
        elif n_thin == 3:
            return "III", 3, comps, debug, reason
        return "?", 0, comps, debug, reason

    # ---- 只有 V ----
    if n_wide == 1 and n_thin == 0:
        return "V", 5, comps, debug, reason

    # ---- V + 两根竖条 = VII(7) ----
    # 注意：VII 是唯一"1个V + 2根竖条"的情况，不用比左右
    if n_wide == 1 and n_thin == 2:
        return "VII", 7, comps, debug, reason

    # ---- V + 一根竖条：看谁在左 ----
    if n_wide == 1 and n_thin == 1:
        if thin[0]['x'] < wide[0]['x']:
            return "IV", 4, comps, debug, reason    # I在左，V在右
        else:
            return "VI", 6, comps, debug, reason    # V在左，I在右

    return "?", 0, comps, debug, reason

# ============ 结果稳定器 ============
# 画面里随便几条线都会猜出结果，所以要连续N帧一样才算数
ROMAN_WINDOW = 6       # 滑窗大小
ROMAN_MIN_VOTES = 4    # 窗内至少4票才算稳定
_roman_hist = []       # 滑窗:每帧一个罗马数字，认不出填 None
_roman_stable = "?"    # 投票出来的稳定结果
_reported = None       # 上一次已经报告过的结果
_roman_sent_ms = 0     # 上一次发罗马数字的时间(用于周期性重发)
ROMAN_VALUE = {"I": 1, "III": 3, "V": 5, "VII": 7}

# 同一个结果最长多久重发一次(ms)。
# 原来只发"和上次不一样"的,同一张卡放第二次就不发了 —— 单片机一旦漏掉第一帧
# (切模式中/刚上电/丢字节),就必须换一张卡才能恢复,现场表现就是
# "识别到了但屏幕不显示"。改成周期性重发后,漏帧会在 0.4 秒内自己补上。
# 9600 波特率下一条消息才 7 字节(约 7ms),400ms 一条完全不会拥堵。
RESEND_MS = 400

ROMAN_EVERY = 3        # 每3帧才跑一次罗马数字（太慢的话就加大）
_frame_no = 0
_roman = "?"
_value = 0
_comps = []
_debug = ""
_reason = ""
_printed_roman = None  # 上一次打过串口的**结论**（是数字，不是 _debug 里的块数）

# ---- 滑窗投票（两路共用）----
# 为什么不用"连续N帧完全相同"：颜色和形状在临界点上会逐帧抖（圆↔方来回跳），
# 一抖计数就归零，永远凑不满N帧 —— 症状是"屏幕上明明认得出来，单片机一条都收不到"。
# 改成"最近W帧里出现次数最多、票数够就算数"，偶尔抖一下不影响。
def _vote(hist):
    """返回(票数最多的结果, 票数)。None 表示这一帧没结果，不参与投票。"""
    best = None
    best_n = 0
    for cand in hist:
        if cand is None:
            continue
        n = hist.count(cand)
        if n > best_n:
            best_n = n
            best = cand
    return best, best_n

# ---- 颜色/形状那一路的稳定器 ----
CARD_WINDOW = 6          # 滑窗大小(最近6帧)
CARD_MIN_VOTES = 4       # 窗内至少4票才算稳定
_card_hist = []          # 滑窗:每帧一个 (color, action, shape, edge)，没看到卡片填 None
_card_stable = None      # 投票出来的稳定结果
_reported_card = None    # 上一次已经发出去过的卡片
_card_sent_ms = 0        # 上一次发车库指令的时间(用于同结果周期性重发)
_last_shape_err = None   # 上一次报过的形状检测报错（同一个错只打一次串口，别刷屏）

# ---- 屏幕顶上那个调试黑框的开关 ----
# 黑框里装的**全是罗马数字**的东西（r/l/m/c/k 计数、_reason），
# 调颜色和形状的时候一点用没有，还挡住画面最上面 50 个像素。
# 所以做成开关而不是删掉 —— 以后调罗马数字的时候改回 True 就回来了。
SHOW_DEBUG = False

# ============ 主循环 ============
while True:
    img = sensor.snapshot()
    _frame_no += 1

    # --- 颜色+形状识别（每帧都做，这个快）---
    # 分两步走，别合并：
    #   第一步（这个 for 循环）：只"算" —— 算出这一帧看到的是哪张卡片，存进 _frame_card
    #   第二步（循环外面）：只"判" —— 攒够帧数、确认没发过，才发出去
    # 原来是把 print 写在循环体里"边算边发"，那样没法做去重，
    # 只要画面里有色块就每帧吐一条，一秒 20 条一模一样的数据。
    _frame_card = None      # 这一帧认出来的卡片，None = 这一帧没看到
    _frame_card_px = 0      # 候选里目前最大的像素数，用来挑"最可信的那个"

    for color_name, threshold in thresholds.items():
        blobs = img.find_blobs(
            [threshold],
            pixels_threshold=MIN_AREA,   # 滤掉小噪点
            area_threshold=MIN_AREA,
            merge=True
        )

        if not blobs:
            continue

        # 只取最大的色块（画面里一张卡片就一个）
        b = max(blobs, key=lambda x: x.pixels())

        # 形状检测保留 try，防止不同固件的 blob 几何成员缺失时主循环停止。
        try:
            shape, _info = detect_shape(img, b)
        except Exception as e:
            shape = "?"
            # 把异常的头几个字**贴到屏幕上**，不只打串口。
            # 屏幕上会显示错误原因，方便发现固件缺少某个几何成员。
            _info = "e:" + str(e)[:16]
            # 出错也别每帧刷串口 —— 同一个错误只报一次
            # （跟罗马数字那个"同一个数字不重复输出"是同一个道理）
            if str(e) != _last_shape_err:
                _last_shape_err = str(e)
                print("形状检测出错: %s" % str(e))

        # 密度和圆度同时显示，方便现场调阈值。
        _dens, _ra = blob_min_density(b)

        action = color_to_action.get(color_name, "Unknown")
        edge   = shape_to_edge.get(shape, "Unknown")

        img.draw_rectangle(b.rect(), color=(255, 255, 255))
        img.draw_cross(b.cx(), b.cy(), color=(255, 255, 255))

        # 屏上显示（只是给人看的，不影响串口）
        info1 = "%s-%s" % (color_name.upper(), action)
        # 形状后面的 d/r 是填充率和圆度：d 主要分三角/圆/方，r 辅助分圆/方。
        info2 = "%s-%s-%s" % (shape, _info, edge)

        # 青色这行显示色块完整度：d 越接近理论值，颜色阈值越合适。
        # 参考值：三角形约 0.50，圆形约 0.785，正方形约 1.00。
        info3 = "px%d/box%d d%.2f" % (b.pixels(), int(_ra), _dens)

        img.draw_string(b.x(), b.y()-25, info1, color=OVERLAY_TEXT_COLOR, scale=1.5)
        img.draw_string(b.x(), b.y()-10, info2, color=OVERLAY_TEXT_COLOR, scale=1.2)
        img.draw_string(b.x(), b.y()+5, info3, color=OVERLAY_TEXT_COLOR, scale=1.2)

        # 画框照旧（画的是"这一帧看到的"，跟稳不稳定无关），但这里不再发送。
        # red 和 yellow 两个阈值都可能命中，取像素更多的那张 ——
        # 不赌字典的遍历顺序（"后一个覆盖前一个"是查不出来的坑）
        if b.pixels() > _frame_card_px:
            _frame_card_px = b.pixels()
            # 元组里**只放结论，不放 b.pixels()**。
            # 坑：px 是每帧都在抖的测量值（曝光还没锁死的时候抖得尤其厉害），
            # 把它塞进来，滑窗投票就永远投不出同一个结果（每帧都不同）——
            # 症状是颜色/形状这一路**一条串口都不发**，而屏幕上画的框、
            # 显示的字一切正常，根本看不出问题出在哪。
            # 选出"最大那张卡"仍然看 px，那是在上面比较，不放进元组。
            _frame_card = (color_name, action, shape, edge)

    # ---- 稳定器：最近 CARD_WINDOW 帧投票 ----
    _card_hist.append(_frame_card)
    if len(_card_hist) > CARD_WINDOW:
        _card_hist.pop(0)
    _card_stable, _card_votes = _vote(_card_hist)

    if _card_stable is None:
        _reported_card = None      # 卡片拿开了:清去重标记,同一张卡放回来能立刻重发

    # 票数够就发:结果变了立刻发,没变则每 RESEND_MS 重发一次
    if (_card_stable is not None
            and _card_votes >= CARD_MIN_VOTES
            and (_card_stable != _reported_card
                 or time.ticks_diff(time.ticks_ms(), _card_sent_ms) >= RESEND_MS)):
        _reported_card = _card_stable
        _card_sent_ms = time.ticks_ms()
        _cn, _gr, _sh, _ps = _card_stable
        print("Color:%s Garage:%s Shape:%s Pos:%s px:%d"
              % (_cn, _gr, _sh, _ps, _frame_card_px))
        # _gr 是动作(REV/PAR)，_ps 是边(BC/CD/AD)。上面那两张表已经查过了，
        # 这里直接拼 —— 再查一遍就得重新想"哪个变量是哪个"，
        # 一不留神两张表就用反了（发出去变成 GNone,None）。
        #
        # 形状没认出来就**不发**。这时候 _ps 是查表的兜底值 "Unknown"，
        # 发出去是 "GUnknown,REV" —— 协议里 _ps 只有两个字符的位置，
        # 多出来的字符把后面的字段全顶歪，单片机那边解析出一堆错。
        # 宁可这一帧不发，也不能发一条协议对不上的数据。
        # （罗马数字那一路对 "?" 也是这么办的，见下面 send("P...") 上面那行。）
        if _ps != "Unknown":
            send("G%s,%s" % (_ps, _gr))
        else:
            print("[不发] 形状没认出来（%s），等认出来再说" % _sh)

    # --- 罗马数字识别（每3帧做一次，省时间）---
    if _frame_no % ROMAN_EVERY == 0:
        # 用try包起来！否则一旦出错，整个程序当场死掉，
        # 屏幕上什么都不显示，你根本不知道发生了什么
        try:
            _roman, _value, _comps, _debug, _reason = detect_roman_numeral(img)
        except Exception as e:
            _roman = "ERR"
            _value = 0
            _comps = []
            _debug = "ERR"
            _reason = str(e)[:42]
            print("!!! 罗马数字识别出错: %s" % e)

    # 把检测到的每个组件框出来：绿框=竖条(I)  蓝框=V形
    for c in _comps:
        color_box = (0, 255, 0) if c['aspect'] < THIN_MAX else (255, 0, 0)
        r = c['rect']
        img.draw_rectangle(r, color=color_box)
        img.draw_string(r[0], r[1] - 10,
                        "%.2f" % c['aspect'], color=color_box, scale=1.0)

    # 稳定化：最近 ROMAN_WINDOW 帧投票
    _roman_hist.append(None if _roman == "?" else _roman)
    if len(_roman_hist) > ROMAN_WINDOW:
        _roman_hist.pop(0)
    _roman_stable, _roman_votes = _vote(_roman_hist)

    if _roman_stable is None:
        _reported = None           # 数字拿开了:清去重标记

    # 罗马数字的调试黑框 —— 开关在文件上面（SHOW_DEBUG）。
    # 先铺一条黑底是因为字是黄的，白纸上直接画看不见。
    #   raw/keep/comps = 原始块数 / 过完滤噪的块数 / 合并后的组件数
    #   如果 raw=0  → 阈值DARK_LAB不对（没找到笔迹：L 放太低，或 A/B 卡太窄）
    #   如果 keep=0 → COMP_MIN_PIX/COMP_MIN_LEN 太严（笔画太小被判成噪点）
    #   如果 comps=0→ 角点算不出长宽比
    #   r/l/m/c/k 后面的都是**块数**，一路筛下来剩多少（见 detect_roman_numeral）
    #   p 后面是**外圈最差那条边上"纸"点的比例**：认卡的时候该有 100%，
    #   有一条边整条贴在桌面/键盘上（就是误报的那种）会掉到 30% 以下，
    #   这时候判 "?"（"外圈最差那条边只有xx%是纸 → 不是白卡"）——
    #   这是挡"认图形时冒数字"的那道闸。
    #   显示 -- = 这帧判不了（卡片顶满画面），闸门没生效。
    if SHOW_DEBUG:
        img.draw_rectangle((0, 0, img.width(), 50), color=(0, 0, 0), fill=True)
        img.draw_string(3, 2, "%s R:%s votes:%d" % (_debug, _roman, _roman_votes),
                        color=(255, 255, 0), scale=1.3)
        img.draw_string(3, 17, _reason, color=(255, 200, 0), scale=1.3)

    # 调试信息只在**结论变了**的时候打一次串口 —— 屏幕看不见的话，看IDE下面的串口输出。
    #
    # 判据必须是 _roman（数字本身），不能是 _debug。坑就在这儿：
    # _debug 里装的是 r/l/m/k 这些**块数**，画面里飘过一个小噪点、或者阈值
    # 在笔画边缘上啃掉一个像素，块数就变一下，字符串跟着变 —— 于是同一个
    # "-> V" 一秒能刷十几行。结论没变，信息量就是零，纯粹刷屏。
    # 原来那句 `if _debug != _last_debug` 就是踩了这个坑。
    #
    # 想看块数实时怎么变（调阈值的时候有用）：把上面 SHOW_DEBUG 打开，
    # 那些数是**每一帧**画在屏幕上的，比串口还快，而且不刷串口。
    if _roman != _printed_roman:
        _printed_roman = _roman
        print("ROMAN DEBUG: %s  -> %s" % (_debug, _roman))

    # 票数够就发:结果变了立刻发,没变则每 RESEND_MS 重发一次
    if _roman_stable is not None and _roman_votes >= ROMAN_MIN_VOTES:
        _now = time.ticks_ms()
        if (_roman_stable != _reported
                or time.ticks_diff(_now, _roman_sent_ms) >= RESEND_MS):
            _reported = _roman_stable
            _roman_sent_ms = _now
            _value = ROMAN_VALUE.get(_roman_stable, 0)
            img.draw_string(5, 25, "OK: %s = %d" % (_roman_stable, _value),
                            color=OVERLAY_TEXT_COLOR, scale=2)
            print("Roman:%s Value:%d" % (_roman_stable, _value))
            if _value in num_to_point:
                send("P%s" % (num_to_point.get(_value)))
    time.sleep_ms(50)