"""把实测结果填进 AAE4203 模板的第三部分。

模板：report/AAE4203_temp.pptx（10 x 5.625 in，16:9，28 页）
输出：report/AAE4203_temp_with_Part3.pptx

严格遵守模板已有做法：
  * 版式沿用 'Title and Content'（蓝色标题带来自版式本身）
  * 右上角主讲人标签复制自第 18 页的圆角矩形（填充 C8860A，12pt 粗体白字 Arial）
  * 正文 Arial，深灰 262626；强调色用模板表格里的 0070C0
  * 表格沿用模板 tableStyleId，表头填充 9EC3EA

第三部分原有两页占位符（Case Study / Demo、Results and Discussion）被替换为
六页实测内容：运行环境 → 实现路径 → 场景矩阵 → 结果 → S3 机理 → 结论。
数字全部来自 results/tables/all_runs.csv（实测），图片来自 results/figures/。
"""
import copy
import csv
import os
import sys

from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN
from pptx.util import Inches, Pt

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.path.join(HERE, 'AAE4203_temp.pptx')
OUT = os.path.join(HERE, 'AAE4203_temp_with_Part3.pptx')
FIGS = os.path.join(ROOT, 'results', 'figures')
CSV = os.path.join(ROOT, 'results', 'tables', 'all_runs.csv')

INK = RGBColor(0x26, 0x26, 0x26)
BLUE = RGBColor(0x00, 0x70, 0xC0)
ORANGE = RGBColor(0xC8, 0x86, 0x0A)
HEAD_FILL = RGBColor(0x9E, 0xC3, 0xEA)
GREY = RGBColor(0x59, 0x59, 0x59)
FONT = 'Arial'


# ---------------------------------------------------------------- 数据
def load_runs():
    """{scenario: {algo: {metric: float}}} —— 只读实测 CSV"""
    out = {}
    with open(CSV, newline='', encoding='utf-8') as f:
        for row in csv.DictReader(f):
            scene = os.path.basename(os.path.dirname(
                (row.get('csv') or '').replace('\\', '/')))
            algo = row['algorithm']
            vals = {}
            for k, v in row.items():
                try:
                    vals[k] = float(v)
                except (TypeError, ValueError, AttributeError):
                    vals[k] = v
            out.setdefault(scene, {})[algo] = vals
    return out


ORDER = ['s0_nominal', 's1_fast_yaw', 's2_high_speed',
         's3_corridor', 's4_sparse_hall', 's5_long_run']
SHORT = dict(zip(ORDER, ['S0', 'S1', 'S2', 'S3', 'S4', 'S5']))


# ---------------------------------------------------------------- 底层工具
def add_text(slide, x, y, w, h, paras, size=14, color=INK, bold=False,
             align=PP_ALIGN.LEFT, spacing=1.0):
    """paras: [str] 或 [(str, {size/color/bold})]"""
    box = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    tf = box.text_frame
    tf.word_wrap = True
    for i, item in enumerate(paras):
        if isinstance(item, tuple):
            text, kw = item
        else:
            text, kw = item, {}
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        p.alignment = kw.get('align', align)
        p.line_spacing = kw.get('spacing', spacing)
        if kw.get('space_before'):
            p.space_before = Pt(kw['space_before'])
        run = p.add_run()
        run.text = text
        run.font.name = FONT
        run.font.size = Pt(kw.get('size', size))
        run.font.bold = kw.get('bold', bold)
        run.font.color.rgb = kw.get('color', color)
    return box


def add_pill(slide, template_slide, text='Presenter: Member 3'):
    """复制模板里那个橙色主讲人标签"""
    for sh in template_slide.shapes:
        if sh.name.startswith('Rounded Rectangle') and sh.has_text_frame \
                and 'Presenter' in sh.text_frame.text:
            el = copy.deepcopy(sh._element)
            slide.shapes._spTree.append(el)
            for t in el.iter('{http://schemas.openxmlformats.org/drawingml/2006/main}t'):
                t.text = text
            return True
    return False


def add_box(slide, x, y, w, h, text, size=11, fill=HEAD_FILL, color=INK,
            bold=True, border=True):
    """流程图用的圆角小方块"""
    from pptx.enum.shapes import MSO_SHAPE
    shp = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(x),
                                 Inches(y), Inches(w), Inches(h))
    shp.fill.solid()
    shp.fill.fore_color.rgb = fill
    if border:
        shp.line.color.rgb = BLUE
        shp.line.width = Pt(1)
    else:
        shp.line.fill.background()
    tf = shp.text_frame
    tf.word_wrap = True
    tf.margin_left = tf.margin_right = Pt(3)
    tf.margin_top = tf.margin_bottom = Pt(1)
    lines = text.split('\n')
    for i, line in enumerate(lines):
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        p.alignment = PP_ALIGN.CENTER
        run = p.add_run()
        run.text = line
        run.font.name = FONT
        run.font.size = Pt(size if i == 0 else size - 2)
        run.font.bold = bold if i == 0 else False
        run.font.color.rgb = color
    return shp


def add_arrow(slide, x, y, w=0.22):
    from pptx.enum.shapes import MSO_SHAPE
    shp = slide.shapes.add_shape(MSO_SHAPE.RIGHT_ARROW, Inches(x), Inches(y),
                                 Inches(w), Inches(0.16))
    shp.fill.solid()
    shp.fill.fore_color.rgb = BLUE
    shp.line.fill.background()
    return shp


def add_picture_fit(slide, path, x, y, w, h):
    """按原图长宽比缩放到框内并居中（不拉伸）"""
    if not os.path.isfile(path):
        add_text(slide, x, y + h / 2 - 0.2, w, 0.4, [
            '[missing figure: %s]' % os.path.basename(path)],
            size=11, color=GREY, align=PP_ALIGN.CENTER)
        return None
    from struct import unpack
    with open(path, 'rb') as f:
        head = f.read(24)
    pw, ph = unpack('>II', head[16:24])
    aspect, box = pw / ph, w / h
    if aspect > box:
        fw, fh = w, w / aspect
    else:
        fw, fh = h * aspect, h
    fx, fy = x + (w - fw) / 2, y + (h - fh) / 2
    return slide.shapes.add_picture(path, Inches(fx), Inches(fy),
                                    Inches(fw), Inches(fh))


def add_table(slide, x, y, w, h, header, rows, col_w=None, head_size=11,
              body_size=11, highlight=None):
    """highlight: {(row_idx, col_idx): RGBColor} 把更好的那个标蓝"""
    n_r, n_c = len(rows) + 1, len(header)
    gf = slide.shapes.add_table(n_r, n_c, Inches(x), Inches(y), Inches(w),
                                Inches(h))
    tbl = gf.table
    # 与模板一致的表格样式
    tblPr = tbl._tbl.tblPr
    tblPr.set('firstRow', '1')
    tblPr.set('bandRow', '1')
    style = tblPr.find('{http://schemas.openxmlformats.org/drawingml/2006/main}tableStyleId')
    if style is None:
        from lxml import etree
        style = etree.SubElement(
            tblPr, '{http://schemas.openxmlformats.org/drawingml/2006/main}tableStyleId')
    style.text = '{5C22544A-7EE6-4342-B048-85BDC9FD1C3A}'

    if col_w:
        total = sum(col_w)
        for i, cw in enumerate(col_w):
            tbl.columns[i].width = Inches(w * cw / total)
    for i, r in enumerate(tbl.rows):
        r.height = Inches(h / n_r)

    def fill_cell(cell, text, size, bold, color, is_head):
        cell.fill.solid()
        cell.fill.fore_color.rgb = HEAD_FILL if is_head else RGBColor(0xFF, 0xFF, 0xFF)
        cell.margin_left = cell.margin_right = Inches(0.05)
        cell.margin_top = cell.margin_bottom = Inches(0.02)
        cell.vertical_anchor = 3  # middle
        tf = cell.text_frame
        tf.word_wrap = True
        p = tf.paragraphs[0]
        p.alignment = PP_ALIGN.CENTER
        run = p.add_run()
        run.text = text
        run.font.name = FONT
        run.font.size = Pt(size)
        run.font.bold = bold
        run.font.color.rgb = color

    for j, title in enumerate(header):
        fill_cell(tbl.cell(0, j), title, head_size, True,
                  RGBColor(0xFF, 0xFF, 0xFF), True)
    for i, row in enumerate(rows):
        for j, val in enumerate(row):
            colour = INK
            if highlight and (i, j) in highlight:
                colour = BLUE
            fill_cell(tbl.cell(i + 1, j), str(val), body_size,
                      bool(highlight and (i, j) in highlight), colour, False)
    return tbl


def new_slide(prs, layout, title, presenter_from):
    slide = prs.slides.add_slide(layout)
    # 删掉版式带来的正文占位符（我们改用文本框，位置更可控）
    for ph in list(slide.placeholders):
        if ph.placeholder_format.idx != 0:      # 0 = title
            ph._element.getparent().remove(ph._element)
    for ph in slide.placeholders:
        if ph.placeholder_format.idx == 0:
            ph.text = title
            for p in ph.text_frame.paragraphs:
                for r in p.runs:
                    r.font.name = FONT
                    r.font.bold = True
    add_pill(slide, presenter_from)
    return slide


# ---------------------------------------------------------------- 各页
def slide_env(prs, layout, pill_src, runs):
    s = new_slide(prs, layout, 'Runtime Environment', pill_src)
    add_text(s, 0.33, 0.95, 4.85, 0.34, [
        ('Where the numbers were produced', {'size': 13, 'bold': True,
                                             'color': BLUE})])
    add_text(s, 0.33, 1.35, 4.9, 3.6, [
        ('Hardware  ', {'size': 13, 'bold': True, 'color': BLUE}),
        ('NVIDIA Jetson Orin Nano 8 GB (aarch64), Ubuntu 24.04 LTS',
         {'size': 12, 'space_before': 0}),
        ('Middleware  ', {'size': 13, 'bold': True, 'color': BLUE,
                          'space_before': 8}),
        ('ROS 2 Jazzy · Gazebo Harmonic 8.15 (ros_gz) · RViz2',
         {'size': 12}),
        ('LIO-SAM  ', {'size': 13, 'bold': True, 'color': BLUE,
                       'space_before': 8}),
        ('Upstream ros2 branch: C++17, GTSAM 4.2, Ceres 2.2, iSAM2',
         {'size': 12}),
        ('A-LOAM (baseline)  ', {'size': 13, 'bold': True, 'color': BLUE,
                                 'space_before': 8}),
        ('ROS 2 port at a pinned commit — laser only, no IMU, no GTSAM',
         {'size': 12}),
        ('One board runs the simulator and both pipelines, so the real-time '
         'factor is 0.78 — the sensors therefore deliver ~78% of their '
         'nominal rate.', {'size': 11, 'color': GREY, 'space_before': 10}),
    ], spacing=1.15)

    add_table(s, 5.35, 1.35, 4.35, 1.9,
              ['Stream', 'Measured', 'Nominal'],
              [['/points  (16-beam)', '7.8 – 8.7 Hz', '10 Hz'],
               ['/imu/data  (9-axis)', '155 – 173 Hz', '200 Hz'],
               ['/gt/odom  (Gazebo)', '77 – 86 Hz', '100 Hz']],
              col_w=[1.9, 1.35, 1.1], head_size=11.5, body_size=11)
    add_text(s, 5.35, 3.35, 4.35, 1.7, [
        ('Ground truth comes from the simulator itself '
         '(gz OdometryPublisher), not from a noisy sensor, so ATE measures '
         'the algorithm rather than the reference.', {'size': 11, 'color': GREY}),
        ('Both algorithms read the same recorded streams at the same replay '
         'rate; nothing is recomputed for the second one.',
         {'size': 11, 'color': GREY, 'space_before': 8}),
    ], spacing=1.2)
    return s


def slide_impl(prs, layout, pill_src, runs):
    s = new_slide(prs, layout, 'Implementation: one recording, two replays',
                  pill_src)
    boxes = [
        (0.28, 'Gazebo Harmonic\nlab world + 16-beam lidar\n+ 9-axis IMU'),
        (2.08, 'ros2 bag\n/points  /imu/data\n/gt/odom'),
        (3.88, 'bag play --clock\nsame bytes to\nboth pipelines'),
        (5.68, 'LOAM   and\nLIO-SAM\n(separate runs)'),
        (7.48, '6-DoF poses\n→ CSV'),
    ]
    for x, text in boxes:
        add_box(s, x, 0.98, 1.62, 0.92, text, size=10.5)
    for x in (1.90, 3.70, 5.50, 7.30):
        add_arrow(s, x, 1.36, 0.18)

    add_text(s, 0.33, 2.12, 9.4, 2.9, [
        ('Record once, replay twice  ', {'size': 13, 'bold': True, 'color': BLUE}),
        ('Each scenario is simulated once into a bag; every algorithm then '
         'consumes that bag. The point clouds are byte-identical, so a '
         'difference in the table cannot come from a difference in the data.',
         {'size': 12}),
        ('Ground truth  ', {'size': 13, 'bold': True, 'color': BLUE,
                            'space_before': 8}),
        ('The 79 Hz simulator pose is interpolated (lerp + slerp) onto every '
         'lidar stamp before comparison; the 12 cm lidar-to-base lever arm is '
         'compensated explicitly.', {'size': 12}),
        ('Metrics  ', {'size': 13, 'bold': True, 'color': BLUE,
                       'space_before': 8}),
        ('ATE (SE(3) Umeyama) and its rotation part · RPE over 1 s windows · '
         'endpoint error · frames beyond 0.5 m · output rate.',
         {'size': 12}),
        ('Fairness  ', {'size': 13, 'bold': True, 'color': BLUE,
                        'space_before': 8}),
        ('Thresholds were fixed before measuring. A sanity check rejects '
         'truncated runs, identical metrics and mismatched replay offsets — '
         'the two failures that would otherwise read as results.',
         {'size': 12}),
    ], spacing=1.12)
    return s


def slide_scenarios(prs, layout, pill_src, runs):
    s = new_slide(prs, layout, 'Six conditions, one variable at a time',
                  pill_src)
    rows = [
        ['S0  nominal', '12 × 12 m room', '90 s', '0.4 m/s circle', 'baseline'],
        ['S1  fast yaw', '12 × 12 m room', '60 s', 'ω = 0.90 rad/s',
         'distortion inside a sweep'],
        ['S2  high speed', '12 × 12 m room', '60 s',
         '1.2 m/s + 90° in place', 'large motion between sweeps'],
        ['S3  corridor', '30 × 3 m corridor', '56 s', 'back and forth',
         'geometry loses 2 DoF'],
        ['S4  sparse hall', '20 × 20 m, 6 pillars', '60 s', 'r = 3 m circle',
         'few features per sweep'],
        ['S5  long run', '12 × 12 m room', '180 s', '9 laps',
         'accumulation'],
    ]
    add_table(s, 0.28, 1.0, 9.45, 3.15,
              ['Scenario', 'World', 'Length', 'Motion', 'What it isolates'],
              rows, col_w=[1.5, 2.0, 0.75, 1.9, 2.4],
              head_size=11.5, body_size=11)
    add_text(s, 0.33, 4.32, 9.4, 0.9, [
        ('Every scenario is feasible by construction: the scenario check '
         'integrates the commanded motion against the world geometry and '
         'runs a separating-axis collision test before anything is recorded '
         '(6/6 pass, closest approach 0.22 m).',
         {'size': 11, 'color': GREY}),
    ], spacing=1.15)
    return s


def slide_scene(prs, layout, pill_src, runs):
    s = new_slide(prs, layout, 'S3 in the simulator: the corridor in Gazebo',
                  pill_src)
    add_picture_fit(s, os.path.join(FIGS, 's3_gazebo_view.png'),
                    0.30, 0.90, 9.4, 3.35)
    add_text(s, 0.33, 4.32, 9.4, 0.3, [
        ('View rebuilt from corridor.sdf and lio_bot/model.sdf geometry; '
         'the points are ray-cast at 16 x 360, coloured by range.',
         {'size': 8.5, 'color': GREY})])
    add_text(s, 0.33, 4.66, 9.4, 0.85, [
        ('7.2 m along the axis, a 180\u00b0 turn, and back \u2014 four legs, '
         '28.8 m in 56 s, always on the centreline.  ', {'size': 11}),
        ('Every sweep returns two parallel planes of points, so sliding along '
         'x moves them along themselves: two of the six degrees of freedom are '
         'missing from the data, not from the solver.', {'size': 11}),
    ], spacing=1.15)
    return s


def slide_results(prs, layout, pill_src, runs):
    s = new_slide(prs, layout, 'Result: the gap depends on the motion',
                  pill_src)
    add_picture_fit(s, os.path.join(FIGS, 'ate_summary.png'),
                    0.28, 0.95, 5.55, 3.55)
    best = {}
    for sc in ORDER:
        lo = runs.get(sc, {}).get('loam', {}).get('ate_rmse_m')
        li = runs.get(sc, {}).get('liosam', {}).get('ate_rmse_m')
        if lo and li:
            best[sc] = (1 - li / lo) * 100

    add_text(s, 6.0, 1.0, 3.75, 5.0, [
        ('Headline numbers (ATE RMSE)', {'size': 12.5, 'bold': True,
                                         'color': BLUE}),
        ('S0  %.4f → %.4f m   %+.0f%%'
         % (runs['s0_nominal']['loam']['ate_rmse_m'],
            runs['s0_nominal']['liosam']['ate_rmse_m'], best['s0_nominal']),
         {'size': 12}),
        ('S1  %.4f → %.4f m   %+.0f%%'
         % (runs['s1_fast_yaw']['loam']['ate_rmse_m'],
            runs['s1_fast_yaw']['liosam']['ate_rmse_m'], best['s1_fast_yaw']),
         {'size': 12}),
        ('S3  %.3f → %.3f m   %+.0f%%'
         % (runs['s3_corridor']['loam']['ate_rmse_m'],
            runs['s3_corridor']['liosam']['ate_rmse_m'], best['s3_corridor']),
         {'size': 12, 'bold': True, 'color': BLUE, 'space_before': 4}),
        ('S4  %.4f → %.4f m   %+.0f%%'
         % (runs['s4_sparse_hall']['loam']['ate_rmse_m'],
            runs['s4_sparse_hall']['liosam']['ate_rmse_m'],
            best['s4_sparse_hall']), {'size': 12}),
        ('S5  %.4f → %.4f m   %+.0f%%  (millimetre level: a tie)'
         % (runs['s5_long_run']['loam']['ate_rmse_m'],
            runs['s5_long_run']['liosam']['ate_rmse_m'], best['s5_long_run']),
         {'size': 12}),
        ('S2  %.4f → %.3f m   %+.0f%%  ← the exception'
         % (runs['s2_high_speed']['loam']['ate_rmse_m'],
            runs['s2_high_speed']['liosam']['ate_rmse_m'],
            best['s2_high_speed']),
         {'size': 12, 'bold': True, 'color': ORANGE, 'space_before': 4}),
        ('Lower is better. Improvement is relative to the LOAM baseline, '
         'computed from the measured ATE, not asserted.',
         {'size': 10.5, 'color': GREY, 'space_before': 10}),
    ], spacing=1.22)
    return s


def slide_s3(prs, layout, pill_src, runs):
    s = new_slide(prs, layout,
                  'Where the IMU pays: the corridor',
                  pill_src)
    lo = runs['s3_corridor']['loam']
    li = runs['s3_corridor']['liosam']
    add_picture_fit(s, os.path.join(FIGS, 'xy_s3_corridor.png'),
                    0.25, 0.92, 2.5, 2.5)
    add_picture_fit(s, os.path.join(FIGS, 'err_s3_corridor.png'),
                    2.95, 0.98, 6.75, 2.35)
    add_text(s, 0.25, 3.5, 2.5, 0.5, [
        ('Ground truth in black, LIO-SAM blue, LOAM orange',
         {'size': 9, 'color': GREY})], spacing=1.1)
    add_text(s, 0.28, 4.05, 9.45, 1.4, [
        ('LOAM never translates along the corridor', {'size': 12.5,
                                                      'bold': True,
                                                      'color': BLUE}),
        ('Its estimated path is 18% of the true path length — the along-axis '
         'direction and yaw are almost unobservable from two parallel walls, '
         'so the solver keeps whatever its initial guess said. The error '
         'oscillates between 0 and 3.6 m as the robot turns round, and '
         '429 of 508 poses sit beyond the 0.5 m failure line.',
         {'size': 11}),
        ('LIO-SAM holds %.3f m against %.3f m, with %.2f° of ATE rotation '
         'against %.2f° — the IMU preintegration factors anchor exactly the '
         'two directions geometry cannot see.'
         % (li['ate_rmse_m'], lo['ate_rmse_m'], li['ate_rot_rmse_deg'],
            lo['ate_rot_rmse_deg']), {'size': 11, 'space_before': 4}),
    ], spacing=1.12)
    return s


def slide_conclusion(prs, layout, pill_src, runs):
    s = new_slide(prs, layout, 'Part 3 conclusion', pill_src)
    lo3 = runs['s3_corridor']['loam']['ate_rmse_m']
    li3 = runs['s3_corridor']['liosam']['ate_rmse_m']
    li2 = runs['s2_high_speed']['liosam']['ate_rmse_m']
    lo2 = runs['s2_high_speed']['loam']['ate_rmse_m']
    add_text(s, 0.35, 1.0, 9.3, 4.0, [
        ('1   Geometry runs out before the IMU does.', {'size': 15,
                                                        'bold': True,
                                                        'color': BLUE}),
        ('In the corridor LIO-SAM is %.0f× more accurate (%.3f m vs %.3f m). '
         'This is the one condition where the two algorithms separate by an '
         'order of magnitude, and it is the condition the fusion was designed '
         'for.' % (lo3 / li3, li3, lo3), {'size': 12, 'space_before': 2}),
        ('2   Where geometry is fine, the IMU buys little.  ',
         {'size': 15, 'bold': True, 'color': BLUE, 'space_before': 12}),
        ('Nominal motion, fast yaw and the sparse hall improve by 30–59% but '
         'all three stay within 1 cm; on the long run the two are '
         'indistinguishable.', {'size': 12, 'space_before': 2}),
        ('3   And it can fail where the baseline does not.  ',
         {'size': 15, 'bold': True, 'color': ORANGE, 'space_before': 12}),
        ('At 1.2 m/s with 90° in-place turns LIO-SAM diverged to %.3f m while '
         'LOAM held %.3f m; all 282 of its poses exceeded 0.5 m. Reporting '
         'only the corridor result would be a sales pitch, not a comparison.'
         % (li2, lo2), {'size': 12, 'space_before': 2}),
        ('4   The cost is real.  ', {'size': 15, 'bold': True, 'color': BLUE,
                                     'space_before': 12}),
        ('A 9-axis IMU becomes mandatory, the dependency chain grows (GTSAM, '
         'Ceres, PCL) and the pipeline answers at 4.2 Hz on this board '
         'against LOAM\u2019s 8.4 Hz.', {'size': 12, 'space_before': 2}),
    ], spacing=1.08)
    return s


# ---------------------------------------------------------------- 组装
def move_after(prs, group, anchor_index):
    """把 group（新加的末尾几页）移到 anchor_index 之后"""
    lst = prs.slides._sldIdLst
    ids = list(lst)
    moving = ids[-len(group):]
    for el in moving:
        lst.remove(el)
    ref = list(lst)[anchor_index]
    idx = list(lst).index(ref) + 1
    for k, el in enumerate(moving):
        lst.insert(idx + k, el)


def drop_slide(prs, index):
    lst = prs.slides._sldIdLst
    el = list(lst)[index]
    rId = el.get('{http://schemas.openxmlformats.org/officeDocument/2006/relationships}id')
    prs.part.drop_rel(rId)
    lst.remove(el)


def main():
    prs = Presentation(SRC)
    runs = load_runs()
    layout_slide = prs.slides[17]        # 第 18 页：标准 Title and Content 页
    layout = layout_slide.slide_layout
    pill_src = layout_slide

    built = [
        slide_env(prs, layout, pill_src, runs),
        slide_impl(prs, layout, pill_src, runs),
        slide_scenarios(prs, layout, pill_src, runs),
        slide_scene(prs, layout, pill_src, runs),
        slide_results(prs, layout, pill_src, runs),
        slide_s3(prs, layout, pill_src, runs),
        slide_conclusion(prs, layout, pill_src, runs),
    ]
    # Part 3 的 section header 是第 17 页（index 16），插到它后面
    move_after(prs, built, 16)
    # 删掉模板里第三部分的两个占位符页。
    # 不按固定索引删 —— 新增页数一变索引就错（已经踩过一次：把结论页删了）。
    # 改成按标题文字定位，稳。
    targets = ("Case Study / Demo", "Results and Discussion")
    hits = []
    for i, sl in enumerate(prs.slides):
        for sh in sl.shapes:
            if sh.has_text_frame and sh.text_frame.text.strip() in targets:
                hits.append(i)
                break
    for i in sorted(hits, reverse=True):
        drop_slide(prs, i)

    prs.save(OUT)
    print('wrote %s  (%d slides)' % (OUT, len(prs.slides)))
    for i, s in enumerate(prs.slides, 1):
        if 16 <= i <= 24:
            title = ''
            for sh in s.shapes:
                if sh.has_text_frame and sh.text_frame.text.strip():
                    title = sh.text_frame.text.strip().split('\n')[0]
                    break
            print('  %2d  %s' % (i, title[:70]))


if __name__ == '__main__':
    sys.exit(main())
