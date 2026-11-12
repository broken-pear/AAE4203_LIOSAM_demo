"""把实测结果填进 AAE4203 模板的第三部分。

模板：report/AAE4203_temp.pptx（10 x 5.625 in，16:9）
输出：report/AAE4203_temp_with_Part3.pptx

结构（8 页，数字在生成时从 results/tables/all_runs.csv 读入）：
  18 Runtime Environment      环境 + S3 小车照片 + 小车参数
  19 Workflow                 录制 → 两条管线分别跑 → 定位 → 分析
  20 How ATE is computed      指标公式（本科数学）
  21 Six conditions           六场景 + 每个场景的几何俯视图
  22 S3 in Gazebo             实景渲染
  23 Result                   ATE 柱状图 + 逐场景数字
  24 Key advantage            S3 专题（几何退化 vs IMU）
  25 Conclusion               只留 S2 异常与 S3 优势
"""
import copy
import csv
import os
import sys

from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_SHAPE
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
WHITE = RGBColor(0xFF, 0xFF, 0xFF)
FONT = 'Arial'

ORDER = ['s0_nominal', 's1_fast_yaw', 's2_high_speed',
         's3_corridor', 's4_sparse_hall', 's5_long_run']


def load_runs():
    out = {}
    with open(CSV, newline='', encoding='utf-8') as f:
        for row in csv.DictReader(f):
            scene = os.path.basename(os.path.dirname(
                (row.get('csv') or '').replace('\\', '/')))
            vals = {}
            for k, v in row.items():
                try:
                    vals[k] = float(v)
                except (TypeError, ValueError, AttributeError):
                    vals[k] = v
            out.setdefault(scene, {})[row['algorithm']] = vals
    return out


def add_text(slide, x, y, w, h, paras, size=14, color=INK, bold=False,
             align=PP_ALIGN.LEFT, spacing=1.0):
    box = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    tf = box.text_frame
    tf.word_wrap = True
    tf.margin_top = tf.margin_bottom = Pt(0)
    for i, item in enumerate(paras):
        text, kw = item if isinstance(item, tuple) else (item, {})
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
    for sh in template_slide.shapes:
        if sh.name.startswith('Rounded Rectangle') and sh.has_text_frame \
                and 'Presenter' in sh.text_frame.text:
            el = copy.deepcopy(sh._element)
            slide.shapes._spTree.append(el)
            for t in el.iter('{http://schemas.openxmlformats.org/drawingml/2006/main}t'):
                t.text = text
            return True
    return False


def add_box(slide, x, y, w, h, text, size=10.5, fill=HEAD_FILL, color=INK,
            edge=BLUE, bold=True):
    shp = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(x),
                                 Inches(y), Inches(w), Inches(h))
    shp.fill.solid()
    shp.fill.fore_color.rgb = fill
    if edge is None:
        shp.line.fill.background()
    else:
        shp.line.color.rgb = edge
        shp.line.width = Pt(1)
    tf = shp.text_frame
    tf.word_wrap = True
    tf.margin_left = tf.margin_right = Pt(3)
    tf.margin_top = tf.margin_bottom = Pt(1)
    for i, line in enumerate(text.split('\n')):
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        p.alignment = PP_ALIGN.CENTER
        p.line_spacing = 0.95
        run = p.add_run()
        run.text = line
        run.font.name = FONT
        run.font.size = Pt(size if i == 0 else size - 1.5)
        run.font.bold = bold if i == 0 else False
        run.font.color.rgb = color
    return shp


def add_arrow(slide, x, y, w=0.20, h=0.14, down=False):
    shp = slide.shapes.add_shape(
        MSO_SHAPE.DOWN_ARROW if down else MSO_SHAPE.RIGHT_ARROW,
        Inches(x), Inches(y), Inches(h if down else w),
        Inches(w if down else h))
    shp.fill.solid()
    shp.fill.fore_color.rgb = BLUE
    shp.line.fill.background()
    return shp


def add_picture_fit(slide, path, x, y, w, h):
    if not os.path.isfile(path):
        add_text(slide, x, y + h / 2, w, 0.3,
                 ['[missing %s]' % os.path.basename(path)], size=10,
                 color=GREY, align=PP_ALIGN.CENTER)
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
    return slide.shapes.add_picture(path, Inches(x + (w - fw) / 2),
                                    Inches(y + (h - fh) / 2),
                                    Inches(fw), Inches(fh))


def add_table(slide, x, y, w, h, header, rows, col_w=None, head_size=10.5,
              body_size=10, highlight=None, left_align_first=False):
    n_r, n_c = len(rows) + 1, len(header)
    gf = slide.shapes.add_table(n_r, n_c, Inches(x), Inches(y), Inches(w),
                                Inches(h))
    tbl = gf.table
    tblPr = tbl._tbl.tblPr
    tblPr.set('firstRow', '1')
    tblPr.set('bandRow', '1')
    style = tblPr.find(
        '{http://schemas.openxmlformats.org/drawingml/2006/main}tableStyleId')
    if style is None:
        from lxml import etree
        style = etree.SubElement(
            tblPr,
            '{http://schemas.openxmlformats.org/drawingml/2006/main}tableStyleId')
    style.text = '{5C22544A-7EE6-4342-B048-85BDC9FD1C3A}'

    if col_w:
        total = sum(col_w)
        for i, cw in enumerate(col_w):
            tbl.columns[i].width = Inches(w * cw / total)
    for r in tbl.rows:
        r.height = Inches(h / n_r)

    def fill(cell, text, size, bold, colour, head, align):
        cell.fill.solid()
        cell.fill.fore_color.rgb = HEAD_FILL if head else WHITE
        cell.margin_left = cell.margin_right = Inches(0.045)
        cell.margin_top = cell.margin_bottom = Inches(0.01)
        cell.vertical_anchor = 3
        tf = cell.text_frame
        tf.word_wrap = True
        p = tf.paragraphs[0]
        p.alignment = align
        run = p.add_run()
        run.text = text
        run.font.name = FONT
        run.font.size = Pt(size)
        run.font.bold = bold
        run.font.color.rgb = colour

    for j, t in enumerate(header):
        fill(tbl.cell(0, j), t, head_size, True, WHITE, True, PP_ALIGN.CENTER)
    for i, row in enumerate(rows):
        for j, val in enumerate(row):
            col, bold = INK, False
            if highlight and (i, j) in highlight:
                col, bold = BLUE, True
            al = (PP_ALIGN.LEFT if (left_align_first and j == 0)
                  else PP_ALIGN.CENTER)
            fill(tbl.cell(i + 1, j), str(val), body_size, bold, col, False, al)
    return tbl


def new_slide(prs, layout, title, presenter_from):
    slide = prs.slides.add_slide(layout)
    for ph in list(slide.placeholders):
        if ph.placeholder_format.idx != 0:
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


def slide_env(prs, layout, pill, runs):
    s = new_slide(prs, layout, 'Runtime Environment', pill)
    add_text(s, 0.33, 0.95, 4.35, 0.3, [
        ('Where the numbers were produced', {'size': 12.5, 'bold': True,
                                             'color': BLUE})])
    add_text(s, 0.33, 1.32, 4.4, 3.9, [
        ('Hardware  ', {'size': 12, 'bold': True, 'color': BLUE}),
        ('NVIDIA Jetson Orin Nano 8 GB (aarch64), Ubuntu 24.04 LTS',
         {'size': 11}),
        ('Middleware  ', {'size': 12, 'bold': True, 'color': BLUE,
                          'space_before': 8}),
        ('ROS 2 Jazzy · Gazebo Harmonic 8.15 (ros_gz) · RViz2',
         {'size': 11}),
        ('Algorithms  ', {'size': 12, 'bold': True, 'color': BLUE,
                          'space_before': 8}),
        ('LIO-SAM (upstream ros2 branch): C++17, GTSAM 4.2, Ceres 2.2, iSAM2',
         {'size': 11}),
        ('A-LOAM (baseline): ROS 2 port at a pinned commit — laser only, '
         'no IMU, no factor graph', {'size': 11, 'space_before': 4}),
        ('One board runs the simulator and both pipelines, so the real-time '
         'factor is 0.78 and every sensor delivers ~78 % of its nominal rate.',
         {'size': 10.5, 'color': GREY, 'space_before': 10}),
    ], spacing=1.15)

    add_picture_fit(s, os.path.join(FIGS, 's3_robot.png'),
                    4.95, 0.92, 2.30, 2.92)
    add_text(s, 4.95, 3.90, 2.30, 0.34, [
        ('The robot used in S3', {'size': 10, 'color': GREY,
                                  'align': PP_ALIGN.CENTER})])

    add_table(s, 7.42, 1.05, 2.34, 2.45,
              ['Item', 'Value'],
              [['Base', '0.50 × 0.34 × 0.12 m'],
               ['Mass', '6.1 kg total'],
               ['Wheels', 'r 0.10 m, track 0.40 m'],
               ['LiDAR', '16 × 360, 10 Hz'],
               ['', '±15°, 0.2 – 40 m'],
               ['IMU', '200 Hz, 9-axis'],
               ['', 'co-located with LiDAR']],
              col_w=[0.85, 1.62], head_size=9.5, body_size=8.8,
              left_align_first=True)
    add_text(s, 7.42, 3.66, 2.34, 0.75, [
        ('Sensors share one frame, so the extrinsic is the identity and '
         'neither algorithm gets a lever-arm advantage.',
         {'size': 8.5, 'color': GREY})], spacing=1.1)
    return s


def slide_impl(prs, layout, pill, runs):
    s = new_slide(prs, layout,
                  'Workflow: record once, replay twice, compare',
                  pill)
    add_box(s, 0.30, 1.16, 1.80, 1.24,
            '1  Record once\nGazebo Harmonic\n3 topics → one bag', size=10.5)
    add_box(s, 2.92, 0.96, 1.98, 0.74,
            '2a  Replay → LIO-SAM\nLiDAR + IMU + GTSAM', size=10)
    add_box(s, 2.92, 1.86, 1.98, 0.74,
            '2b  Replay → LOAM\nLiDAR only (baseline)', size=10)
    add_box(s, 5.72, 1.16, 1.72, 1.24,
            '3  Localisation\none 6-DoF pose\nper sweep', size=10.5)
    add_box(s, 8.10, 1.16, 1.60, 1.24,
            '4  Analysis\nATE / RPE /\nrobustness', size=10.5)

    add_arrow(s, 2.12, 1.71)
    add_arrow(s, 4.92, 1.26)
    add_arrow(s, 4.92, 2.16)
    add_arrow(s, 7.46, 1.71)

    add_text(s, 2.42, 1.72, 0.46, 0.24, [
        ('same bag', {'size': 7.5, 'color': GREY})], spacing=1.0)

    add_text(s, 0.33, 2.78, 9.4, 2.4, [
        ('One recording, two consumers  ', {'size': 12, 'bold': True,
                                            'color': BLUE}),
        ('Step 1 runs the scenario once in Gazebo and writes /points, '
         '/imu/data and /gt/odom to a bag. Steps 2a and 2b replay that same '
         'bag into each algorithm, so the point clouds are byte-identical — '
         'a difference in the table cannot come from a difference in data.',
         {'size': 10.5}),
        ('Localisation  ', {'size': 12, 'bold': True, 'color': BLUE,
                            'space_before': 7}),
        ('Each algorithm publishes its own pose sequence; the recorder stores '
         'it beside the interpolated ground truth, one CSV per scenario and '
         'algorithm.', {'size': 10.5}),
        ('Analysis  ', {'size': 12, 'bold': True, 'color': BLUE,
                        'space_before': 7}),
        ('Only at the end are the two CSVs aligned and scored — ATE, RPE, '
         'endpoint error, poses beyond 0.5 m, output rate. Thresholds were '
         'fixed before measuring and a sanity check rejects truncated or '
         'identical runs.', {'size': 10.5}),
    ], spacing=1.12)
    return s


def slide_ate(prs, layout, pill, runs):
    s = new_slide(prs, layout, 'The metric: how ATE is computed', pill)

    # 左栏：三步 + 两张公式图（公式用 mathtext 渲染，见 make_ate_formula.py）
    add_text(s, 0.33, 0.93, 4.55, 0.3, [
        ('1   pair the poses in time', {'size': 12.5, 'bold': True,
                                        'color': BLUE})])
    add_text(s, 0.33, 1.24, 4.55, 0.4, [
        ('Match each estimate to the ground-truth pose carrying the same lidar '
         'stamp; keep one-to-one matches only.', {'size': 9.8})], spacing=1.1)

    add_text(s, 0.33, 1.66, 4.55, 0.3, [
        ('2   remove the unknown frame', {'size': 12.5, 'bold': True,
                                          'color': BLUE})])
    add_text(s, 0.33, 1.97, 4.55, 0.42, [
        ('Solve the rigid transform (R, t) that best maps the estimate onto '
         'the truth \u2014 least squares, closed form via SVD:',
         {'size': 9.8})], spacing=1.1)

    add_picture_fit(s, os.path.join(FIGS, 'ate_formula_align.png'),
                    0.36, 2.40, 3.95, 1.30)
    add_text(s, 0.33, 3.74, 4.55, 0.34, [
        ('The det term forbids a mirrored solution, and no scale is fitted: a '
         'trajectory 10 % too large must not be "aligned" into agreement.',
         {'size': 8.5, 'color': GREY})], spacing=1.08)

    add_text(s, 0.33, 4.10, 4.55, 0.3, [
        ('3   one number per run', {'size': 12.5, 'bold': True,
                                    'color': BLUE})])
    add_picture_fit(s, os.path.join(FIGS, 'ate_formula_error.png'),
                    0.36, 4.40, 2.05, 0.98)
    add_text(s, 2.55, 4.36, 2.35, 0.95, [
        ('RMSE, not the mean, so one bad stretch cannot hide behind many good '
         'ones. Metres, lower is better.', {'size': 9, 'color': GREY})],
        spacing=1.1)

    # 右栏：图解 + 说明
    add_picture_fit(s, os.path.join(FIGS, 'ate_math.png'),
                    5.10, 0.93, 4.65, 1.80)
    add_text(s, 5.10, 2.84, 4.65, 2.4, [
        ('Worked example (illustration)', {'size': 11, 'bold': True,
                                           'color': BLUE}),
        ('Nine poses, estimate rotated by 32\u00b0 and offset. Step 2 '
         'recovers that transform; the residual segments of step 3 give '
         'ATE = 0.11 m.', {'size': 9.8}),
        ('What we report  ', {'size': 11, 'bold': True, 'color': BLUE,
                              'space_before': 7}),
        ('ATE RMSE and its rotation part, RPE over 1 s windows, endpoint '
         'error, poses beyond 0.5 m, and output rate \u2014 because one '
         'average can hide a trajectory that is locally fine and globally '
         'wrong.', {'size': 9.8}),
    ], spacing=1.13)
    return s


def slide_scenarios(prs, layout, pill, runs):
    s = new_slide(prs, layout, 'Six conditions, one variable at a time', pill)
    rows = [
        ['S0  nominal', '12 × 12 m room', '36 m', 'circle, 0.40 m/s',
         'baseline: geometry is rich'],
        ['S1  fast yaw', '12 × 12 m room', '27 m', 'ω = 0.90 rad/s',
         'distortion inside one sweep'],
        ['S2  high speed', '12 × 12 m room', '48 m', '1.2 m/s + 90° turns',
         'large motion between sweeps'],
        ['S3  corridor', '30 × 3 m corridor', '29 m', 'back and forth',
         'geometry loses 2 DoF'],
        ['S4  sparse hall', '20 × 20 m, 6 pillars', '36 m', 'r = 3 m circle',
         'few features per sweep'],
        ['S5  long run', '12 × 12 m room', '72 m', '9 laps',
         'accumulation over 180 s'],
    ]
    add_table(s, 0.24, 0.98, 6.45, 2.80,
              ['Scenario', 'World', 'Path', 'Motion', 'What it isolates'],
              rows, col_w=[1.25, 1.75, 0.55, 1.6, 1.95],
              head_size=9.5, body_size=9.0, left_align_first=True)

    xs = [7.05, 8.48]
    ys = [0.96, 2.06, 3.16]
    for i in range(6):
        gx, gy = xs[i % 2], ys[i // 2]
        add_picture_fit(s, os.path.join(FIGS, 'map_s%d.png' % i),
                        gx, gy, 1.28, 0.76)
        add_text(s, gx, gy + 0.76, 1.28, 0.2, [
            ('S%d' % i, {'size': 8.5, 'color': GREY,
                         'align': PP_ALIGN.CENTER})])

    add_text(s, 0.24, 3.92, 6.45, 1.1, [
        ('The sketches on the right show each world\u2019s actual geometry '
         'and commanded path, read from the scenario files. S3 is the only '
         'case whose driven direction is parallel to the only surfaces in '
         'view.', {'size': 9.5, 'color': GREY})], spacing=1.12)
    add_text(s, 7.05, 4.30, 2.7, 0.8, [
        ('Grey: walls and pillars · orange: commanded path · dot: start',
         {'size': 8.5, 'color': GREY})], spacing=1.1)
    return s


def slide_scene(prs, layout, pill, runs):
    s = new_slide(prs, layout, 'S3 in the simulator: the corridor in Gazebo',
                  pill)
    add_picture_fit(s, os.path.join(FIGS, 's3_gazebo_view.png'),
                    0.30, 0.88, 9.4, 3.28)
    add_text(s, 0.33, 4.24, 9.4, 0.3, [
        ('View rebuilt from corridor.sdf and lio_bot/model.sdf geometry; '
         'points are ray-cast at 16 × 360, coloured by range.',
         {'size': 8.5, 'color': GREY})])
    add_text(s, 0.33, 4.58, 9.4, 0.9, [
        ('7.2 m along the axis, a 180° turn, and back — four legs, 28.8 m in '
         '56 s, always on the centreline.  ', {'size': 11}),
        ('Every sweep returns two parallel planes of points, so sliding along '
         'x moves them along themselves: two of the six degrees of freedom '
         'are missing from the data, not from the solver.', {'size': 11}),
    ], spacing=1.15)
    return s


def _ratio_text(lo, li):
    """以 LOAM 基线自身的误差为分母的下降比例。

        reduction = (ATE(LOAM) - ATE(LIO-SAM)) / ATE(LOAM) * 100 %

    正值表示 LIO-SAM 误差更小，负值表示输给基线。分母固定是基线，
    这样"改善"这个词只有一种读法 —— 之前用对方的误差做分母时，
    同一个场景会算出两个都说得通的百分比。
    """
    red = (lo - li) / lo * 100
    # 100 以内保留一位小数（98.6 不能显示成 99），上千才舍成整数
    pct = ('%+.0f %%' % red) if abs(red) >= 100 else ('%+.1f %%' % red)
    return pct


def slide_results(prs, layout, pill, runs):
    s = new_slide(prs, layout, 'Result: the gap depends on the motion', pill)
    add_picture_fit(s, os.path.join(FIGS, 'ate_summary.png'),
                    0.28, 0.95, 5.55, 3.55)

    rows = []
    for sc, tag in zip(ORDER, ['S0', 'S1', 'S2', 'S3', 'S4', 'S5']):
        lo = runs[sc]['loam']['ate_rmse_m']
        li = runs[sc]['liosam']['ate_rmse_m']
        pct = _ratio_text(lo, li)
        rows.append((sc, tag, lo, li, pct))

    paras = [('Headline numbers (ATE RMSE)', {'size': 12, 'bold': True,
                                              'color': BLUE})]
    for sc, tag, lo, li, pct in rows:
        if sc == 's3_corridor':
            kw = {'size': 10.5, 'bold': True, 'color': BLUE, 'space_before': 4}
        elif sc == 's2_high_speed':
            kw = {'size': 10.5, 'bold': True, 'color': ORANGE, 'space_before': 4}
        else:
            kw = {'size': 10.5}
        paras.append(('%s  %.4f \u2192 %.4f m      %s'
                      % (tag, lo, li, pct), kw))
    paras.append(
        ('Reduction is measured against the LOAM baseline\u2019s own error: '
         '(ATE(LOAM) \u2212 ATE(LIO-SAM)) / ATE(LOAM) \u00d7 100 %. Positive '
         'means LIO-SAM is more accurate, negative means it lost to the '
         'baseline. Computed from the CSV at full precision.',
         {'size': 8.5, 'color': GREY, 'space_before': 8}))
    add_text(s, 6.0, 1.0, 3.75, 4.3, paras, spacing=1.2)
    return s


def slide_s3(prs, layout, pill, runs):
    s = new_slide(prs, layout,
                  'Key advantage: where geometry runs out', pill)
    lo = runs['s3_corridor']['loam']
    li = runs['s3_corridor']['liosam']
    add_picture_fit(s, os.path.join(FIGS, 'xy_s3_corridor.png'),
                    0.22, 0.90, 2.30, 2.10)
    add_picture_fit(s, os.path.join(FIGS, 'err_s3_corridor.png'),
                    2.66, 0.92, 7.00, 2.08)

    add_table(s, 0.24, 3.12, 2.28, 1.52,
              ['S3', 'LOAM', 'LIO-SAM'],
              [['ATE', '%.3f' % lo['ate_rmse_m'], '%.3f' % li['ate_rmse_m']],
               ['rot °', '%.1f' % lo['ate_rot_rmse_deg'],
                '%.1f' % li['ate_rot_rmse_deg']],
               ['final m', '%.2f' % lo['final_pos_err_m'],
                '%.3f' % li['final_pos_err_m']],
               ['>0.5 m', '%d' % int(lo['frames_over_0p5m']),
                '%d' % int(li['frames_over_0p5m'])]],
              col_w=[0.62, 0.85, 0.85], head_size=8.5, body_size=8.5,
              highlight={(0, 2), (1, 2), (2, 2), (3, 2)})

    add_text(s, 2.66, 3.14, 7.0, 1.95, [
        ('What the two plots say', {'size': 11.5, 'bold': True, 'color': BLUE}),
        ('Trajectory: LOAM (orange) covers a fraction of the corridor, '
         'LIO-SAM (blue) covers all of it. LOAM\u2019s estimated path is 18 % '
         'of the true length — it barely translates, because translation along '
         'x and yaw are not observable from two parallel walls.',
         {'size': 9.8}),
        ('Error: LOAM oscillates between 0 and 3.6 m, resetting each time the '
         'robot turns round; LIO-SAM stays under 0.16 m for the whole run. '
         '429 of LOAM\u2019s 508 poses are beyond the 0.5 m line, none of '
         'LIO-SAM\u2019s are.', {'size': 9.8, 'space_before': 4}),
        ('That is the argument for the IMU: it constrains exactly the two '
         'directions the corridor removes.',
         {'size': 9.8, 'bold': True, 'space_before': 4}),
    ], spacing=1.12)
    return s


def slide_conclusion(prs, layout, pill, runs):
    s = new_slide(prs, layout, 'Part 3 conclusion', pill)
    lo3 = runs['s3_corridor']['loam']['ate_rmse_m']
    li3 = runs['s3_corridor']['liosam']['ate_rmse_m']
    li2 = runs['s2_high_speed']['liosam']['ate_rmse_m']
    lo2 = runs['s2_high_speed']['loam']['ate_rmse_m']
    add_text(s, 0.35, 1.02, 9.3, 3.9, [
        ('1   Geometric degeneracy is where the IMU earns its place.',
         {'size': 15, 'bold': True, 'color': BLUE}),
        ('In the corridor LIO-SAM cuts ATE by %.1f %% against the LOAM '
         'baseline (%.3f m vs %.3f m), and the mechanism is visible in the '
         'data: LOAM never translates along the axis it cannot observe. '
         'Where geometry is rich (S0, S1, S4, S5) the two stay within a '
         'centimetre of each other — the fusion buys precision only under '
         'degeneracy.'
         % ((lo3 - li3) / lo3 * 100, li3, lo3),
         {'size': 12, 'space_before': 3}),
        ('2   One anomaly, reported rather than explained.',
         {'size': 15, 'bold': True, 'color': ORANGE, 'space_before': 16}),
        ('On S2 (1.2 m/s with 90° in-place turns) LIO-SAM diverged to %.3f m '
         'while LOAM held %.3f m, with every pose beyond 0.5 m. The same bag '
         'replayed at a different rate gave 0.040 m, so the result is not '
         'stable. Message loss or IMU preintegration gaps at this speed are '
         'plausible, but we have not isolated the cause and do not claim to.'
         % (li2, lo2), {'size': 12, 'space_before': 3}),
    ], spacing=1.14)
    return s


def move_after(prs, count, anchor_index):
    lst = prs.slides._sldIdLst
    ids = list(lst)
    moving = ids[-count:]
    for el in moving:
        lst.remove(el)
    ref = list(lst)[anchor_index]
    idx = list(lst).index(ref) + 1
    for k, el in enumerate(moving):
        lst.insert(idx + k, el)


def drop_slide(prs, index):
    lst = prs.slides._sldIdLst
    el = list(lst)[index]
    rId = el.get(
        '{http://schemas.openxmlformats.org/officeDocument/2006/relationships}id')
    prs.part.drop_rel(rId)
    lst.remove(el)


def main():
    prs = Presentation(SRC)
    runs = load_runs()
    layout_slide = prs.slides[17]
    layout = layout_slide.slide_layout

    built = [
        slide_env(prs, layout, layout_slide, runs),
        slide_impl(prs, layout, layout_slide, runs),
        slide_ate(prs, layout, layout_slide, runs),
        slide_scenarios(prs, layout, layout_slide, runs),
        slide_scene(prs, layout, layout_slide, runs),
        slide_results(prs, layout, layout_slide, runs),
        slide_s3(prs, layout, layout_slide, runs),
        slide_conclusion(prs, layout, layout_slide, runs),
    ]
    move_after(prs, len(built), 16)

    targets = ('Case Study / Demo', 'Results and Discussion')
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
        if 16 <= i <= 26:
            title = ''
            for sh in s.shapes:
                if sh.has_text_frame and sh.text_frame.text.strip():
                    title = sh.text_frame.text.strip().split('\n')[0]
                    break
            print('  %2d  %s' % (i, title[:72]))


if __name__ == '__main__':
    sys.exit(main())
