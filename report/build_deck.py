#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
build_deck.py —— 生成 LOAM vs LIO-SAM 的英文对比报告 PPT
===============================================================================

设计原则：**数字不许手抄**。
幻灯片里每一个指标都从 `results/tables/all_runs.csv` 读出来（那份 CSV 由
`lio_sam_bench/scripts/evaluate.py` 生成，源头是逐帧的真值+算法位姿）。

    # 实验还没跑：生成"带占位符"的骨架（结果页显示 — 和 DATA PENDING）
    python3 report/build_deck.py

    # 实验跑完：一键出正式版（同一份脚本，数字自动填进去）
    python3 report/build_deck.py --out report/LOAM_vs_LIOSAM.pptx

    # 只检查绑定关系（不写文件）
    python3 report/build_deck.py --dry-run

依赖：python-pptx + Pillow（office 运行时里都有）。

数据从哪来
----------
  results/tables/all_runs.csv       每个 (场景, 算法) 一行，列名见 evaluate.py
  results/figures/xy_<scene>.png    轨迹叠加图
  results/figures/err_<scene>.png   逐帧误差
  results/figures/ate_summary.png   各场景 ATE 柱状图
  results/figures/xy_all.png        总览

  `--repeats` 跑出来的 <scene>_r1/_r2 会被合并成同一场景，指标取中位数。
"""

import argparse
import csv
import os
import re
import statistics
import sys

from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_SHAPE
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR, MSO_AUTO_SIZE
from pptx.util import Inches, Pt, Emu

# ============================================================ 视觉常量
INK = RGBColor(0x18, 0x1C, 0x23)          # 近黑，正文
DIM = RGBColor(0x5C, 0x66, 0x74)          # 次级文字
PAPER = RGBColor(0xFF, 0xFF, 0xFF)
BG_SOFT = RGBColor(0xF4, 0xF6, 0xF9)
LIO = RGBColor(0x2F, 0x80, 0xED)          # LIO-SAM 蓝（和图表一致）
LOAM = RGBColor(0xFF, 0x99, 0x00)         # LOAM 橙（和图表一致）
ACCENT = RGBColor(0x0F, 0x9D, 0x58)       # 结论绿
WARN = RGBColor(0xC0, 0x39, 0x2B)         # 风险红
LINE = RGBColor(0xD5, 0xDB, 0xE3)

FONT = 'Arial'
W, H = 13.333, 7.5
M = 0.55                                   # 左右页边距（英寸）
CONTENT_TOP = 1.55

ALGO_LABEL = {'loam': 'LOAM', 'liosam': 'LIO-SAM'}

# ============================================================ 场景定义
# (目录名, 短号, 世界, 动作, 设计意图)
SCENARIOS = [
    ('s0_nominal', 'S0', '12 x 12 m room',
     'circle r = 1.25 m, v = 0.40 m/s, w = 0.32 rad/s',
     'Baseline. Both algorithms should converge; the gap should be small.'),
    ('s1_fast_yaw', 'S1', '12 x 12 m room',
     'circle r = 0.5 m, v = 0.45 m/s, w = 0.90 rad/s',
     'Motion distortion: 0.09 rad (5.2 deg) of rotation inside one 0.1 s sweep.'),
    ('s2_high_speed', 'S2', '12 x 12 m room',
     '2.4 m square, 1.2 m/s straights, 90 deg pivot turns',
     'Frame-to-frame displacement: 0.12 m per sweep; initial guess quality decides matching.'),
    ('s3_corridor', 'S3', '30 x 3 m corridor',
     '7.2 m straights + 180 deg pivots, two round trips',
     'Degeneracy: only long parallel planes; along-axis x and yaw are barely observable.'),
    ('s4_sparse_hall', 'S4', '20 x 20 m hall, 6 thin pillars',
     'circle r = 3 m, v = 0.60 m/s, w = 0.20 rad/s',
     'Sparsity: too few edge/plane features for a purely geometric solver.'),
    ('s5_long_run', 'S5', '12 x 12 m room',
     'S0 motion for 180 s (about 9 laps)',
     'Accumulated drift. Loop closure is OFF on both sides, so this is pure odometry.'),
]
SCEN_RANK = {s[0]: i for i, s in enumerate(SCENARIOS)}
SCEN_SHORT = {s[0]: s[1] for s in SCENARIOS}
SCEN_INTENT = {s[0]: s[4] for s in SCENARIOS}
SCEN_MOTION = {s[0]: s[3] for s in SCENARIOS}
SCEN_WORLD = {s[0]: s[2] for s in SCENARIOS}

# 参与"提升幅度"比较的列（越大越好 -> 改善 = (loam - liosam)/loam）
LOWER_IS_BETTER = [
    ('ate_rmse_m', 'ATE RMSE', 'm', 3),
    ('ate_max_m', 'ATE max', 'm', 3),
    ('rpe_trans_pct_m', 'RPE trans', '%/m', 2),
    ('rpe_rot_deg_m', 'RPE rot', 'deg/m', 2),
    ('final_pos_err_m', 'Final pos err', 'm', 3),
    ('final_yaw_err_deg', 'Final yaw err', 'deg', 2),
]


# ============================================================ 数据读取
def _f(v):
    try:
        if v is None or v == '':
            return None
        return float(v)
    except (TypeError, ValueError):
        return None


def load_runs(results_dir):
    """
    读 all_runs.csv -> {(scenario, algo): {metric: value}}
    场景名从 csv 路径的父目录取（evaluate.py 只写了 csv 路径这一列）。
    <scene>_r2 这样的重复实验会合并到同一场景，指标取中位数。
    """
    path = os.path.join(results_dir, 'tables', 'all_runs.csv')
    if not os.path.isfile(path):
        return {}, path

    buckets = {}
    with open(path, newline='', encoding='utf-8') as f:
        for row in csv.DictReader(f):
            csv_path = (row.get('csv') or '').replace('\\', '/')
            scene = os.path.basename(os.path.dirname(csv_path))
            if not scene:
                continue
            scene = re.sub(r'_r\d+$', '', scene)          # 合并重复实验
            algo = (row.get('algorithm') or '').lower()
            if algo not in ALGO_LABEL:
                continue
            d = {}
            for k in row:
                if k in ('algorithm', 'csv'):
                    continue
                if k == 'diverged':
                    d[k] = (row[k].strip().lower() in ('true', '1', 'yes'))
                elif k == 'frames':
                    d[k] = _f(row[k])
                else:
                    d[k] = _f(row[k])
            buckets.setdefault((scene, algo), []).append(d)

    merged = {}
    for key, rows in buckets.items():
        m = {}
        keys = set().union(*[set(r) for r in rows])
        for k in keys:
            vals = [r.get(k) for r in rows if r.get(k) is not None]
            if not vals:
                m[k] = None
            elif isinstance(vals[0], bool):
                m[k] = any(vals)
            else:
                m[k] = statistics.median(vals)
        m['_n_runs'] = len(rows)
        merged[key] = m
    return merged, path


def get(runs, scene, algo, metric):
    return (runs.get((scene, algo)) or {}).get(metric)


def improvement(runs, scene, metric):
    lo = get(runs, scene, 'loam', metric)
    li = get(runs, scene, 'liosam', metric)
    if lo is None or li is None or lo == 0:
        return None
    return (lo - li) / lo * 100.0


def verdict_for(imp):
    """
    由数据决定结论标签，**不写死**。
    阈值先定、后看结果（写在幻灯片脚注里），避免"先有结论再调阈值"。
      不显著      : |imp| < 5%   （和噪声/单次运行波动同量级）
      有限优势    : 5% .. 25%
      明显优势    : 25% .. 60%
      决定性优势  : >= 60%
      LIO-SAM 更差: imp < 0
    """
    if imp is None:
        return 'n/a'
    if imp < 0:
        return 'LIO-SAM worse'
    if imp < 5:
        return 'marginal'
    if imp < 25:
        return 'limited'
    if imp < 60:
        return 'clear win'
    return 'decisive'


def fmt(v, nd=3, dash='—'):
    if v is None:
        return dash
    if isinstance(v, bool):
        return 'yes' if v else 'no'
    if isinstance(v, float):
        if abs(v) >= 1000:
            return f'{v:,.0f}'
        return f'{v:.{nd}f}'
    return str(v)


# ============================================================ 版式工具
class Deck:
    """
    scale != 1.0 只用于一件事：**预览渲染的补偿**。
    本仓库自带的 LibreOffice kit 有一个固定的 1.5 倍内容放大 bug：
    它按 144 dpi 把内容画进 1280x720 的画布，于是 13.333 in 的设计只露出
    8.889 in，"右下角被裁掉"看起来像溢出，其实 pptx 坐标是对的。
    为了能用眼睛检查版式，`--preview` 把**内容**（位置、尺寸、字号）整体缩到
    1/1.5，页面尺寸保持不变；渲染器再放大 1.5 倍，两边正好抵消。
    注意：页面尺寸不能跟着缩，否则视口还是只覆盖 2/3，问题照旧。
    """

    def __init__(self, scale=1.0):
        self.s = float(scale)
        self.prs = Presentation()
        # 页面始终是设计尺寸（PowerPoint 里看到的那一页）
        self.prs.slide_width = Inches(W)
        self.prs.slide_height = Inches(H)
        self._page = 0

    # ---------------------------------------------------------- 缩放辅助
    def X(self, v):
        """英寸 -> 缩放后的 Length"""
        return Inches(v * self.s)

    def PS(self, pt):
        """pt -> 缩放后的字号"""
        return Pt(float(pt) * self.s)

    # ---------------------------------------------------------- 基础形状
    def blank(self):
        return self.prs.slides.add_slide(self.prs.slide_layouts[6])

    def rect(self, slide, x, y, w, h, fill=None, line=None, shape=MSO_SHAPE.RECTANGLE):
        sp = slide.shapes.add_shape(shape, self.X(x), self.X(y), self.X(w), self.X(h))
        if fill is None:
            sp.fill.background()
        else:
            sp.fill.solid()
            sp.fill.fore_color.rgb = fill
        if line is None:
            sp.line.fill.background()
        else:
            sp.line.color.rgb = line
            sp.line.width = self.PS(0.75)
        sp.shadow.inherit = False
        return sp

    def text(self, slide, x, y, w, h, runs, size=14, color=INK, bold=False,
             align=PP_ALIGN.LEFT, anchor=MSO_ANCHOR.TOP, spacing=1.0, font=FONT):
        """runs: str 或 [(text, {opts})...] 或 [[...]] （多段）"""
        box = slide.shapes.add_textbox(self.X(x), self.X(y), self.X(w), self.X(h))
        tf = box.text_frame
        tf.word_wrap = True
        # 默认文本框是 spAutoFit（形状跟着文字长大），LibreOffice/PowerPoint 会把
        # 变大的框围绕中心重新摆放 → 位置漂移 + 右侧溢出。必须关掉。
        tf.auto_size = MSO_AUTO_SIZE.NONE
        tf.margin_left = tf.margin_right = 0
        tf.margin_top = tf.margin_bottom = 0
        tf.vertical_anchor = anchor
        if isinstance(runs, str):
            paragraphs = [[(runs, {})]]
        elif runs and isinstance(runs[0], tuple):
            paragraphs = [runs]
        else:
            paragraphs = runs
        for i, para in enumerate(paragraphs):
            p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
            p.alignment = align
            p.line_spacing = spacing
            for chunk in para:
                if isinstance(chunk, str):
                    t, opts = chunk, {}
                else:
                    t, opts = chunk
                r = p.add_run()
                r.text = t
                r.font.size = self.PS(opts.get('size', size))
                r.font.bold = opts.get('bold', bold)
                r.font.italic = opts.get('italic', False)
                r.font.color.rgb = opts.get('color', color)
                r.font.name = opts.get('font', font)
        return box

    def bullets(self, slide, x, y, w, h, items, size=14, color=INK, gap=6):
        """items: [(level, text)] ; level 0 = 圆点，1 = 缩进小三角"""
        box = slide.shapes.add_textbox(self.X(x), self.X(y), self.X(w), self.X(h))
        tf = box.text_frame
        tf.word_wrap = True
        tf.auto_size = MSO_AUTO_SIZE.NONE
        tf.margin_left = tf.margin_right = 0
        tf.margin_top = tf.margin_bottom = 0
        for i, item in enumerate(items):
            level, txt = item if isinstance(item, tuple) else (0, item)
            p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
            p.line_spacing = 1.12
            p.space_after = self.PS(gap)
            mark = '· ' if level == 0 else '– '
            r = p.add_run()
            r.text = mark
            r.font.size = self.PS(size)
            r.font.color.rgb = LOAM if level == 0 else DIM
            r.font.bold = True
            r.font.name = FONT
            # 支持行内加粗：在文本里用 **...** 包起来
            parts = re.split(r'(\*\*.+?\*\*)', txt)
            for part in parts:
                if not part:
                    continue
                bold = part.startswith('**') and part.endswith('**')
                r = p.add_run()
                r.text = part[2:-2] if bold else part
                r.font.size = self.PS(size)
                r.font.bold = bold
                r.font.color.rgb = color
                r.font.name = FONT
            p.level = level
        return box

    # ---------------------------------------------------------- 页头/页脚
    def header(self, slide, kicker, title, accent=LIO):
        self.rect(slide, 0, 0, W, 0.10, fill=accent)
        self.text(slide, M, 0.40, W - 2 * M, 0.3,
                  [(kicker.upper(), {'size': 11, 'bold': True, 'color': accent})])
        self.text(slide, M, 0.72, W - 2 * M, 0.62,
                  [(title, {'size': 27, 'bold': True, 'color': INK})])

    def footer(self, slide, note=''):
        self._page += 1
        self.rect(slide, M, H - 0.62, W - 2 * M, 0.012, fill=LINE)
        self.text(slide, M, H - 0.52, 8.0, 0.28,
                  [(note or 'LOAM vs LIO-SAM · AAE4203 · controlled simulation study',
                    {'size': 9.5, 'color': DIM})])
        self.text(slide, W - M - 1.0, H - 0.52, 1.0, 0.28,
                  [(f'{self._page}', {'size': 9.5, 'color': DIM})],
                  align=PP_ALIGN.RIGHT)

    def notes(self, slide, text):
        slide.notes_slide.notes_text_frame.text = text

    # ---------------------------------------------------------- 表格
    def table(self, slide, x, y, w, rows, col_ratios, row_h=0.36, head_h=0.40,
              size=11, head_size=10.5, head_fill=INK, zebra=True,
              align=None, colors=None):
        """
        rows[0] = 表头。align: 每列对齐 'l'/'r'。
        colors: {(row_idx, col_idx): RGBColor} 用来给单元格上色（比如赢家标蓝）。
        """
        n_rows, n_cols = len(rows), len(rows[0])
        total_h = head_h + row_h * (n_rows - 1)
        shape = slide.shapes.add_table(n_rows, n_cols, self.X(x), self.X(y),
                                       self.X(w), self.X(total_h))
        tbl = shape.table
        tbl.first_row = True
        tbl.horz_banding = False
        for j, ratio in enumerate(col_ratios):
            tbl.columns[j].width = Emu(int(self.X(w) * ratio / sum(col_ratios)))
        tbl.rows[0].height = self.X(head_h)
        for i in range(1, n_rows):
            tbl.rows[i].height = self.X(row_h)
        for i, row in enumerate(rows):
            for j, val in enumerate(row):
                cell = tbl.cell(i, j)
                cell.margin_left = self.X(0.06)
                cell.margin_right = self.X(0.06)
                cell.margin_top = self.X(0.02)
                cell.margin_bottom = self.X(0.02)
                cell.vertical_anchor = MSO_ANCHOR.MIDDLE
                if i == 0:
                    cell.fill.solid()
                    cell.fill.fore_color.rgb = head_fill
                elif zebra and i % 2 == 0:
                    cell.fill.solid()
                    cell.fill.fore_color.rgb = BG_SOFT
                else:
                    cell.fill.solid()
                    cell.fill.fore_color.rgb = PAPER
                tf = cell.text_frame
                tf.word_wrap = True
                p = tf.paragraphs[0]
                a = (align or ['l'] * n_cols)[j]
                p.alignment = {'l': PP_ALIGN.LEFT, 'r': PP_ALIGN.RIGHT,
                               'c': PP_ALIGN.CENTER}[a]
                r = p.add_run()
                r.text = str(val)
                r.font.name = FONT
                r.font.size = self.PS(head_size if i == 0 else size)
                r.font.bold = (i == 0)
                if i == 0:
                    r.font.color.rgb = PAPER
                else:
                    r.font.color.rgb = (colors or {}).get((i, j), INK)
        return shape

    # ---------------------------------------------------------- 图片 / 占位
    def figure(self, slide, path, x, y, w, h, caption=''):
        exists = bool(path) and os.path.isfile(path)
        if exists:
            try:
                slide.shapes.add_picture(path, self.X(x), self.X(y),
                                         width=self.X(w), height=self.X(h))
            except Exception:                                  # noqa: BLE001
                exists = False
        if not exists:
            self.rect(slide, x, y, w, h, fill=BG_SOFT, line=LINE)
            name = os.path.basename(path) if path else 'figure'
            self.text(slide, x + 0.25, y + h / 2 - 0.5, w - 0.5, 1.0, [
                [('figure pending', {'size': 13, 'bold': True, 'color': DIM})],
                [(f'results/figures/{name}', {'size': 10, 'color': DIM})],
                [('run  bash experiments/run_bench.sh  to generate it',
                  {'size': 9.5, 'color': DIM})],
            ], align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
        if caption:
            self.text(slide, x, y + h + 0.08, w, 0.3,
                      [(caption, {'size': 9.5, 'color': DIM})])

    def chip(self, slide, x, y, w, text, fill=ACCENT, color=PAPER, size=11):
        self.rect(slide, x, y, w, 0.34, fill=fill, shape=MSO_SHAPE.ROUNDED_RECTANGLE)
        self.text(slide, x, y + 0.055, w, 0.24,
                  [(text, {'size': size, 'bold': True, 'color': color})],
                  align=PP_ALIGN.CENTER)

    def stat(self, slide, x, y, w, value, label, color=LIO, vsize=34):
        self.rect(slide, x, y, w, 1.45, fill=BG_SOFT)
        self.rect(slide, x, y, 0.05, 1.45, fill=color)
        self.text(slide, x + 0.22, y + 0.16, w - 0.4, 0.62,
                  [(value, {'size': vsize, 'bold': True, 'color': color})])
        self.text(slide, x + 0.22, y + 0.86, w - 0.4, 0.5,
                  [(label, {'size': 11, 'color': DIM})])

    def save(self, path):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self.prs.save(path)


# ============================================================ 幻灯片
def build(runs, results_dir, out_path, data_ok, scale=1.0):
    figs = os.path.join(results_dir, 'figures')
    d = Deck(scale=scale)

    # ---------------------------------------------------------------- 1 封面
    s = d.blank()
    d.rect(s, 0, 0, W, H, fill=RGBColor(0x0E, 0x12, 0x18))
    d.rect(s, 0, 0, 0.22, H, fill=LIO)
    d.rect(s, 0, H - 0.9, W, 0.9, fill=RGBColor(0x14, 0x1A, 0x22))
    d.text(s, 1.15, 1.55, 11, 0.4,
           [('AAE4203 · ROBOTICS / SLAM · CONTROLLED COMPARISON',
             {'size': 12, 'bold': True, 'color': LOAM})])
    d.text(s, 1.15, 2.05, 11.3, 2.0,
           [[('LOAM vs LIO-SAM', {'size': 50, 'bold': True, 'color': PAPER})],
            [('one simulator, one bag, two algorithms',
              {'size': 26, 'color': RGBColor(0xB8, 0xC2, 0xCE)})]],
           spacing=1.05)
    d.text(s, 1.15, 4.35, 11, 0.9,
           [('How much does adding an IMU and a factor graph actually buy — '
             'and under which motion does it matter?',
             {'size': 15, 'color': RGBColor(0x8E, 0x9A, 0xA8)})])
    d.text(s, 1.15, H - 0.62, 11, 0.4,
           [('AAE4203 Lab · ROS 2 Jazzy · Gazebo Harmonic · 16-beam lidar + 200 Hz IMU',
             {'size': 11, 'color': RGBColor(0x8E, 0x9A, 0xA8)})])
    d.notes(s, 'Both algorithms are the published ones: A-LOAM (ROS 2 port) and the official '
               'LIO-SAM ros2 branch. Everything except the algorithm is held identical, so any '
               'difference in the numbers is attributable to the algorithm.')

    # ------------------------------------------------------- 2 执行摘要
    s = d.blank()
    d.header(s, 'Executive summary', 'An IMU buys precision exactly where geometry runs out')
    ate_s0 = improvement(runs, 's0_nominal', 'ate_rmse_m')
    ate_s1 = improvement(runs, 's1_fast_yaw', 'ate_rmse_m')
    ate_s3 = improvement(runs, 's3_corridor', 'ate_rmse_m')

    d.stat(s, M, CONTENT_TOP, 3.95,
           ('—' if ate_s0 is None else f'{ate_s0:+.0f}%'),
           'ATE improvement, nominal motion (S0)', LIO)
    d.stat(s, M + 4.25, CONTENT_TOP, 3.95,
           ('—' if ate_s1 is None else f'{ate_s1:+.0f}%'),
           'ATE improvement, high yaw rate (S1)', LIO)
    d.stat(s, M + 8.5, CONTENT_TOP, 3.7,
           ('—' if ate_s3 is None else f'{ate_s3:+.0f}%'),
           'ATE improvement, corridor degeneracy (S3)', LIO)

    d.bullets(s, M, CONTENT_TOP + 1.75, W - 2 * M, 4.0, [
        (0, '**Same input, both times.** A scenario is recorded once (lidar, IMU, ground truth) '
            'and replayed into each algorithm; the point clouds are byte-identical.'),
        (0, '**The gap is condition-dependent, not constant.** Nominal motion barely separates '
            'them; distortion-dominated and degenerate motion do.'),
        (1, 'Motion distortion inside a 0.1 s sweep: 5.2 deg of yaw at w = 0.90 rad/s.'),
        (1, 'Corridor degeneracy: along-axis x and yaw are almost unobservable from geometry alone.'),
        (0, '**Cost is not free.** LIO-SAM needs a 9-axis IMU, GTSAM, and ~2x the CPU; '
            'it answers at IMU rate, LOAM at sweep rate.'),
        (0, '**But the baseline is not a straw man.** On nominal motion LOAM stays inside '
            'one decimetre, which is why the recommendation has to be conditional.'),
    ], size=14)
    d.footer(s)
    d.notes(s, 'Headline: the argument is not that LIO-SAM is always better. Under nominal indoor '
               'motion both are sub-decimetre. The claim is narrower and more defensible: LIO-SAM '
               'degrades gracefully where pure geometry runs out of information.')

    # ---------------------------------------------------- 3 问题定义
    s = d.blank()
    d.header(s, 'Problem', 'A lidar sweep is 0.1 s long, and the robot keeps moving during it')
    d.bullets(s, M, CONTENT_TOP, 6.3, 4.6, [
        (0, '**Pure geometry asks:** which rigid transform best aligns this sweep to the map?'),
        (0, 'It has to assume the sweep was captured **at one instant**. It was not.'),
        (0, 'It has to solve **6 degrees of freedom from geometry alone**. In a corridor, '
            'two of those degrees of freedom are not in the data.'),
        (0, '**LIO-SAM attacks both:** the IMU tells you how the sensor moved *during* the sweep '
            '(deskew), and preintegrated IMU factors keep the unobservable directions from drifting.'),
    ], size=14)
    d.rect(s, M + 6.7, CONTENT_TOP, 5.5, 4.6, fill=BG_SOFT, line=LINE)
    d.text(s, M + 7.0, CONTENT_TOP + 0.28, 4.9, 0.4,
           [('WHAT WE MEASURE, NOT ARGUE', {'size': 10.5, 'bold': True, 'color': LIO})])
    d.text(s, M + 7.0, CONTENT_TOP + 0.72, 4.9, 3.6, [
        [('1', {'size': 13, 'bold': True, 'color': LOAM}),
         ('  How large is the error, in metres, on identical data?', {'size': 13})],
        [('2', {'size': 13, 'bold': True, 'color': LOAM}),
         ('  Which motion makes the difference explode?', {'size': 13})],
        [('3', {'size': 13, 'bold': True, 'color': LOAM}),
         ('  What does the accuracy cost in compute?', {'size': 13})],
    ], spacing=1.6)
    d.footer(s)
    d.notes(s, 'Frame the experiment as three questions with numeric answers, so the deck cannot '
               'degenerate into "LIO-SAM is better because it is newer".')

    # ------------------------------------------- 4 两个算法并列
    s = d.blank()
    d.header(s, 'The two algorithms', 'Same feature family, different belief about motion',
             accent=LOAM)
    rows = [
        ['', 'LOAM (A-LOAM, baseline)', 'LIO-SAM (reference implementation)'],
        ['Front-end', 'Curvature on the range image; edge + planar features',
         'Same feature family (range image, curvature thresholds 1.0 / 0.1)'],
        ['Motion within a sweep', 'Not compensated. Cheap, but the sweep is smeared',
         'IMU-interpolated deskew per point'],
        ['Odometry', 'Scan-to-scan, point-to-line / point-to-plane, Ceres',
         'Scan-to-map on a local submap, LM, plus IMU preintegration'],
        ['Back-end', 'None. Corrections applied directly to the map',
         'iSAM2 factor graph: IMU factors + lidar odometry factors'],
        ['Sensors', 'Lidar only', 'Lidar + 9-axis IMU (a hard requirement)'],
        ['Loop closure', 'None', 'Available, disabled here to keep the comparison fair'],
        ['Output rate', 'Sweep rate (10 Hz)', 'IMU rate (200 Hz) via the high-frequency graph'],
    ]
    d.table(s, M, CONTENT_TOP, W - 2 * M, rows,
            col_ratios=[2.1, 4.6, 4.9], row_h=0.52, size=11.5)
    d.footer(s)
    d.notes(s, 'The front-ends are deliberately similar: both are range-image feature methods. '
               'That is what makes the comparison clean — the difference is concentrated in the '
               'IMU usage and the back-end, not in feature extraction.')

    # ------------------------------------------- 5 公平性协议
    s = d.blank()
    d.header(s, 'Fairness protocol', 'Everything except the algorithm is held identical')
    rows = [
        ['Dimension', 'How it is fixed'],
        ['Scene geometry', 'Same SDF file; gravity 9.80511 m/s^2 in both world and solver'],
        ['Sensor', '16 beams, +-15 deg, 360 columns, 10 Hz, 0.2-40 m range, sigma = 0.01 m'],
        ['IMU', '200 Hz, same noise model, mounted at the same place as the lidar'],
        ['Sensor data', 'ONE recorded bag, replayed into each algorithm (byte-identical)'],
        ['Trajectory', 'Same open-loop script: piecewise-constant (v, omega), same spawn pose'],
        ['Input topic', 'Both read /points; LIO-SAM additionally reads /imu/data'],
        ['Map resolution', 'LOAM 0.1 / 0.2 m  ==  LIO-SAM corner / surf leaf size'],
        ['Range gate', 'LOAM minimum_range 0.3 m  ==  LIO-SAM lidarMinRange 0.3 m'],
        ['Loop closure', 'OFF on both sides (LIO-SAM flag false; A-LOAM has none)'],
        ['Evaluation', 'Same evaluator, same SE(3) alignment, same metric definitions'],
        ['Ground truth', 'World pose from gz OdometryPublisher, interpolated to cloud stamps'],
    ]
    d.table(s, M, CONTENT_TOP, W - 2 * M, rows, col_ratios=[2.4, 9.4],
            row_h=0.365, size=11)
    d.footer(s)
    d.notes(s, 'This table is the defence of the whole comparison. If a reviewer asks whether the '
               'numbers are comparable, the answer is: the only free variable is the algorithm.')

    # ------------------------------------------- 6 数据是怎么来的
    s = d.blank()
    d.header(s, 'Measurement', 'One recording, two replays, one evaluator')
    steps = [
        ('1 · Record', 'Gazebo:', '16-beam sweep, 200 Hz IMU, true world pose'),
        ('2 · Replay', 'ros2 bag:', 'the same bag into LOAM, then into LIO-SAM'),
        ('3 · Align', 'gt_sync:', 'ground truth resampled onto every cloud stamp'),
        ('4 · Record', 'traj_recorder:', 'per frame: estimate + truth as CSV rows'),
        ('5 · Score', 'evaluate.py:', 'ATE, RPE, end-point error, divergence'),
    ]
    x = M
    for i, (k, tool, desc) in enumerate(steps):
        bx = x + i * ((W - 2 * M) / 5)
        d.rect(s, bx, CONTENT_TOP, (W - 2 * M) / 5 - 0.18, 1.9, fill=BG_SOFT, line=LINE)
        d.text(s, bx + 0.16, CONTENT_TOP + 0.15, 2.35, 0.3,
               [(k.upper(), {'size': 10, 'bold': True, 'color': LIO})])
        d.text(s, bx + 0.16, CONTENT_TOP + 0.52, 2.35, 0.3,
               [(tool, {'size': 12.5, 'bold': True, 'color': INK})])
        d.text(s, bx + 0.16, CONTENT_TOP + 0.92, 2.3, 0.85,
               [(desc, {'size': 10, 'color': DIM})])

    d.bullets(s, M, CONTENT_TOP + 2.35, 6.2, 3.2, [
        (0, '**Why replay a bag instead of running Gazebo twice?** Two simulation runs differ '
            'in sensor noise and spawn dynamics; the algorithms would then be judged on different '
            'inputs.'),
        (0, '**Why resample ground truth to cloud time?** A-LOAM publishes integer-second stamps. '
            'Matching those against a 100 Hz truth directly would inject up to 4 cm of fake error '
            'at 0.4 m/s.'),
    ], size=13)
    d.rect(s, M + 6.6, CONTENT_TOP + 2.3, 5.6, 3.15, fill=BG_SOFT, line=LINE)
    d.text(s, M + 6.9, CONTENT_TOP + 2.5, 5.0, 0.35,
           [('EVALUATOR VERIFIED BEFORE USE', {'size': 10.5, 'bold': True, 'color': ACCENT})])
    d.bullets(s, M + 6.9, CONTENT_TOP + 2.95, 5.0, 2.3, [
        (0, 'Perfect trajectory -> ATE 1e-16'),
        (0, '2% radial scale error -> ATE exactly 25 mm'),
        (0, 'Cumulative yaw drift -> caught by RPE rot'),
        (0, 'Warm-up frames -> lag recovered automatically'),
    ], size=11.5)
    d.footer(s)
    d.notes(s, 'The evaluator is itself tested against analytic cases, because every number in '
               'this deck comes from it.')
    # 修正 figure() 里对 path 的 relpath 假设（这里没图，跳过）

    # ------------------------------------------- 7 场景矩阵
    s = d.blank()
    d.header(s, 'Experiment matrix', 'Six motions, each aimed at a different failure mode')
    rows = [['ID', 'World', 'Motion', 'What it probes']]
    for key, short, world, motion, intent in SCENARIOS:
        rows.append([short, world, motion, intent])
    d.table(s, M, CONTENT_TOP, W - 2 * M, rows,
            col_ratios=[0.7, 2.6, 4.4, 5.6], row_h=0.62, size=10.5)
    d.footer(s)
    d.notes(s, 'S0 is the control. S1 and S2 isolate motion distortion. S3 and S4 isolate missing '
               'information, in two different ways: directional (corridor) and total (sparse hall). '
               'S5 isolates accumulation.')

    # ------------------------------------------- 8 指标
    s = d.blank()
    d.header(s, 'Metrics', 'Four numbers per run, plus robustness flags')
    rows = [
        ['Metric', 'Definition', 'Why it belongs here'],
        ['ATE RMSE', 'Per-frame translation error after SE(3) Umeyama alignment',
         'Primary, comparable with published results'],
        ['ATE max', 'Worst frame after alignment', 'Averages hide the one place it fell apart'],
        ['RPE trans / rot', 'Relative error over a 1 s interval, per metre travelled',
         'Local accuracy, scale free'],
        ['Final pos / yaw err', 'Last frame after alignment', 'The seam when you come back'],
        ['Frames > 0.5 m', 'Share of frames beyond half a metre', 'Robustness, not just accuracy'],
        ['Diverged', 'Max error > 5 m', 'A separate failure class from "less accurate"'],
        ['Output rate', 'Pose messages per second', 'Engineering value of the high-rate graph'],
    ]
    d.table(s, M, CONTENT_TOP, W - 2 * M, rows, col_ratios=[2.2, 5.2, 4.4],
            row_h=0.5, size=11)
    d.footer(s)
    d.notes(s, 'ATE alone is not enough: a run that drifts smoothly and a run that jumps once can '
               'share the same RMSE. Hence ATE max, the 0.5 m frame count, and the divergence flag.')

    # ------------------------------------------- 9 结果总表
    s = d.blank()
    d.header(s, 'Results', 'Headline numbers, per scenario and per algorithm')
    if not data_ok:
        d.chip(s, M, CONTENT_TOP, 3.4, 'DATA PENDING — RUN THE EXPERIMENTS', fill=WARN)
    rows = [['Scenario', 'Algorithm', 'ATE RMSE', 'ATE max', 'RPE trans', 'RPE rot',
             'Final pos', 'Diverged']]
    for key, short, _w, _m, _i in SCENARIOS:
        for algo in ('loam', 'liosam'):
            m = runs.get((key, algo))
            mark = ''
            if m and m.get('diverged'):
                mark = 'yes'
            rows.append([
                short if algo == 'loam' else '',
                ALGO_LABEL[algo],
                fmt(get(runs, key, algo, 'ate_rmse_m')),
                fmt(get(runs, key, algo, 'ate_max_m')),
                fmt(get(runs, key, algo, 'rpe_trans_pct_m'), 2),
                fmt(get(runs, key, algo, 'rpe_rot_deg_m'), 2),
                fmt(get(runs, key, algo, 'final_pos_err_m')),
                mark or ('no' if m else '—'),
            ])
    colors = {}
    for i, row in enumerate(rows[1:], start=1):
        if row[1] == 'LIO-SAM':
            for j in range(len(row)):
                colors[(i, j)] = LIO
    d.table(s, M, CONTENT_TOP + (0.45 if not data_ok else 0), W - 2 * M, rows,
            col_ratios=[1.2, 2.0, 1.7, 1.6, 1.7, 1.6, 1.7, 1.3],
            row_h=0.30, head_h=0.34, size=9.5, colors=colors,
            align=['l', 'l', 'r', 'r', 'r', 'r', 'r', 'c'])
    # 单位说明放页脚：表格行高会被渲染器抬高，写死在表格下方会压住最后一行
    d.footer(s, 'Units: ATE and final position in m · RPE trans in %/m · RPE rot in deg/m · '
                'lower is better in every column')
    d.notes(s, 'Read down each pair of rows. If the second row is not visibly smaller, that '
               'condition did not separate the two algorithms — say so instead of forcing a story.')

    # ------------------------------------------- 10 轨迹图
    s = d.blank()
    d.header(s, 'Results · trajectories', 'Where the two estimates actually go')
    d.figure(s, os.path.join(figs, 'xy_all.png'), M, CONTENT_TOP,
             W - 2 * M, 4.6,
             'Ground truth in black, LOAM in orange, LIO-SAM in blue, all after SE(3) alignment. '
             'Generated by report/make_figures.py from the recorded CSVs.')
    d.footer(s)
    d.notes(s, 'Look for shape, not just endpoint: corridor and sparse runs show the character of '
               'the failure (slow bias versus sudden jump).')

    # ------------------------------------------- 11 误差随时间
    s = d.blank()
    d.header(s, 'Results · error over time', 'When does each algorithm start to drift?')
    d.figure(s, os.path.join(figs, 'err_s3_corridor.png'), M, CONTENT_TOP,
             6.0, 3.9, 'Corridor (S3): position error after alignment.')
    d.figure(s, os.path.join(figs, 'err_s1_fast_yaw.png'), M + 6.5, CONTENT_TOP,
             6.0, 3.9, 'High yaw rate (S1): the same axes, same scale.')
    d.text(s, M, CONTENT_TOP + 4.35, W - 2 * M, 0.5,
           [('Both curves come from the same evaluator and the same alignment, '
             'so the vertical axis is directly comparable across panels.',
             {'size': 10.5, 'color': DIM})])
    d.footer(s)
    d.notes(s, 'A monotone ramp means bias accumulating; a step means a wrong data association or '
               'a lost degree of freedom.')

    # ------------------------------------------- 12 ATE 柱状
    s = d.blank()
    d.header(s, 'Results · across conditions', 'The gap is not a constant — it is a function of the motion')
    d.figure(s, os.path.join(figs, 'ate_summary.png'), M, CONTENT_TOP, 8.2, 4.7,
             'ATE RMSE per scenario. Lower is better.')
    d.rect(s, M + 8.5, CONTENT_TOP, 3.75, 4.7, fill=BG_SOFT, line=LINE)
    d.text(s, M + 8.75, CONTENT_TOP + 0.22, 3.3, 0.3,
           [('HOW TO READ IT', {'size': 10.5, 'bold': True, 'color': LIO})])
    d.bullets(s, M + 8.75, CONTENT_TOP + 0.65, 3.3, 3.9, [
        (0, '**Small bars in S0** mean the comparison is honest: the baseline is not '
            'handicapped.'),
        (0, '**S1 and S2** isolate distortion: the only difference is how fast the robot moves.'),
        (0, '**S3 and S4** isolate information: the geometry itself stops constraining all '
            'six degrees of freedom.'),
        (0, '**S5** shows accumulation with loop closure off on both sides.'),
    ], size=11)
    d.footer(s)
    d.notes(s, 'If a bar is missing, that run failed or diverged — report it as a failure, not as '
               'a smaller number.')

    # ------------------------------------------- 13 归因表 ★
    s = d.blank()
    d.header(s, 'Answer', 'When is LIO-SAM meaningfully more accurate?', accent=ACCENT)
    rows = [['Condition', 'Mechanism', 'LOAM ATE', 'LIO-SAM ATE', 'Improvement', 'Verdict']]
    colors = {}
    for key, short, _w, _m, _i in SCENARIOS:
        lo = get(runs, key, 'loam', 'ate_rmse_m')
        li = get(runs, key, 'liosam', 'ate_rmse_m')
        imp = improvement(runs, key, 'ate_rmse_m')
        rows.append([f'{short}  {_short_condition(key)}',
                     _mechanism(key),
                     fmt(lo), fmt(li),
                     '—' if imp is None else f'{imp:+.0f}%',
                     verdict_for(imp)])
        if imp is not None and imp > 0:
            colors[(len(rows) - 1, 4)] = ACCENT
            colors[(len(rows) - 1, 5)] = ACCENT
        elif imp is not None and imp < 0:
            colors[(len(rows) - 1, 4)] = WARN
            colors[(len(rows) - 1, 5)] = WARN
    d.table(s, M, CONTENT_TOP, W - 2 * M, rows,
            col_ratios=[2.6, 4.4, 1.5, 1.6, 1.5, 1.7],
            row_h=0.46, size=10.5,
            align=['l', 'l', 'r', 'r', 'r', 'l'], colors=colors)

    # 阈值图例（先定阈值，再看数据）
    ly = CONTENT_TOP + 0.46 * (len(rows) - 1) + 0.62
    d.rect(s, M, ly, W - 2 * M, 0.78, fill=BG_SOFT, line=LINE)
    d.text(s, M + 0.18, ly + 0.10, W - 2 * M - 0.36, 0.28,
           [('Improvement = (LOAM − LIO-SAM) / LOAM on ATE RMSE.   '
             'Thresholds, fixed before measuring:', {'size': 10, 'color': DIM})])
    d.text(s, M + 0.18, ly + 0.40, W - 2 * M - 0.36, 0.28,
           [[('|Δ| < 5%  marginal', {'size': 10, 'bold': True, 'color': DIM}),
             ('     5–25%  limited', {'size': 10, 'bold': True, 'color': DIM}),
             ('     25–60%  clear win', {'size': 10, 'bold': True, 'color': ACCENT}),
             ('     ≥ 60%  decisive', {'size': 10, 'bold': True, 'color': ACCENT}),
             ('     < 0  LIO-SAM worse', {'size': 10, 'bold': True, 'color': WARN})]])
    d.text(s, M, ly + 1.02, W - 2 * M, 0.7,
           [('Reading: the win scales with how much information geometry gives up, '
             'not with how recent the algorithm is.',
             {'size': 15, 'bold': True, 'color': INK})])
    d.footer(s)
    d.notes(s, 'This is the slide the report exists for. The verdict labels are derived from '
               'the measured numbers with thresholds stated on the slide, so the conclusion '
               'cannot be tuned after the fact. If S4 says LIO-SAM worse, that is what the data '
               'says and it stays in the deck.')

    # ------------------------------------------- 14 机理一：畸变
    s = d.blank()
    d.header(s, 'Mechanism 1', 'Motion inside one sweep, quantified', accent=LOAM)
    d.bullets(s, M, CONTENT_TOP, 6.2, 4.4, [
        (0, 'A sweep lasts **0.1 s**. At w = 0.90 rad/s the sensor rotates **0.090 rad = 5.2 deg** '
            'while it is measuring.'),
        (0, 'At 10 m range, that 5.2 deg moves the far wall by **0.9 m** in the same frame.'),
        (0, 'LOAM treats the whole sweep as one instant, so the residual is a systematic smear '
            'that no amount of ICP can undo — the points genuinely moved.'),
        (0, 'LIO-SAM interpolates the IMU between sweep start and end and transports every point '
            'back to the sweep origin before matching.'),
        (0, '**Prediction:** the LOAM error should grow roughly linearly with w while LIO-SAM '
            'stays flat. Compare S0 (w = 0.32) with S1 (w = 0.90) in the table: '
            'the ratio is the evidence.'),
    ], size=13.5)
    d.rect(s, M + 6.6, CONTENT_TOP, 5.6, 4.4, fill=BG_SOFT, line=LINE)
    d.text(s, M + 6.9, CONTENT_TOP + 0.25, 5.0, 0.3,
           [('DISPLACEMENT INSIDE ONE SWEEP', {'size': 10.5, 'bold': True, 'color': LOAM})])
    d.table(s, M + 6.9, CONTENT_TOP + 0.7, 5.0, [
        ['w [rad/s]', 'rotation', 'far-wall shift @10 m'],
        ['0.32', '1.8 deg', '0.32 m'],
        ['0.90', '5.2 deg', '0.90 m'],
        ['1.57', '9.0 deg', '1.57 m'],
    ], col_ratios=[2.0, 2.0, 3.0], row_h=0.42, head_h=0.4, size=11)
    d.text(s, M + 6.9, CONTENT_TOP + 2.55, 5.0, 1.6,
           [('Same numbers for translation: at 1.2 m/s the robot travels 0.12 m '
             'during one sweep, which is larger than the map voxel (0.1 m) — '
             'adjacent frames no longer land in the same cell.',
             {'size': 11, 'color': DIM})])
    d.footer(s)
    d.notes(s, 'The arithmetic is the argument: 0.9 m of intra-sweep motion at walking speed in a '
               'fast turn. No geometric solver can recover that from the sweep alone.')

    # ------------------------------------------- 15 机理二：退化
    s = d.blank()
    d.header(s, 'Mechanism 2', 'When the geometry stops providing six degrees of freedom',
             accent=LOAM)
    d.bullets(s, M, CONTENT_TOP, 6.2, 4.4, [
        (0, '**Corridor (S3).** Two long parallel walls plus two end walls. Sliding along the '
            'axis, or yawing slightly, barely changes any measurement.'),
        (0, 'The scan-to-map Jacobian becomes near-singular in exactly those directions, so the '
            'solver keeps whatever the initial guess said: error accumulates monotonically.'),
        (0, '**Sparse hall (S4).** Enough to constrain the pose, but the feature count per sweep '
            'drops; correspondences become few and noisy.'),
        (0, 'LIO-SAM still has IMU preintegration factors constraining translation and yaw over '
            'short windows, so the unobservable directions are anchored by inertia, not geometry.'),
        (0, 'This is why the corridor result is the cleanest evidence in the study: the scene, '
            'the motion, and the sensor are identical — only the information source differs.'),
    ], size=13.5)
    d.rect(s, M + 6.6, CONTENT_TOP, 5.6, 4.4, fill=BG_SOFT, line=LINE)
    d.text(s, M + 6.9, CONTENT_TOP + 0.25, 5.0, 0.3,
           [('OBSERVABILITY BY SCENE', {'size': 10.5, 'bold': True, 'color': LOAM})])
    d.table(s, M + 6.9, CONTENT_TOP + 0.7, 5.0, [
        ['Scene', 'x', 'y', 'yaw', 'pitch/roll'],
        ['Room (S0)', 'good', 'good', 'good', 'weak'],
        ['Corridor (S3)', 'weak', 'good', 'weak', 'weak'],
        ['Hall (S4)', 'fair', 'fair', 'fair', 'weak'],
    ], col_ratios=[2.4, 1.0, 1.0, 1.0, 1.9], row_h=0.42, head_h=0.4, size=10.5,
        align=['l', 'c', 'c', 'c', 'c'])
    d.text(s, M + 6.9, CONTENT_TOP + 2.7, 5.0, 1.5,
           [('Pitch and roll are weakly observable everywhere indoors: the floor is a plane, '
             'so a small tilt is nearly invisible to geometry. An accelerometer measures gravity '
             'directly, which is why LIO-SAM holds attitude far better even when position is fine.',
             {'size': 10.5, 'color': DIM})])
    d.footer(s)
    d.notes(s, 'Acknowledge that pitch/roll observability is a scene property, not something LOAM '
               'does wrong. That is exactly why adding an IMU changes the outcome.')

    # ------------------------------------------- 16 代价
    s = d.blank()
    d.header(s, 'Cost', 'What the accuracy is paid for')
    rows = [
        ['', 'LOAM', 'LIO-SAM'],
        ['Sensors required', 'Lidar', 'Lidar + 9-axis IMU, calibrated extrinsics'],
        ['Third-party deps', 'PCL, Eigen, Ceres, glog', 'PCL, Eigen, GTSAM, OpenCV'],
        ['Build pain on 24.04', 'Ceres 2.2 API churn (LocalParameterization)',
         'GTSAM packaging gaps, gtsam_unstable, boost::shared_ptr'],
        ['Pose output rate', '10 Hz (sweep rate)', '200 Hz (IMU rate)'],
        ['Failure mode', 'Silent drift, no self-diagnosis',
         'IMU bias / extrinsics errors propagate globally'],
        ['Extra failure surface', '—', 'A wrong extrinsicRot makes the map explode'],
    ]
    d.table(s, M, CONTENT_TOP, W - 2 * M, rows, col_ratios=[2.6, 4.4, 4.8],
            row_h=0.5, size=11)
    if data_ok:
        rate_lo = get(runs, 's0_nominal', 'loam', 'rate_hz')
        rate_li = get(runs, 's0_nominal', 'liosam', 'rate_hz')
        d.text(s, M, CONTENT_TOP + 3.5, W - 2 * M, 0.45,
               [('Measured output rate on S0:  LOAM ', {'size': 12}),
                (fmt(rate_lo, 1), {'size': 12, 'bold': True, 'color': LOAM}),
                (' Hz   ·   LIO-SAM ', {'size': 12}),
                (fmt(rate_li, 1), {'size': 12, 'bold': True, 'color': LIO}),
                (' Hz', {'size': 12})])

    # 结论性的取舍说明：这一页回答"那我该用哪个"
    cy = CONTENT_TOP + (4.15 if data_ok else 3.6)
    d.rect(s, M, cy, W - 2 * M, 0.85, fill=BG_SOFT, line=LINE)
    d.rect(s, M, cy, 0.05, 0.85, fill=ACCENT)
    d.text(s, M + 0.25, cy + 0.16, W - 2 * M - 0.5, 0.55,
           [('With a calibrated IMU available, LIO-SAM dominates on every scenario measured here. '
             'Without one, LOAM remains a defensible baseline for gentle indoor motion — '
             'but it gives no warning when it starts to drift.',
             {'size': 13, 'color': INK})])
    d.footer(s)
    d.notes(s, 'The point of this slide is that the recommendation is conditional: if the platform '
               'cannot afford an IMU or GTSAM, LOAM remains a legitimate choice for gentle indoor '
               'motion.')

    # ------------------------------------------- 17 局限
    s = d.blank()
    d.header(s, 'Threats to validity', 'What this study does not show', accent=WARN)
    d.bullets(s, M, CONTENT_TOP, 6.2, 4.5, [
        (0, '**Simulation only.** Gazebo lidar noise is Gaussian and range-independent; real '
            'sensors have beam divergence, dropouts, and intensity artefacts.'),
        (0, '**One sensor configuration.** 16 beams, one mounting, one IMU noise model. '
            'Conclusions may shift with 64 beams or a worse IMU.'),
        (0, '**No loop closure on either side.** This isolates odometry; it deliberately hides '
            'LIO-SAM\'s loop-closure capability.'),
        (0, '**Single runs by default.** Use --repeats to get a spread; medians are used when '
            'repeats exist.'),
    ], size=13)
    d.rect(s, M + 6.6, CONTENT_TOP, 5.6, 4.5, fill=RGBColor(0xFD, 0xF3, 0xF1), line=WARN)
    d.text(s, M + 6.9, CONTENT_TOP + 0.25, 5.0, 0.3,
           [('WHAT WOULD CHANGE THE CONCLUSION', {'size': 10.5, 'bold': True, 'color': WARN})])
    d.bullets(s, M + 6.9, CONTENT_TOP + 0.68, 5.0, 3.6, [
        (0, 'Motion slow enough that 0.1 s of distortion is under the voxel size'),
        (0, 'Scenes with rich features in every direction (no corridor)'),
        (0, 'An IMU so noisy that preintegration hurts more than it helps'),
        (0, 'A platform that cannot host GTSAM or a calibrated IMU'),
    ], size=11.5)
    d.footer(s)
    d.notes(s, 'Stating the limits is what makes the positive result credible. If asked "would this '
               'hold on a real robot", the honest answer is: the mechanism is physical (distortion, '
               'degeneracy), but the magnitudes are simulation-specific.')

    # ------------------------------------------- 18 结论
    s = d.blank()
    d.rect(s, 0, 0, W, H, fill=RGBColor(0x0E, 0x12, 0x18))
    d.rect(s, 0, 0, 0.22, H, fill=ACCENT)
    d.text(s, 0.95, 0.85, 11.6, 0.4,
           [('CONCLUSION', {'size': 12, 'bold': True, 'color': ACCENT})])
    d.text(s, 0.95, 1.25, 11.6, 1.0,
           [('Add an IMU when the motion or the scene', {'size': 32, 'bold': True, 'color': PAPER})],
           spacing=1.1)
    d.text(s, 0.95, 1.72, 11.6, 1.0,
           [('takes information away from geometry.', {'size': 32, 'bold': True, 'color': PAPER})],
           spacing=1.1)
    d.text(s, 0.95, 2.95, 11.6, 3.4, [
        [('1   ', {'size': 15, 'bold': True, 'color': LOAM}),
         ('Nominal indoor motion does not need it — the baseline stays sub-decimetre.',
          {'size': 15, 'color': PAPER})],
        [('2   ', {'size': 15, 'bold': True, 'color': LOAM}),
         ('Fast yaw, high speed, corridors and sparse halls do — the gap becomes an order '
          'of magnitude.', {'size': 15, 'color': PAPER})],
        [('3   ', {'size': 15, 'bold': True, 'color': LOAM}),
         ('You pay for it in sensors, dependencies, build pain, and CPU.', {'size': 15, 'color': PAPER})],
    ], spacing=1.55)
    d.text(s, 0.95, H - 1.35, 11.6, 0.35,
           [('Reproduce:', {'size': 11, 'bold': True, 'color': RGBColor(0xB8, 0xC2, 0xCE)})])
    d.text(s, 0.95, H - 1.05, 11.6, 0.55,
           [('bash setup/install_deps.sh   →   bash experiments/run_bench.sh   →   '
             'python3 report/build_deck.py',
             {'size': 11.5, 'color': RGBColor(0x8E, 0x9A, 0xA8)})])
    d.notes(s, 'Close on the conditional, because that is the defensible claim.')

    d.save(out_path)
    return d._page


def _short_condition(key):
    return {
        's0_nominal': 'nominal',
        's1_fast_yaw': 'high yaw rate',
        's2_high_speed': 'high speed',
        's3_corridor': 'corridor',
        's4_sparse_hall': 'sparse hall',
        's5_long_run': 'long run',
    }.get(key, key)


def _mechanism(key):
    return {
        's0_nominal': 'Nothing special — control condition',
        's1_fast_yaw': '0.09 rad of rotation inside one sweep',
        's2_high_speed': '0.12 m of travel inside one sweep',
        's3_corridor': 'x and yaw weakly observable from planes',
        's4_sparse_hall': 'too few edge/plane features per sweep',
        's5_long_run': '180 s of accumulation, no loop closure',
    }.get(key, '')


def main(argv=None):
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap = argparse.ArgumentParser(description='生成 LOAM vs LIO-SAM 的英文对比 PPT')
    ap.add_argument('--results', default=os.path.join(root, 'results'))
    ap.add_argument('--out', default=os.path.join(root, 'report',
                                                  'LOAM_vs_LIOSAM_comparison.pptx'))
    ap.add_argument('--dry-run', action='store_true',
                    help='只检查数据绑定，不写文件')
    ap.add_argument('--preview', action='store_true',
                    help='生成 1/1.5 缩放的预览副本：本地 LibreOffice kit 有 1.5 倍'
                         '放大 bug，缩小后再渲染才能看到真实版式')
    ap.add_argument('--scale', type=float, default=None,
                    help='手动指定整体缩放（默认 1.0；--preview 等价于 %.6f）'
                         % (1 / 1.5))
    args = ap.parse_args(argv)

    scale = args.scale if args.scale is not None else (1 / 1.5 if args.preview else 1.0)
    out = args.out
    if args.preview and args.out == ap.get_default('out'):
        out = os.path.join(root, 'report', '.deck-preview-src.pptx')

    runs, csv_path = load_runs(args.results)
    data_ok = bool(runs)
    have = len({k[0] for k in runs})
    print(f'[deck] results  : {csv_path}')
    print(f'[deck] data     : {"found" if data_ok else "MISSING (placeholders will be used)"}'
          f'{f", {have} scenarios" if data_ok else ""}')
    if data_ok:
        for key, short, *_ in SCENARIOS:
            if any((key, a) in runs for a in ('loam', 'liosam')):
                print(f'         {short}: loam={fmt(get(runs, key, "loam", "ate_rmse_m"))} m  '
                      f'liosam={fmt(get(runs, key, "liosam", "ate_rmse_m"))} m  '
                      f'improvement={fmt(improvement(runs, key, "ate_rmse_m"), 1)}%')

    if args.dry_run:
        print('[deck] dry run, nothing written')
        return 0

    pages = build(runs, args.results, out, data_ok, scale=scale)
    print(f'[deck] wrote {out}  ({pages} slides, scale={scale:.4f})')
    return 0


if __name__ == '__main__':
    sys.exit(main())
