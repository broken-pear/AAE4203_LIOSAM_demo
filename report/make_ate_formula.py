"""把 ATE 一页的公式渲染成图片（用于 PPT）。

用 matplotlib 的 mathtext —— 不需要安装 LaTeX，输出的 PNG 在高 dpi 下
投影依然锐利。两块分别对应幻灯片上的第 2 步和第 3 步：

  ate_formula_align.png  互协方差 → SVD → R, t（含防镜像的 det 项）
  ate_formula_error.png  逐帧残差 → RMSE

字号按"放在幻灯片上的实际尺寸"设定（图宽即版面宽），这样投影时不会缩太小。
"""
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIGS = os.path.join(ROOT, 'results', 'figures')

matplotlib.rcParams['mathtext.fontset'] = 'cm'      # Computer Modern，论文观感

ALIGN = [
    r'$H \;=\; \sum_{i=1}^{N}\,(\hat{p}_i-\hat{\bar{p}})\,(p_i-\bar{p})^{\mathsf{T}}$',
    r'$H = U\Sigma V^{\mathsf{T}} \quad\Rightarrow\quad '
    r'R = V\,\mathrm{diag}\!\left(1,\,1,\,\det(VU^{\mathsf{T}})\right)U^{\mathsf{T}}$',
    r'$t \;=\; \bar{p} \;-\; R\,\hat{\bar{p}}$',
]

ERROR = [
    r'$e_i \;=\; \left\|\, p_i - \left(R\,\hat{p}_i + t\right) \right\|_2$',
    r'$\mathrm{ATE} \;=\; \sqrt{\dfrac{1}{N}\sum_{i=1}^{N} e_i^{\,2}}$',
]


def render(lines, out, figsize, fontsize, pad=0.06, colour='#262626'):
    fig = plt.figure(figsize=figsize, dpi=340)
    fig.patch.set_facecolor('white')
    n = len(lines)
    for k, tex in enumerate(lines):
        fig.text(0.012, 1 - (k + 0.5) / n, tex, fontsize=fontsize,
                 color=colour, va='center', ha='left')
    fig.savefig(out, format='png', facecolor='white',
                bbox_inches='tight', pad_inches=pad)
    plt.close(fig)
    print('wrote', os.path.basename(out))


def main():
    os.makedirs(FIGS, exist_ok=True)
    # 与幻灯片上的版面宽度一致（约 4.4 in），两行公式
    render(ALIGN, os.path.join(FIGS, 'ate_formula_align.png'),
           figsize=(4.35, 1.18), fontsize=13.0)
    # 右侧一列：残差与 ATE，竖排更像"定义"
    render(ERROR, os.path.join(FIGS, 'ate_formula_error.png'),
           figsize=(3.05, 1.02), fontsize=15)


if __name__ == '__main__':
    main()
