#!/usr/bin/env python3
"""Render evidence-backed Chinese 16:9 figures for the Nov 3–4 report.

Reads saved analysis only. Does not initialize ROS or touch navigation settings.
"""
import csv
import hashlib
import json
from pathlib import Path
import shutil
import zipfile

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.patches import Rectangle, FancyArrowPatch, Circle, Wedge
from PIL import Image
import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
A = ROOT / 'docs/analysis'
IMG = OUT / 'images'
DATA = OUT / 'source_data'
IMG.mkdir(exist_ok=True)
DATA.mkdir(exist_ok=True)
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
FONT = font_manager.FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc').get_name()
plt.rcParams.update({'font.family': FONT, 'font.size': 15, 'axes.unicode_minus': False,
                     'svg.fonttype': 'path', 'pdf.fonttype': 42,
                     'axes.spines.top': False, 'axes.spines.right': False})
BG, INK, MUTED = '#F7FAFC', '#17344B', '#617587'
BEFORE, AFTER, BLUE, LINE = '#D97558', '#07867F', '#3069B0', '#DCE5EC'
SOURCES = {
    'visualization': 'mppi_frequency_2026-11-03/metrics.json',
    'heightjump': 'heightjump_implementation_2026-11-03/metrics.json',
    'old_bag': 'indoor_spin_2026-11-02_20-19-46/metrics.json',
    'gravity': 'initial_pose_tf_review_2026-11-04/metrics.json',
    'cloud': 'navigation_cloud_2026-11-04/metrics.json',
    'origin': 'elevation_sensor_origin_2026-11-04/bag_comparison.json',
    'gpu': 'elevation_sensor_origin_2026-11-04/gpu_comparison.json',
}
metrics = {name: json.loads((A/path).read_text()) for name, path in SOURCES.items()}
for name, path in SOURCES.items():
    shutil.copyfile(A/path, DATA/f'{name}.json')
csv_path = A/'elevation_sensor_origin_2026-11-04/bag_comparison_frames.csv'
rows = list(csv.DictReader(csv_path.open()))
shutil.copyfile(csv_path, DATA/'origin_frames.csv')
pages = []


def text(ax, x, y, s, size=20, color=INK, **kw):
    return ax.text(x, y, s, fontsize=size, color=color, va='center', **kw)


def page(number, title, subtitle, tag, source):
    fig = plt.figure(figsize=(16, 9), facecolor=BG)
    ax = fig.add_axes([0, 0, 1, 1]); ax.set(xlim=(0, 16), ylim=(0, 9)); ax.axis('off')
    ax.add_patch(Rectangle((.65, 8.05), .08, .65, color=AFTER))
    text(ax, .95, 8.38, title, 30)
    text(ax, .95, 7.78, subtitle, 16, MUTED)
    text(ax, 15.3, 8.48, tag, 12, BLUE, ha='right')
    ax.plot([.7, 15.3], [.68, .68], color=LINE, lw=1)
    text(ax, .75, .35, source, 10.5, MUTED)
    text(ax, 15.25, .35, f'{number:02d} / 12', 11, MUTED, ha='right')
    return fig, ax


def chart(fig, rect):
    ax = fig.add_axes(rect, facecolor=BG)
    ax.tick_params(colors=MUTED, labelsize=13)
    for spine in ax.spines.values(): spine.set_color(LINE)
    ax.set_axisbelow(True)
    return ax


def arrow(ax, start, end, color=BLUE, lw=2.5):
    ax.add_patch(FancyArrowPatch(start, end, arrowstyle='-|>', mutation_scale=17, lw=lw, color=color))


def rule(ax, x, y, w=3.8):
    ax.plot([x, x+w], [y, y], color=LINE, lw=1.2)


def save(fig, name, title, takeaway, evidence):
    path = IMG/name
    fig.canvas.draw()
    renderer = fig.canvas.get_renderer()
    outside_ticks = set()
    for axes in fig.axes:
        for axis, limits in [(axes.xaxis, axes.get_xlim()), (axes.yaxis, axes.get_ylim())]:
            for tick in axis.get_major_ticks()+axis.get_minor_ticks():
                if not min(limits) <= tick.get_loc() <= max(limits):
                    outside_ticks.update([id(tick.label1), id(tick.label2)])
    for item in fig.findobj(matplotlib.text.Text):
        if not item.get_visible() or not item.get_text() or id(item) in outside_ticks:
            continue
        bounds = item.get_window_extent(renderer)
        width, height = fig.canvas.get_width_height()
        assert bounds.x0 >= -2 and bounds.y0 >= -2 and bounds.x1 <= width+2 and bounds.y1 <= height+2, (name, item.get_text(), bounds)
    fig.savefig(path.with_suffix('.png'), dpi=160, facecolor=BG)
    fig.savefig(path.with_suffix('.svg'), facecolor=BG)
    svg = path.with_suffix('.svg')
    svg.write_text('\n'.join(line.rstrip() for line in svg.read_text().splitlines())+'\n')
    pdf.savefig(fig, facecolor=BG)
    plt.close(fig)
    pages.append(dict(name=name, title=title, takeaway=takeaway, evidence=evidence))


with PdfPages(OUT/'导航优化汇报_12页.pdf') as pdf:
    # 01: headline results from separately controlled experiments.
    f, ax = page(1, '两天工作：降低计算负载，修正空间基准',
                 'MPPI + 高程图导航｜2026.11.03—11.04', '工作总览', '来源：显示、HeightJump、定位、高程原点四组独立记录；不把不同实验合并为整机提速。')
    for x, day, topic, detail in [(1., '11.03', '计算效率', '轨迹显示降载 / 九点批量查询'),
                                (8.3, '11.04', '几何正确性', '重力初始化 / 物理传感器原点')]:
        text(ax, x, 6.8, day, 34, BLUE)
        text(ax, x+2., 6.82, topic, 25)
        text(ax, x, 6.05, detail, 19, MUTED)
        rule(ax, x, 5.5, 6.)
    cards = [(1., '84.7%', 'HeightJump 平地评分耗时下降', '25.80 → 3.95 ms｜配对内核基准'),
             (6.1, '62.8%', '优化器 + 显示耗时下降', '40.33 → 15.00 ms｜离线完整链路'),
             (11., '重力 + TF', '地图与融合原点语义修正', '定位姿态保留 / 注册点云重表达')]
    for x, value, label, detail in cards:
        text(ax, x, 4.25, value, 38, AFTER)
        text(ax, x, 3.3, label, 17)
        text(ax, x, 2.65, detail, 13, MUTED)
    text(ax, 1., 1.35, '交付：源码、配置、回归测试、复现脚本、对照数据', 21)
    save(f, '01_overview', '工作总览', '计算效率与空间几何分别改进；现场导航仍需复测。', ['heightjump', 'origin', 'gravity'])

    # 02: observed aborts and operator workflow are not a proof of sensor latency.
    old = metrics['old_bag']
    f, ax = page(2, '转圈分析：降频与恢复旋转是两条证据',
                 '旧包 nav_elevation_2026-11-02_20-19-46｜92.23 s', '旧包 + 日志', '来源：indoor_spin_2026-11-02_20-19-46；包内没有手动/导航模式切换时刻。')
    text(ax, 1., 6.8, f"{old['active_mppi_rate_hz']:.2f} Hz", 40, BEFORE)
    text(ax, 1., 6.05, '活跃优化输出｜目标 20 Hz', 18)
    text(ax, 6.45, 6.8, '570 次', 40, BEFORE)
    text(ax, 6.45, 6.05, '控制周期超时告警', 17)
    text(ax, 11.05, 6.8, '27–54 s', 34, BEFORE)
    text(ax, 11.05, 6.05, '原始IMU时间差，非底盘延迟', 14.5)
    time_ax = chart(f, [.08, .40, .83, .18])
    time_ax.set(xlim=(-1, 94), ylim=(0, 1), yticks=[], xlabel='录包相对时间 / s')
    segments = [(0, 20.888, BLUE, '跟踪'), (20.888, 40.954, BLUE, '跟踪'),
                (40.978, 50.986, BEFORE, '旋转'), (50.999, 56.061, '#D5A549', '等待'),
                (56.061, 92.229, BLUE, '跟踪')]
    for a, b, c, label in segments:
        time_ax.barh(.45, b-a, left=a, height=.34, color=c)
        time_ax.text((a+b)/2, .45, label, ha='center', va='center', color='white', fontsize=13)
    for t in [.351, 20.888, 40.954]:
        time_ax.plot(t, .85, 'v', color=BEFORE, ms=9)
    time_ax.text(20.5, 1.04, '3 次 Failed to make progress', ha='center', color=BEFORE, fontsize=15)
    text(ax, 1., 2.55, '提前发目标 → 手动等待数十秒 → 无进展检查 → 可能进入旋转恢复', 20)
    text(ax, 1., 1.7, '建议：稳定定位 → 取消旧任务 → 切导航 → 发新目标', 22, AFTER)
    text(ax, 1., 1.12, '静止反馈不能单独证明定位延迟；该包没有优化器计算失败日志。', 15, MUTED)
    save(f, '02_diagnosis', '旧包问题定位', '确认存在计算降频和无进展后的旋转恢复；不能唯一归因于底盘延迟。', ['old_bag'])

    # 03: plot directly measured totals, rather than adding independent medians.
    v = metrics['visualization']
    f, ax = page(3, '优化一：降低轨迹显示负载',
                 '保留 visualize=true；优化器仍评估 1000 条轨迹 × 56 步', '离线性能对照', '来源：mppi_frequency_2026-11-03/metrics.json；预热10轮、统计60轮，含实际DDS订阅者。')
    c = chart(f, [.09, .32, .53, .45])
    vals = [v['current_visualization']['stages']['compute_plus_visualization']['p50_ms'],
            v['sparse_visualization']['stages']['compute_plus_visualization']['p50_ms']]
    c.barh([1, 0], vals, color=[BEFORE, AFTER], height=.48)
    c.set(yticks=[1, 0], yticklabels=['原显示配置', '降载后'], xlim=(0, 84), xlabel='优化器 + 显示 / ms，P50')
    c.axvline(50, color=BLUE, ls='--', lw=1.5)
    c.text(50.8, 1.43, '20 Hz预算：50 ms', color=BLUE, fontsize=13)
    for y, val in zip([1, 0], vals): c.text(val+1, y, f'{val:.2f}', va='center', fontsize=21, color=INK)
    text(ax, 10.8, 6.15, '−43.2%', 44, AFTER)
    text(ax, 10.8, 5.35, '合计耗时下降', 21)
    text(ax, 10.8, 4.35, '显示轨迹步长  5 → 20', 19)
    text(ax, 10.8, 3.6, '显示时间步长  3 → 5', 19)
    text(ax, 1.35, 1.65, '70.29 → 39.93 ms', 30, AFTER)
    text(ax, 8.3, 1.65, '只减少显示点，不减少规划采样', 21)
    save(f, '03_visualization', '轨迹显示降载', '显示 stride 调整使离线优化加显示 P50 从70.29降至39.93 ms。', ['visualization'])

    # 04: same-process paired microbenchmark across four map conditions.
    h = metrics['heightjump']
    f, ax = page(4, '优化二：HeightJump 九点批量查询',
                 '复用地图几何；每轴只算三个位置；unknown 允许时跳过无效豁免计算', '同进程配对基准', '来源：heightjump_implementation_2026-11-03；1000×56预测点，预热10轮、配对统计100轮。')
    c = chart(f, [.12, .31, .54, .45])
    names = ['平地', '台阶', 'unknown 拒绝', 'unknown 允许']
    keys = ['0', '1', '3', '4']; y = np.arange(4)[::-1]
    before = [h['paired'][k]['prepared_rollout']['p50_ms'] for k in keys]
    after = [h['paired'][k]['batched_rollout']['p50_ms'] for k in keys]
    c.barh(y+.16, before, .29, color=BEFORE, label='原标量查询')
    c.barh(y-.16, after, .29, color=AFTER, label='批量查询')
    c.set(yticks=y, yticklabels=names, xlim=(0, 30), xlabel='评分耗时 / ms，P50')
    c.legend(loc='lower left', bbox_to_anchor=(0, 1.015), ncol=2, fontsize=12, frameon=False)
    for yy, a, b in zip(y, before, after):
        c.text(a+.3, yy+.16, f'{a:.2f}', va='center', fontsize=12)
        c.text(b+.3, yy-.16, f'{b:.2f}', va='center', fontsize=12)
    text(ax, 11.45, 6.25, '−84.7%', 39, AFTER)
    text(ax, 11.45, 5.45, '平地评分耗时', 20)
    text(ax, 11.45, 4.2, '242,304', 32, BLUE)
    text(ax, 11.45, 3.5, '组差分结果一致', 19)
    text(ax, 1.1, 1.65, '独立九点查询 → 三个 x + 三个 y 索引 → 同一判定逻辑', 23)
    text(ax, 1.1, 1.08, '地形阈值、unknown策略、提前拒绝与地图健康保护保持原语义。', 15, MUTED)
    save(f, '04_heightjump', 'HeightJump 批量查询', '平地评分25.80→3.95 ms；24张地图242304组差分评估一致。', ['heightjump'])

    # 05: throughput and periodic timing are deliberately shown separately.
    full = h['full_mppi']
    f, ax = page(5, '完整 MPPI：计算余量增加，周期抖动仍在',
                 '相同1000×56、相同稀疏显示配置；插件更新前后分别测量', '离线完整链路', '来源：heightjump_implementation_2026-11-03/full_mppi*.log；不含真实建图、costmap线程和底盘。')
    c = chart(f, [.08, .35, .40, .38])
    labels = ['优化器', '优化器 + 显示']; x = np.arange(2)
    oldv = [full['before'][k]['p50_ms'] for k in ['optimizer', 'compute_plus_visualization']]
    newv = [full['after'][k]['p50_ms'] for k in ['optimizer', 'compute_plus_visualization']]
    c.bar(x-.18, oldv, .32, color=BEFORE, label='更新前')
    c.bar(x+.18, newv, .32, color=AFTER, label='更新后')
    c.set(xticks=x, xticklabels=labels, ylim=(0, 49), ylabel='持续吞吐测试 / ms，P50')
    c.legend(frameon=False, fontsize=12)
    for xx, a, b in zip(x, oldv, newv):
        c.text(xx-.18, a+1, f'{a:.2f}', ha='center', fontsize=14)
        c.text(xx+.18, b+1, f'{b:.2f}', ha='center', fontsize=14)
    c2 = chart(f, [.61, .35, .31, .38])
    c2.set(xlim=(0, 60), ylim=(-.6, 1.6), yticks=[1, 0], yticklabels=['更新前', '更新后'], xlabel='50 ms周期测试：计算 + 显示 / ms')
    for yy, key, col in [(1, 'periodic_before', BEFORE), (0, 'periodic_after', AFTER)]:
        p50 = full[key]['compute_plus_visualization']['p50_ms']
        p95 = full[key]['compute_plus_visualization']['p95_ms']
        c2.barh(yy, p50, height=.38, color=col)
        c2.plot([p50, p95], [yy, yy], color=INK, lw=2)
        c2.plot(p95, yy, '|', ms=14, color=INK)
        c2.text(p95+1, yy, f'{p95:.2f}\nP95', va='center', fontsize=12)
    c2.axvline(50, color=BLUE, ls='--')
    text(ax, 1., 1.8, '40.33 → 15.00 ms｜P50下降62.8%', 24, AFTER)
    text(ax, 1., 1.15, '周期完成间隔 P95：51.97 → 56.46 ms；未证明现场稳定20 Hz，也未证明抖动改善。', 16, MUTED)
    save(f, '05_full_mppi', '完整 MPPI 对照', '完整优化加显示耗时下降；完成间隔抖动没有同步改善证据。', ['heightjump'])

    # 06: exact mathematical illustration, explicitly not measured robot accuracy.
    tilt = metrics['gravity']['tilt_example']['floor_height_change_over_2m']
    f, ax = page(6, '定位修正：地图水平基准来自重力',
                 '手动点击设置 x / y / yaw；真实机身 roll / pitch 与三维高度继续保留', '合成几何 + ROS验证', '来源：initial_pose_tf_review / initial_pose_tf_fix；10°是人为复现条件，不代表旧包实测姿态误差。')
    c = chart(f, [.10, .35, .48, .40])
    distance = np.linspace(0, 2, 80)
    c.plot(distance*np.cos(np.deg2rad(10)), -distance*np.sin(np.deg2rad(10)), color=BEFORE, lw=4, label='旧：点击瞬间机身当水平')
    c.plot(distance, np.zeros_like(distance), color=AFTER, lw=4, label='新：重力定义地图向上')
    c.set(xlim=(-.05, 2.15), ylim=(-.42, .12), xlabel='地图水平投影 / m', ylabel='地面高度 / m')
    c.grid(axis='y', color=LINE)
    c.legend(loc='lower left', fontsize=11, frameon=False)
    c.annotate(f'{tilt*1000:.1f} mm', xy=(1.97, -.3473), xytext=(1.22, -.15), color=BEFORE,
               fontsize=18, arrowprops=dict(arrowstyle='->', color=BEFORE))
    text(ax, 10.2, 6.15, '机身俯仰 10°', 25)
    text(ax, 10.2, 5.35, '旧：被初始化吸收为地图倾斜', 17, BEFORE)
    text(ax, 10.2, 4.6, '新：机身保持10°，地图保持水平', 17, AFTER)
    text(ax, 10.2, 3.55, '静止窗口约1 s', 25, BLUE)
    text(ax, 10.2, 2.85, 'IMU / TF 新鲜、同步、稳定才接受', 16)
    text(ax, 1.05, 1.75, '重定位保留水平与高度基准；增加 reset 与无效输入拒绝', 22)
    text(ax, 1.05, 1.13, '2 m地面端点的旧假高差347.3 mm；修正后理想几何为0，尚无实机精度对照。', 15, MUTED)
    save(f, '06_gravity_alignment', '重力初始化修正', '避免将点击瞬间机身姿态固化为地图倾斜；静止与时间校验成为初始化条件。', ['gravity'])

    # 07: origin deviation across every comparable sampled frame, not only selected tail.
    origin = metrics['origin']
    f, ax = page(7, '高程修正：世界坐标原点 ≠ 当前雷达位置',
                 '注册点云重表达为当前传感器坐标；距离噪声、门限和清理射线采用真实TF原点', '旧包离线配对', '来源：elevation_sensor_origin_2026-11-04/bag_comparison*.json/csv；记录TF不是独立位置真值。')
    c = chart(f, [.09, .34, .52, .40])
    times = [float(r['receipt_time_s']) for r in rows]
    err = [float(r['sensor_origin_error_before_m']) for r in rows]
    c.plot(times, err, color=BEFORE, lw=3, label='旧：固定点云frame原点')
    c.plot(times, np.zeros(len(rows)), color=AFTER, lw=3, label='新：采用当前雷达TF原点')
    c.set(xlabel='录包相对时间 / s', ylabel='使用原点相对雷达TF的偏差 / m', ylim=(-.4, 15))
    c.legend(loc='upper left', fontsize=11, frameon=False); c.grid(color=LINE)
    text(ax, 11., 6.15, '5.207 → 0 m', 31, AFTER)
    text(ax, 11., 5.35, '位移子集：17帧偏差中位数', 16)
    text(ax, 11., 4.35, '最大位置变化', 19)
    text(ax, 11., 3.65, '0.0121 mm', 31, BLUE)
    text(ax, 11., 2.95, '重表达前后，点在map中的变化', 15, MUTED)
    text(ax, 1.05, 1.75, '651帧中抽48帧；46帧具备精确时刻TF，2帧边界跳过', 22)
    text(ax, 1.05, 1.12, '0表示采用记录雷达TF，不表示零定位/标定误差；新CPU转换约1.34 ms / 18000点（P50）。', 14.5, MUTED)
    save(f, '07_sensor_origin', '物理传感器原点修正', '原点跟随雷达运动，同时保持点在map中的几何位置。', ['origin'])

    # 08: model variance and GPU height error are kept in distinct panels.
    gpu = metrics['gpu']
    f, ax = page(8, '高程量化：模型方差与合成高度误差分开看',
                 '左：旧包同一批点的噪声模型｜右：合成平面实际GPU融合', '两类独立对照', '来源：bag_comparison.json / gpu_comparison.json；模型方差不是实测精度，合成结果不是实机结果。')
    s = origin['displaced_frame_stats']
    panels = [([.11, .33, .32, .39], [s['mean_measurement_variance_before_m2']['p50'], s['mean_measurement_variance_after_m2']['p50']],
               '测量方差 / m²', ['2.136', '0.300']),
              ([.60, .33, .30, .39], [gpu['plane']['height_mae_to_synthetic_truth_before_m']*1000, gpu['plane']['height_mae_to_synthetic_truth_after_m']*1000],
               '高度平均绝对误差 / mm', ['15.339', '0.078'])]
    for rect, values, ylabel, labels in panels:
        c = chart(f, rect)
        c.bar([0, 1], values, width=.5, color=[BEFORE, AFTER])
        c.set(xticks=[0, 1], xticklabels=['修正前', '修正后'], ylabel=ylabel, ylim=(0, max(values)*1.23))
        for x, value, lab in zip([0, 1], values, labels): c.text(x, value+max(values)*.05, lab, ha='center', fontsize=24)
    text(ax, 1.8, 6.95, '位移 > 1m 的17个记录帧', 19)
    text(ax, 9.55, 6.95, '88个格子 × 13次无噪声观测', 19)
    text(ax, 1.05, 1.7, 'GPU：新适配与独立原生传感器输入参考的高度、方差一致', 21, AFTER)
    text(ax, 1.05, 1.12, '5 cm近距离点、超过高度门限点：旧各错误建图1格 → 新各0格。', 16, MUTED)
    save(f, '08_elevation_results', '高程前后对照', '分别展示旧包模型变化和合成GPU收敛结果，避免把方差当高度真值误差。', ['origin', 'gpu'])

    # 09: do not present candidate height filters as implemented.
    cloud = metrics['cloud']
    f, ax = page(9, '点云分析：近场覆盖与安装视场比总点数更重要',
                 '本轮已审计数据分布；尚未新增导航ROI、自体过滤或补盲功能', '旧包48帧抽样', '来源：navigation_cloud_2026-11-04；7m方框是滚动ROI近似，高处点不自动等于噪声。')
    total = cloud['stats']['points']['p50']; inside = cloud['stats']['inside_7m_square']['p50']
    c = chart(f, [.10, .38, .35, .34])
    c.barh([1, 0], [total, inside], height=.45, color=[BLUE, AFTER])
    c.set(yticks=[1, 0], yticklabels=['整帧', '7×7 m近场'], xlabel='单帧点数中位数', xlim=(0, 21000))
    c.text(total+300, 1, f'{total:,.0f}', va='center', fontsize=19)
    c.text(inside+300, 0, f'{inside:,.1f}', va='center', fontsize=19)
    text(ax, 1.55, 2.55, '方框外占比中位数 65.55%', 21, BEFORE)
    c2 = chart(f, [.57, .33, .37, .40])
    c2.add_patch(Wedge((0, .5), 2.7, -40.976, 18.024, color=AFTER, alpha=.15))
    c2.add_patch(Wedge((0, .5), 2.7, 94.024, 153.024, color=BLUE, alpha=.15))
    for deg, col in [(-40.976, AFTER), (18.024, AFTER), (94.024, BLUE), (153.024, BLUE)]:
        r=np.deg2rad(deg); c2.plot([0, 2.7*np.cos(r)], [.5, .5+2.7*np.sin(r)], color=col)
    c2.axhline(0, color=INK, lw=1.7); c2.plot(0, .5, 'o', color=INK, ms=9)
    c2.set(xlim=(-2.9, 2.9), ylim=(-1.3, 3.5), xticks=[], yticks=[])
    c2.set_aspect('equal', adjustable='box')
    c2.text(-2.6, 3.1, '后方：26.98°～85.98°', fontsize=14, color=BLUE)
    c2.text(.1, -1., '前方：−40.98°～18.02°', fontsize=13.5, color=AFTER)
    c2.text(1.7, .1, '平地', fontsize=13)
    text(ax, 1.05, 1.65, '约34°俯装：前方更易看到地面，后方射线主要朝上', 23)
    text(ax, 1.05, 1.08, '视场图按静态TF与标称FOV计算；滤波无法补出未观测地面，未做物理安装标定。', 14.5, MUTED)
    save(f, '09_cloud_coverage', '点云密度与视场', '总点量约1.76万，但近场点约6119.5；不能仅减小LIO体素就宣称补盲。', ['cloud'])

    # 10: save the map size change without claiming measured speedup.
    f, ax = page(10, '配置与工具：缩小工作范围，统一实验入口',
                 '一并提交既有7 m地图配置；原raw启动入口加载新包内launch与传感器frame配置', '配置 + 工程整理', '来源：高程仓库3b1ebbf、主工程545feb0/daf64a0；地图格数为理论计算，非实测总耗时。')
    x, y, side, scale = 1.25, 2.4, 3.8, 3.8/20
    ax.add_patch(Rectangle((x, y), side, side, fill=False, ec=BEFORE, lw=2))
    for xx in np.linspace(x, x+side, 21): ax.plot([xx, xx], [y, y+side], color=LINE, lw=.7)
    for yy in np.linspace(y, y+side, 21): ax.plot([x, x+side], [yy, yy], color=LINE, lw=.7)
    small=7*scale; off=(side-small)/2
    ax.add_patch(Rectangle((x+off, y+off), small, small, facecolor=AFTER, alpha=.75))
    text(ax, x+side/2, 6.7, '20×20 m → 7×7 m', 25, ha='center')
    text(ax, x+side/2, 1.75, '有效格 40,000 → 4,900', 20, ha='center')
    text(ax, 8., 6.2, '−87.75% 格数', 34, AFTER)
    text(ax, 8., 5.45, '分辨率仍为0.1 m；覆盖范围同步缩小', 17)
    items = [('坐标统一', 'raw GridMap使用map，与local costmap一致'),
             ('回放工具', '2倍速循环，保留/cmd_vel显示'),
             ('操作入口', '一键启停、旧脚本/配置归档、参数说明')]
    for i, (label, detail) in enumerate(items):
        yy = 4.35-i*1.03
        text(ax, 8., yy, label, 20, BLUE)
        text(ax, 8., yy-.43, detail, 15.5)
    text(ax, 1.05, 1.03, '启停脚本在前阶段建立；本轮高程源码修正未修改stop/start一键脚本，未新增自动导航门控。', 14, MUTED)
    save(f, '10_configuration_tools', '配置与工具整理', '地图大小、frame、回放与入口统一管理；格数变化不能当成整机性能提升。', [])

    # 11: independent test scopes, never summed into a misleading total.
    f, ax = page(11, '验证覆盖：功能、语义、传输分别确认',
                 '以下是各阶段独立验证结果，不相加为同一次完整测试', '验证台账', '来源：HeightJump实施报告、定位修正报告、elevation_sensor_origin/comparison.md。')
    tests = [(1.1, 6.45, '30', 'HeightJump GTest', '插件加载 / TF / 地图 / 失效保护'),
             (8.6, 6.45, '242,304', '差分评估组合', '24张地图，decision/unknown/jump一致'),
             (1.1, 3.95, '52', '主工程 Python测试', '含28项定位专项；最新一次全通过'),
             (8.6, 3.95, '23', '高程专项测试', '回调 / launch / GPU / raw / 默认frame')]
    for x, y, value, label, detail in tests:
        text(ax, x, y, value, 40, AFTER)
        text(ax, x+2.8 if len(value)<4 else x+4.8, y, label, 20)
        text(ax, x, y-.8, detail, 15.5, MUTED)
        rule(ax, x, y-1.27, 6.2)
    text(ax, 1.1, 1.65, '实际ROS链路：重力对齐 → TF；注册点云 → GPU → GridMap', 22, BLUE)
    text(ax, 1.1, 1.08, '完整高程测试仍有6/15组失败：PyTorch缺libcudss.so.0；raw路径专项通过。', 15, BEFORE)
    save(f, '11_validation', '验证覆盖与已知阻断', '回归与独立ROS链路已有证据；完整高程测试尚未全绿。', ['heightjump'])

    # 12: clear handover and actual remaining work.
    f, ax = page(12, '当前结论与下一轮实机验证',
                 '代码已落地，性能与几何有离线证据；现场收益等待同路线重新录包', '交付与下一步', '提交：主工程545feb0/daf64a0/3c59a62/7609f5c；高程4190bb9/3b1ebbf。未移植Nav2 PR #4621。')
    text(ax, 1.05, 6.8, '已完成', 26, AFTER)
    text(ax, 8.55, 6.8, '待实机复测 / 尚未修复', 26, BEFORE)
    done = ['显示降载 + HeightJump查询优化', '重力初始化 + 重定位基准保护', '物理传感器原点 + 有界TF检查', '配置归档 + 可复现数据与本地提交']
    todo = ['真实MPPI频率、最大间隔与转圈', '原始IMU时间推进异常的根因', '地图覆盖、薄小障碍与动态清理', '实际底盘响应与TF/观测年龄']
    for i, (left, right) in enumerate(zip(done, todo)):
        yy=5.8-i*.9
        ax.add_patch(Circle((1.2, yy), .075, color=AFTER)); text(ax, 1.5, yy, left, 19)
        ax.add_patch(Circle((8.7, yy), .075, color=BEFORE)); text(ax, 9., yy, right, 19)
    rule(ax, 1.05, 2.25, 13.8)
    text(ax, 1.05, 1.7, '录制：模式切换 + 原始IMU/LiDAR + TF + 高程 + 原始/最终cmd_vel', 21)
    text(ax, 1.05, 1.08, '不能据当前数据宣称“转圈已解决”或“实机稳定20 Hz”；FAST-LIO、驱动与MPPI采样规模未改。', 14.5, MUTED)
    save(f, '12_handover', '交付与实机复测', '先按正确交接流程重测，再判断真实控制频率、地形覆盖和执行效果。', [])

# Lightweight contact sheet for manual visual QA.
thumbs = []
for p in pages:
    with Image.open(IMG/(p['name']+'.png')) as im:
        im.load()
        assert im.size == (2560, 1440)
        thumbs.append(im.resize((640, 360)).convert('RGB'))
sheet = Image.new('RGB', (1920, 1440), '#E4EAF0')
for i, thumb in enumerate(thumbs): sheet.paste(thumb, ((i % 3)*640, (i // 3)*360))
sheet.save(OUT/'全部图表预览.jpg', quality=94)

md = ['# 2026年11月3—4日导航优化与修改汇报', '',
      '> 12张16:9高清图，每张可直接作为一页PPT。PNG为2560×1440，SVG可无损放大。', '',
      '[下载整份PDF](导航优化汇报_12页.pdf) · [查看全部图表预览](全部图表预览.jpg)', '',
      '**口径：** 离线性能、旧包分析、合成验证分别标注；没有修改后的实机导航对照包。', '']
for i, p in enumerate(pages, 1):
    md += [f'## {i:02d}｜{p["title"]}', '', f'![{p["title"]}](images/{p["name"]}.png)', '',
           p['takeaway'], '', f'[PNG](images/{p["name"]}.png) · [SVG](images/{p["name"]}.svg)', '']
md += ['## 数据来源', '',
       '所有数字来自已保存的报告、日志汇总和逐帧数据；副本见[source_data](source_data/)。', '',
       '| 内容 | 原始记录位置（相对项目根目录） |', '|---|---|']
for name, path in SOURCES.items(): md.append(f'| {name} | `docs/analysis/{path}` |')
md += ['', '**版本：** 主工程 `545feb0`、`daf64a0`、`3c59a62`、`7609f5c`；高程仓库 `4190bb9`、`3b1ebbf`。', '',
       '**未完成：** 实机稳定20Hz、转圈闭环验证、原始IMU时间异常根因、点云补盲/新增导航过滤；未移植Nav2 PR #4621。', '',
       '**重新生成：** 在项目环境运行 `python3 docs/reports/2026-11-03_04_navigation_optimization/render_report.py`。', '']
(OUT/'优化与修改汇报.md').write_text('\n'.join(md))
manifest = {'period': ['2026-11-03', '2026-11-04'], 'pages': pages,
            'image_size': [2560, 1440], 'aspect_ratio': '16:9',
            'field_navigation_improvement_verified': False,
            'source_sha256': {path: hashlib.sha256((A/path).read_bytes()).hexdigest() for path in SOURCES.values()},
            'origin_csv_sha256': hashlib.sha256(csv_path.read_bytes()).hexdigest()}
(OUT/'data_manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')
bundle = OUT/'导航优化汇报_图片与MD.zip'
with zipfile.ZipFile(bundle, 'w', compression=zipfile.ZIP_DEFLATED) as archive:
    for path in sorted(OUT.rglob('*')):
        if path.is_file() and path != bundle and '__pycache__' not in path.parts:
            archive.write(path, path.relative_to(OUT))
print(json.dumps({'report': str(OUT/'优化与修改汇报.md'), 'slides': len(pages),
                  'pngs': len(list(IMG.glob('*.png'))), 'svgs': len(list(IMG.glob('*.svg'))),
                  'bundle_bytes': bundle.stat().st_size}, ensure_ascii=False, indent=2))
