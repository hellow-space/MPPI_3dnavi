#!/usr/bin/env python3
"""Plain plots and numeric tables from saved measurements; no ROS operations."""
import csv
import json
from pathlib import Path
import shutil
import zipfile

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
ANALYSIS = ROOT/'docs/analysis'
IMAGES = OUT/'images'
IMAGES.mkdir(exist_ok=True)
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
font = font_manager.FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc').get_name()
plt.rcParams.update({'font.family': font, 'font.size': 11, 'axes.unicode_minus': False,
                     'figure.facecolor': 'white', 'axes.facecolor': 'white', 'svg.fonttype': 'path'})


def read(path):
    return json.loads((ANALYSIS/path).read_text())


def finish(fig, name, note):
    fig.text(.01, .012, note, fontsize=9, color='#444444')
    fig.tight_layout(rect=(0, .045, 1, .97))
    fig.savefig(IMAGES/f'{name}.png', dpi=180)
    fig.savefig(IMAGES/f'{name}.svg')
    p=IMAGES/f'{name}.svg'
    p.write_text('\n'.join(line.rstrip() for line in p.read_text().splitlines())+'\n')
    plt.close(fig)


def grouped(ax, labels, before, after, title, ylabel):
    x=np.arange(len(labels));width=.34
    ax.bar(x-width/2,before,width,label='修改前',color='tab:blue')
    ax.bar(x+width/2,after,width,label='修改后',color='tab:orange')
    for xx, a, b in zip(x,before,after):
        for pos,val in [(xx-width/2,a),(xx+width/2,b)]:
            ax.annotate(f'{val:.2f}',(pos,val),xytext=(0,4),textcoords='offset points',ha='center',fontsize=10)
    ax.set(xticks=x,xticklabels=labels,title=title,ylabel=ylabel)
    ax.set_ylim(0,max(before+after)*1.22)
    ax.grid(axis='y',alpha=.25);ax.set_axisbelow(True);ax.legend(fontsize=9)


v=read('mppi_frequency_2026-11-03/metrics.json')
h=read('heightjump_implementation_2026-11-03/metrics.json')
full=h['full_mppi']
o=read('elevation_sensor_origin_2026-11-04/bag_comparison.json')
gpu=read('elevation_sensor_origin_2026-11-04/gpu_comparison.json')

fig,axes=plt.subplots(1,2,figsize=(13,4.8))
keys=['optimizer','visualizer_publish','compute_plus_visualization']
labels=['优化器','显示发布阶段','优化器 + 显示']
for ax,q in zip(axes,['p50_ms','p95_ms']):
    before=[v['current_visualization']['stages'][k][q] for k in keys]
    after=[v['sparse_visualization']['stages'][k][q] for k in keys]
    grouped(ax,labels,before,after,f'轨迹显示降载：{q[:3].upper()}','耗时 / ms')
    ax.axhline(50,color='red',ls='--',lw=1,label='20 Hz预算')
    ax.legend(fontsize=9)
finish(fig,'01_visualization','离线60次测量，含DDS订阅者；发布阶段含库内处理。显示stride 5/3→20/5；优化采样1000×56不变。')

fig,axes=plt.subplots(1,2,figsize=(13,4.8))
keys=['0','1','3','4'];labels=['平地','台阶','unknown拒绝','unknown允许']
for ax,q in zip(axes,['p50_ms','p95_ms']):
    before=[h['paired'][k]['prepared_rollout'][q] for k in keys]
    after=[h['paired'][k]['batched_rollout'][q] for k in keys]
    grouped(ax,labels,before,after,f'HeightJump评分：{q[:3].upper()}','耗时 / ms')
finish(fig,'02_heightjump','同进程配对基准；100轮、每轮1000×56预测点。原prepared标量查询与批量查询比较；不含完整控制器。')

fig,axes=plt.subplots(1,3,figsize=(16,4.8))
for ax,key,title in [(axes[0],'optimizer','持续吞吐：优化器'),
                     (axes[1],'compute_plus_visualization','持续吞吐：优化器 + 显示')]:
    grouped(ax,['P50','P95'],[full['before'][key][q] for q in ['p50_ms','p95_ms']],
            [full['after'][key][q] for q in ['p50_ms','p95_ms']],title,'耗时 / ms')
grouped(axes[2],['P50','P95','最大值'],
        [full['periodic_before']['completion_interval'][q] for q in ['p50_ms','p95_ms','max_ms']],
        [full['periodic_after']['completion_interval'][q] for q in ['p50_ms','p95_ms','max_ms']],
        '50 ms周期组：完成间隔','间隔 / ms')
axes[2].axhline(50,color='red',ls='--',lw=1)
finish(fig,'03_full_mppi','插件更新前后保持相同稀疏显示与1000×56采样；离线60轮。周期完成间隔抖动没有改善，不能称实机稳定20 Hz。')

rows=list(csv.DictReader((ANALYSIS/'elevation_sensor_origin_2026-11-04/bag_comparison_frames.csv').open()))
t=np.array([float(r['receipt_time_s']) for r in rows])
fig,axes=plt.subplots(3,1,figsize=(13,9),sharex=True)
panels=[('sensor_origin_error_before_m','sensor_origin_error_after_m','融合原点相对记录雷达TF的偏差','偏差 / m'),
        ('mean_measurement_variance_before_m2','mean_measurement_variance_after_m2','共同通过门限点的逐帧平均噪声模型方差','模型方差 / m²'),
        ('gate_passed_before_in_roi','gate_passed_after_in_roi','7×7 m近场内通过距离/高度门限的点数','点数')]
for ax,(before,after,title,ylabel) in zip(axes,panels):
    ax.plot(t,[float(r[before]) for r in rows],'.-',label='修改前',lw=1.3,ms=4)
    ax.plot(t,[float(r[after]) for r in rows],'.-',label='修改后',lw=1.3,ms=4)
    ax.set(title=title,ylabel=ylabel);ax.grid(alpha=.3);ax.legend(loc='upper left',fontsize=9)
axes[-1].set_xlabel('相对首帧点云的录包时间 / s')
finish(fig,'04_sensor_origin','旧包2026-11-02_20-19-46：抽48帧，46帧有精确TF；全部比较帧均展示。模型方差不是实测高度误差，零偏差表示使用记录TF。')

g=gpu['plane'];fig,axes=plt.subplots(1,3,figsize=(14,4.8))
for ax,before,after,title,unit in [
    (axes[0],g['height_mae_to_synthetic_truth_before_m']*1000,g['height_mae_to_synthetic_truth_after_m']*1000,'平面高度平均绝对误差','mm'),
    (axes[1],g['mean_variance_before_m2'],g['mean_variance_after_m2'],'平均融合方差','m²'),
    (axes[2],g['max_height_difference_to_native_reference_before_m']*1000,g['max_height_difference_to_native_reference_after_m']*1000,'相对原生传感器参考的最大高度差','mm')]:
    ax.bar(['修改前','修改后'],[before,after],color=['tab:blue','tab:orange'],width=.5)
    ax.set(title=title,ylabel=unit,ylim=(0,before*1.25))
    ax.grid(axis='y',alpha=.25);ax.set_axisbelow(True)
    for i,val in enumerate([before,after]):ax.annotate(f'{val:.5g}',(i,val),xytext=(0,5),textcoords='offset points',ha='center')
finish(fig,'05_gpu_plane','合成实际GPU实验：88格、13次无噪声观测、4×4 m地图；关闭visibility/overlap清理并放宽离群门限，不代表实机精度。')

existing=[
    ('06_bag_timing.png','indoor_spin_2026-11-02_20-19-46/timing_and_motion.png'),
    ('07_command_streams.png','indoor_spin_2026-11-02_20-19-46/command_streams.png'),
    ('08_cloud_density.png','navigation_cloud_2026-11-04/cloud_density.png'),
    ('09_trajectory.png','indoor_spin_2026-11-02_20-19-46/trajectory.png')]
for name,path in existing:shutil.copyfile(ANALYSIS/path,IMAGES/name)

titles=['轨迹显示降载：P50/P95耗时','HeightJump四种场景：P50/P95耗时',
        '完整MPPI耗时与周期完成间隔','高程原点、模型方差和过滤点数：逐帧曲线',
        '合成GPU平面：高度误差与融合方差','旧包：指令、反馈、控制间隔与输入时间',
        '旧包：原始控制指令与最终cmd_vel','旧包：近场点云投影与距离分布','旧包：定位轨迹与恢复阶段']
files=[f'{i:02d}_{name}.png' for i,name in enumerate(['visualization','heightjump','full_mppi','sensor_origin','gpu_plane','bag_timing','command_streams','cloud_density','trajectory'],1)]
md=['# 两天导航修改：数据图与数值表','',
    '数据范围：2026年11月3—4日开展的分析与修改。图6—9直接使用原报告图片。', '',
    '## 图片目录','', '| 编号 | 数据图 | 数据来源 |','|---|---|---|']
for i,(title,file) in enumerate(zip(titles,files),1):
    scope='离线性能测量' if i<=3 else ('旧包逐帧离线配对' if i==4 else ('合成GPU实验' if i==5 else '旧包原始分析图'))
    md.append(f'| {i} | [{title}](images/{file}) | {scope} |')
md+=['','## 修改前后数值表','',
     '| 项目 | 修改前P50 / ms | 修改后P50 / ms | 修改前P95 / ms | 修改后P95 / ms |',
     '|---|---:|---:|---:|---:|']
measurements=[('轨迹显示降载：优化器+显示',v['current_visualization']['stages']['compute_plus_visualization'],v['sparse_visualization']['stages']['compute_plus_visualization']),
              ('插件更新：优化器',full['before']['optimizer'],full['after']['optimizer']),
              ('插件更新：优化器+显示',full['before']['compute_plus_visualization'],full['after']['compute_plus_visualization']),
              ('周期组：完成间隔',full['periodic_before']['completion_interval'],full['periodic_after']['completion_interval'])]
for name,before,after in measurements:
    md.append(f'| {name} | {before["p50_ms"]:.3f} | {after["p50_ms"]:.3f} | {before["p95_ms"]:.3f} | {after["p95_ms"]:.3f} |')
md+=['','各行为不同实验，不能叠加成整机提速百分比；完成间隔P95未改善。','',
     '| HeightJump场景 | 原标量P50 / ms | 批量P50 / ms | 原标量P95 / ms | 批量P95 / ms |','|---|---:|---:|---:|---:|']
for key,label in zip(['0','1','3','4'],['平地','台阶','unknown拒绝','unknown允许']):
    a,b=h['paired'][key]['prepared_rollout'],h['paired'][key]['batched_rollout']
    md.append(f'| {label} | {a["p50_ms"]:.3f} | {b["p50_ms"]:.3f} | {a["p95_ms"]:.3f} | {b["p95_ms"]:.3f} |')
md+=['','| 高程对照指标 | 修改前 | 修改后 | 数据类型 |','|---|---:|---:|---|',
     '| 融合原点与雷达TF偏差，中位数 | 5.207 m | 0（采用TF） | 旧包中偏差>1m的17帧 |',
     '| 同一批点的平均测量模型方差，中位数 | 2.136454 m² | 0.300119 m² | 同上；不是实测高度误差 |',
     '| 平面高度MAE | 15.339 mm | 0.0779 mm | 合成GPU，88格×13帧 |',
     '| 平均融合方差 | 0.306783 m² | 0.00155745 m² | 同上 |','']
for i,(title,file) in enumerate(zip(titles,files),1):
    md += [f'## 图{i}：{title}','',f'![{title}](images/{file})','']
    if i==4:md+=['旧包只计算原点/门限/噪声公式，未重放完整GPU地图时序；原点采用TF不等于零定位误差。','']
    if i==5:md+=['这是合成收敛实验，不是现场精度。','']
    if i==6:md+=[
        '这是优化前旧包，不是修改后的效果图；横轴为从录包开始计的秒数。', '',
        '| 从上到下 | 曲线及单位 | 含义 |', '|---|---|---|',
        '| 1：前进速度 | 蓝：指令vx；橙：定位推算机身前向速度，m/s | 定位速度由位置变化平滑求导得到，不是独立底盘测速。 |',
        '| 2：转向角速度 | 蓝：指令wz；橙：定位航向推算角速度，rad/s | 正负代表相反转向，约40.98—50.99s的指令来自旋转恢复。 |',
        '| 3：位移与朝向 | 蓝：map的X位移；橙：map的Y位移，左轴m；淡红：Yaw相对初始变化，右轴deg | 位移相对录包首帧；Yaw不是角速度，也不是俯仰/横滚。 |',
        '| 4：优化输出间隔 | 蓝点：相邻成功优化显示输出间隔，ms；红虚线：50ms | 目标20Hz，活跃实际约10.86Hz；恢复期没有输出不等于MPPI崩溃。 |',
        '| 5：输入时间差 | 蓝：IMU回调日志时间减原始header；橙：LiDAR对应时间差，s | IMU约27—54s且持续扩大；不能直接等同于定位或底盘执行延迟。 |', '',
        '背景：橙色区间40.978—50.986s为spin恢复，灰色50.999—56.061s为wait阶段；紫色竖虚线为3次无进展失败。录包在旧任务中途开始。', '',
        '前段手动等待期间机器人静止，不能据此判为定位延迟；手动/导航切换时刻未录制。第三个子图的Yaw图例已补齐。','']
md+=['## 来源与范围','',
     '- 显示降载：`docs/analysis/mppi_frequency_2026-11-03/metrics.json`。',
     '- HeightJump及完整MPPI：`docs/analysis/heightjump_implementation_2026-11-03/metrics.json`。',
     '- 高程配对：`docs/analysis/elevation_sensor_origin_2026-11-04/`的JSON和逐帧CSV。',
     '- 旧包：`nav_elevation_2026-11-02_20-19-46`；点云投影来自48帧抽样。',
     '- 暂无修改后的实机包，不能报告实机更新频率、转圈或覆盖率的改善百分比。','']
(OUT/'数据图与数值表.md').write_text('\n'.join(md))
with zipfile.ZipFile(OUT/'数据图与数值表.zip','w',compression=zipfile.ZIP_DEFLATED) as archive:
    for p in sorted(OUT.rglob('*')):
        if p.is_file() and p.suffix in ['.md','.png','.svg','.py']:
            archive.write(p,p.relative_to(OUT))
print(json.dumps({'report':str(OUT/'数据图与数值表.md'),'plots':len(files)},ensure_ascii=False))
