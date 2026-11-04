#!/usr/bin/env python3
"""Offline analysis of this bag and its matching logs. Never starts ROS nodes."""
import csv
import json
import pickle
import re
import sqlite3
from collections import Counter
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.spatial.transform import Rotation

OUT = Path(__file__).resolve().parent
D = pickle.load(open('/tmp/mppi_indoor_spin_201946_data.pkl', 'rb'))
duration = (D['t1'] - D['t0']) / 1e9
t0 = D['t0'] / 1e9
NAV = Path('/home/nvidia/.ros/log/component_container_isolated_14850_1793621885194.log')
LIO = Path('/home/nvidia/.ros/log/fastlivo_mapping_14348_1793621870425.log')


def stats(values):
    a = np.asarray(values, dtype=float)
    a = a[np.isfinite(a)]
    return dict(zip(['min', 'p50', 'p95', 'p99', 'max'],
                    map(float, np.percentile(a, [0, 50, 95, 99, 100])))) if len(a) else {}


def save_csv(name, header, rows):
    with (OUT / name).open('w') as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def zoh(times, a, col):
    indices = np.searchsorted(a[:, 0], times, side='right') - 1
    return a[np.clip(indices, 0, len(a) - 1), col]


# Compose full 3D TF. map->camera_init is constant in this recording.
A = D['tf'][('camera_init', 'aft_mapped')]
B = D['tf'][('map', 'camera_init')]
C = D['static'][('aft_mapped', 'base_link')]
assert np.all(B[:, 2:] == B[0, 2:]), 'Use time-dependent TF composition if map alignment changes'
ra, rb, rc = Rotation.from_quat(A[:, 6:10]), Rotation.from_quat(B[0, 6:10]), Rotation.from_quat(C[4:8])
xyz = rb.apply(ra.apply(C[:3]) + A[:, 2:5]) + B[0, 2:5]
rot = rb * ra * rc
yaw = np.unwrap(rot.as_euler('ZYX')[:, 0])
pose = np.column_stack([A[:, 0], A[:, 1], xyz, yaw])
raw, final = D['cmd']['/command/cmd_twist'], D['cmd']['/cmd_vel']

nav_counts = Counter()
events = []
misses = []
for line_number, line in enumerate(NAV.open(), 1):
    match = re.search(r'\[(179\d+\.\d+)\]', line)
    if not match:
        continue
    t = float(match[1]) - t0
    if not 0 <= t <= duration:
        continue
    for key in ['Control loop missed', 'Failed to make progress', 'Optimizer fail',
                'HeightJump unavailable', 'HeightJump TF', 'Aborting handle',
                'Running spin', 'Running wait', 'Goal failed', '[ERROR]']:
        if key in line:
            nav_counts[key] += 1
    if 'Control loop missed' in line:
        misses.append(t)
    if '[ERROR]' in line or any(key in line for key in [
            'Aborting handle', 'Running spin', 'Running wait', 'wait completed',
            'Turning ', 'Exceeded time allowance', 'HeightJump unavailable', 'HeightJump TF']):
        events.append(dict(line=line_number, t=t, text=line.strip()))

imu, lidar, mismatch, hard = [], [], [], []
for line in LIO.open():
    match = re.search(r'\[(179\d+\.\d+)\]', line)
    if not match:
        continue
    wall = float(match[1])
    t = wall - t0
    if not 0 <= t <= duration:
        continue
    for key, dest in [('get imu at time:', imu), ('Get LiDAR, its header time:', lidar)]:
        if key in line:
            st = float(line.split(key)[1].split()[0])
            dest.append([t, st - t0, wall - st])
    for key, dest in [('delta time:', mismatch), ('lag is', hard)]:
        if key in line:
            dest.append([t, float(line.split(key)[1].split()[0])])
imu, lidar, mismatch, hard = map(np.asarray, [imu, lidar, mismatch, hard])

# Use successful optimizer/visualizer output to measure active MPPI intervals.
# /command/cmd_twist is shared with recovery behaviors and contains a deliberate wait gap.
trajectory_times = np.asarray(D['times']['/trajectories'])
trajectory_gaps = np.diff(trajectory_times)
active_gaps = trajectory_gaps[trajectory_gaps < .5]
windows = [(0, 20.888, 'tracking_1'), (20.888, 40.954, 'tracking_2'),
           (40.978, 50.986, 'spin_recovery'), (50.999, 56.061, 'wait_recovery'),
           (56.061, duration, 'tracking_3')]
segments = []
for start, end, name in windows:
    selection = (A[:, 0] >= start) & (A[:, 0] < end)
    x, y = xyz[selection], yaw[selection]
    r = raw[(raw[:, 0] >= start) & (raw[:, 0] < end)]
    ts = trajectory_times[(trajectory_times >= start) & (trajectory_times < end)]
    segments.append(dict(
        name=name, start_s=start, end_s=end, pose_samples=len(x),
        xy_span_m=np.ptp(x[:, :2], axis=0).tolist(),
        xy_displacement_m=float(np.linalg.norm(x[-1, :2] - x[0, :2])),
        xy_path_length_m=float(np.linalg.norm(np.diff(x[:, :2], axis=0), axis=1).sum()),
        yaw_delta_deg=float(np.rad2deg(y[-1] - y[0])),
        yaw_span_deg=float(np.rad2deg(np.ptp(y))),
        yaw_abs_travel_deg=float(np.rad2deg(np.abs(np.diff(y)).sum())),
        command_count=len(r),
        command_mean_vx_wz=r[:, [1, 3]].mean(axis=0).tolist() if len(r) else None,
        trajectory_count=len(ts),
        trajectory_rate_hz=float((len(ts) - 1) / (ts[-1] - ts[0])) if len(ts) > 1 else None,
        trajectory_intervals_s=stats(np.diff(ts)),
    ))

# Match exact command values with final cmd_vel by nearest recorder reception time.
# This is evidence of forwarding, not a hardware execution latency measurement.
matches, used = [], set()
for row in raw:
    indices = np.flatnonzero((np.max(np.abs(final[:, 1:] - row[1:]), axis=1) < 1e-9) &
                             (final[:, 0] >= row[0] - .01) & (final[:, 0] < row[0] + .03))
    if len(indices):
        index = int(indices[np.argmin(np.abs(final[indices, 0] - row[0]))])
        used.add(index)
        matches.append(final[index, 0] - row[0])
other = final[[i not in used for i in range(len(final))]]
latest = np.searchsorted(raw[:, 0], other[:, 0], side='right') - 1
valid = latest >= 0
delta = other[valid, 3] - raw[latest[valid], 3]

grid = np.arange(A[0, 0], A[-1, 0], .02)
interpolated_yaw = np.interp(grid, A[:, 0], yaw)
observed_wz = gaussian_filter1d(np.gradient(interpolated_yaw, .02), 5)
vx = gaussian_filter1d(np.gradient(np.interp(grid, A[:, 0], xyz[:, 0]), .02), 5)
vy = gaussian_filter1d(np.gradient(np.interp(grid, A[:, 0], xyz[:, 1]), .02), 5)
observed_vx = vx * np.cos(interpolated_yaw) + vy * np.sin(interpolated_yaw)
lag_results, lag_rows = [], []
for start, end in [(0, 40), (56.5, 92), (60, 75), (75, 92)]:
    curve = []
    for lag in np.arange(0, 10.001, .02):
        selection = (grid >= start + lag) & (grid < end)
        u, v = zoh(grid[selection] - lag, raw, 3), observed_wz[selection]
        corr = float(np.corrcoef(u, v)[0, 1])
        curve.append((float(lag), corr))
        lag_rows.append([start, end, lag, corr])
    best = max(curve, key=lambda row: row[1])
    lag_results.append(dict(start=start, end=end, best_lag_s=best[0],
                            best_correlation=best[1], zero_correlation=curve[0][1]))

path_errors = []
for row in pose:
    errors = []
    for name in ['/plan', '/transformed_global_plan']:
        paths = D['paths'][name]
        index = np.searchsorted([p['t'] for p in paths], row[0], side='right') - 1
        errors.append(float('nan') if index < 0 else float(np.sqrt(np.min(
            np.sum((paths[index]['pts'][:, :2] - row[2:4]) ** 2, axis=1)))))
    path_errors.append(errors)
path_errors = np.asarray(path_errors)

# What terrain data was available to the recorder at each successful MPPI output.
terrain_stamps = np.asarray([[m['t'], m['stamp']] for m in D['terrain']])
indices = np.searchsorted(terrain_stamps[:, 0], trajectory_times, side='right') - 1
selection = indices >= 0
terrain_age = trajectory_times[selection] - terrain_stamps[indices[selection], 1]
con = sqlite3.connect(f"file:{next(Path(D['bag']).glob('*.db3'))}?mode=ro", uri=True)
sizes = {name: dict(count=count, bytes=total, average_bytes=total/count)
         for name, count, total in con.execute('''
             SELECT topics.name, count(*), sum(length(messages.data)) FROM messages
             JOIN topics ON topics.id=messages.topic_id GROUP BY topics.id''')}

metrics = dict(
    operator_context='User sends navigation goal in manual mode, waits tens of seconds, then switches to navigation; mode switch timestamp was not recorded. Static pose before handover is not evidence of stale localization.',
    bag=D['bag'], duration_s=duration, bag_start_ns=D['t0'],
    active_mppi_interval_s=stats(active_gaps), active_mppi_rate_hz=float(1/np.mean(active_gaps)),
    trajectory_gaps_over_half_second_s=trajectory_gaps[trajectory_gaps >= .5].tolist(),
    nav_log=str(NAV), lidar_log=str(LIO), nav_log_counts=dict(nav_counts), events=events,
    segments=segments, command_forward_matches=len(matches), command_total=len(raw),
    recorder_forward_difference_s=stats(matches),
    remaining_final_stream_count=len(other),
    remaining_final_stream_rate_hz=float((len(other)-1)/(other[-1, 0]-other[0, 0])),
    other_final_wz_difference_abs=stats(np.abs(delta)),
    other_final_opposite_wz_count=int(np.sum((other[valid, 3]*raw[latest[valid], 3] < 0) &
                                          (np.abs(other[valid, 3]) > .1) &
                                          (np.abs(raw[latest[valid], 3]) > .1))),
    imu_raw_callback_age_s=stats(imu[:, 2]), lidar_callback_age_s=stats(lidar[:, 2]),
    imu_callback_count=len(imu), imu_callback_rate_hz=float((len(imu)-1)/(imu[-1,0]-imu[0,0])),
    imu_header_time_advance_s=float(imu[-1,1]-imu[0,1]),
    imu_callback_wall_advance_s=float(imu[-1,0]-imu[0,0]),
    imu_header_seconds_per_wall_second=float(np.polyfit(imu[:, 0], imu[:, 1], 1)[0]),
    imu_source_timestamp_intervals_s=stats(np.diff(imu[:,1])),
    imu_lidar_mismatch_warning_count=len(mismatch),
    imu_lidar_mismatch_s=stats(mismatch[:,1]), hard_sync_log_s=stats(hard[:,1]),
    odom_twist_all_zero=bool(np.all(D['odom'][:,6:] == 0)),
    odom_receipt_minus_header_s=stats(D['ages']['/aft_mapped_to_init'][:,1]),
    recorder_available_terrain_age_at_mppi_s=stats(terrain_age),
    pose_xy_path_length_m=float(np.linalg.norm(np.diff(xyz[:,:2],axis=0),axis=1).sum()),
    pose_xy_displacement_m=float(np.linalg.norm(xyz[-1,:2]-xyz[0,:2])),
    pose_total_abs_yaw_deg=float(np.rad2deg(np.abs(np.diff(yaw)).sum())),
    pose_start_end_xyz=xyz[[0,-1]].tolist(), path_error_global_m=stats(path_errors[:,0]),
    path_error_local_m=stats(path_errors[:,1]), response_lag_exploratory=lag_results,
    response_lag_caveat='Changing windows changes best lag; this is command-to-localization correlation, not hardware delay.',
    topic_serialized_sizes=sizes,
)
(OUT/'metrics.json').write_text(json.dumps(metrics, indent=2))
save_csv('pose.csv', ['receipt_s','header_s','map_x','map_y','map_z','yaw_rad','global_path_error_m','local_path_error_m'], np.c_[pose,path_errors])
save_csv('phase_metrics.csv', list(segments[0]), [[s[k] for k in segments[0]] for s in segments])
save_csv('response_lag.csv', ['window_start_s','window_end_s','lag_s','correlation'], lag_rows)
for name, a in [('imu',imu),('lidar',lidar)]:
    save_csv(name+'_log_timing.csv', ['callback_s','header_relative_s','callback_minus_header_s'], a)

plt.rcParams.update({'font.size': 10})
fig, axes = plt.subplots(5, 1, figsize=(13, 14), sharex=True, constrained_layout=True)
axes[0].step(raw[:,0],raw[:,1],where='post',label='Recorded command vx',lw=1)
axes[0].plot(grid,observed_vx,label='Localization-derived body vx',lw=1)
axes[0].set_ylabel('m/s')
axes[1].step(raw[:,0],raw[:,3],where='post',label='Recorded command wz',lw=1)
axes[1].plot(grid,observed_wz,label='Localization-derived wz',lw=1)
axes[1].set_ylabel('rad/s')
axes[2].plot(A[:,0],xyz[:,0]-xyz[0,0],label='Map x displacement')
axes[2].plot(A[:,0],xyz[:,1]-xyz[0,1],label='Map y displacement')
axes[2].set_ylabel('m')
ax_yaw=axes[2].twinx()
ax_yaw.plot(A[:,0],np.rad2deg(yaw-yaw[0]),color='tab:red',alpha=.5,label='Yaw change')
ax_yaw.set_ylabel('yaw change (deg)')
axes[3].plot(trajectory_times[1:],np.where(trajectory_gaps<.5,trajectory_gaps,np.nan)*1000,'.',ms=2,label='MPPI visualization output interval')
axes[3].axhline(50,color='red',ls='--',label='20 Hz target: 50 ms')
axes[3].set_ylabel('ms')
axes[4].plot(imu[:,0],imu[:,2],label='IMU callback minus raw timestamp')
axes[4].plot(lidar[:,0],lidar[:,2],label='LiDAR callback minus timestamp')
axes[4].set_ylabel('s')
axes[4].set_xlabel('Seconds since first bag message')
for ax in axes:
    ax.axvspan(40.978,50.986,color='orange',alpha=.15)
    ax.axvspan(50.999,56.061,color='gray',alpha=.15)
    for t in [.351,20.888,40.954]:ax.axvline(t,color='purple',lw=.7,ls=':')
    ax.grid(alpha=.2)
    ax.legend(loc='upper left',fontsize=8)
    ax.set_xlim(0,duration)
fig.suptitle('Old bag 2026-11-02 20:19:46: navigation output before manual handover may be blocked\nOrange: spin recovery; gray: wait recovery; purple: progress failures; mode switch time unknown')
fig.savefig(OUT/'timing_and_motion.png',dpi=160)
plt.close(fig)

fig, ax = plt.subplots(figsize=(10,10),constrained_layout=True)
for p in D['paths']['/plan'][::5]:ax.plot(p['pts'][:,0],p['pts'][:,1],color='gray',lw=.7,alpha=.5)
sc=ax.scatter(xyz[:,0],xyz[:,1],c=A[:,0],cmap='viridis',s=12)
fig.colorbar(sc,ax=ax,label='Bag time (s)')
for t in [0,20,40,50,56,60,65,70,75,80,85,90]:
    i=np.argmin(abs(A[:,0]-t))
    ax.annotate(f'{t}s',xyz[i,:2],xytext=(4,4),textcoords='offset points',fontsize=8)
    ax.arrow(xyz[i,0],xyz[i,1],.25*np.cos(yaw[i]),.25*np.sin(yaw[i]),head_width=.06,color='tab:red',alpha=.6)
goal=D['paths']['/plan'][-1]['pts'][-1,:2]
ax.plot(*goal,'r*',ms=14,label='Goal')
ax.plot(*xyz[0,:2],'go',label='Bag start')
ax.plot(*xyz[-1,:2],'rx',label='Bag end')
ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend()
ax.set_xlabel('map x (m)');ax.set_ylabel('map y (m)')
ax.set_title('Full 3D TF reconstruction; recorded localization, not external ground truth')
fig.savefig(OUT/'trajectory.png',dpi=160);plt.close(fig)

fig,axes=plt.subplots(2,1,figsize=(12,6),sharex=True,constrained_layout=True)
for ax,col,label in zip(axes,[1,3],['vx (m/s)','wz (rad/s)']):
    ax.step(raw[:,0],raw[:,col],where='post',label='/command/cmd_twist',lw=1)
    ax.plot(final[:,0],final[:,col],'.-',label='/cmd_vel',ms=3,lw=.5)
    ax.set_xlim(60,63);ax.set_ylabel(label);ax.grid(alpha=.2);ax.legend()
axes[-1].set_xlabel('Bag time (s)')
fig.suptitle('Final command includes exact raw samples and an additional ~20 Hz stream')
fig.savefig(OUT/'command_streams.png',dpi=160);plt.close(fig)
print(json.dumps({k:v for k,v in metrics.items() if k not in ['topic_serialized_sizes','events']},indent=2))
