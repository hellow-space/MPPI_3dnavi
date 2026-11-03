#!/usr/bin/env python3
"""Collect diagnostic results; all measurements are offline, not robot validation."""
import hashlib,json,re
from pathlib import Path
out=Path(__file__).resolve().parent
modes=['current_visualization','sparse_visualization','no_visualization','no_height_diagnostic','sparse_periodic_20hz']
results={}
pattern=re.compile(r'^RESULT (\w+) n=(\d+) p50_ms=([\d.e+-]+) p95_ms=([\d.e+-]+) max_ms=([\d.e+-]+)$',re.M)
for mode in modes:
 s=(out/(mode+'.log')).read_text()
 rows={m[1]:{'n':int(m[2]),'p50_ms':float(m[3]),'p95_ms':float(m[4]),'max_ms':float(m[5])} for m in pattern.finditer(s)}
 if len(rows)!=5:raise ValueError(f'{mode}: missing measurements')
 flags=re.search(r'received_marker_arrays=(\d+) visualize=(\d+) periodic=(\d+)',s)
 if not flags:raise ValueError(f'{mode}: missing status')
 results[mode]={'stages':rows,'received_marker_arrays':int(flags[1]),'visualize':bool(int(flags[2])),'periodic':bool(int(flags[3]))}
assert results['current_visualization']['visualize']
assert results['sparse_visualization']['visualize']
assert not results['no_visualization']['visualize']
assert not results['no_height_diagnostic']['visualize']
a=results['current_visualization']['stages']['compute_plus_visualization']['p50_ms']
b=results['sparse_visualization']['stages']['compute_plus_visualization']['p50_ms']
results['summary']={'p50_saved_ms':a-b,'p50_reduction_percent':100*(a-b)/a,'offline_only':True,'sensor_pipeline_and_field_control_rate_measured':False}
results['sha256']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [out/'probe.cpp',out/'CMakeLists.txt',out/'baseline_controller.yaml',Path('/opt/ros/humble/lib/libmppi_controller.so'),Path('/home/nvidia/MPPI_3dnavi/install/height_jump_mppi_critic/lib/libheight_jump_mppi_critic.so')]}
(out/'metrics.json').write_text(json.dumps(results,indent=2)+'\n')
for mode in modes:
 r=results[mode]['stages'];print(mode,'total P50/P95:',r['compute_plus_visualization']['p50_ms'],r['compute_plus_visualization']['p95_ms'])
print(results['summary'])
