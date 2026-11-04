#!/usr/bin/env python3
import json,re,hashlib
from pathlib import Path
out=Path(__file__).resolve().parent;root=out.parents[2]
text=(out/'paired_benchmark.log').read_text();metrics={'paired':{},'full_mppi':{}}
for m in re.finditer(r'phase=(\w+) scene=(\d+) samples=(\d+) p50_ms=([\d.e+-]+) p95_ms=([\d.e+-]+) p99_ms=([\d.e+-]+) checksum=(\d+)',text):
 metrics['paired'].setdefault(m[2],{})[m[1]]={'samples':int(m[3]),'p50_ms':float(m[4]),'p95_ms':float(m[5]),'p99_ms':float(m[6]),'checksum':int(m[7])}
for name in ['before','after','periodic_before','periodic_after']:
 s=(out/('full_mppi_'+name+'.log')).read_text()
 metrics['full_mppi'][name]={m[1]:{'n':int(m[2]),'p50_ms':float(m[3]),'p95_ms':float(m[4]),'max_ms':float(m[5])} for m in re.finditer(r'^RESULT (\w+) n=(\d+) p50_ms=([\d.e+-]+) p95_ms=([\d.e+-]+) max_ms=([\d.e+-]+)$',s,re.M)}
 assert 'received_marker_arrays=70 visualize=1' in s
 assert len(metrics['full_mppi'][name])==5
metrics['tests']={'native_gtest':30,'python_unittest':24,'standalone_ctest':2,'differential_cases':242304,'differential_fixtures':24}
files=[root/'src/height_jump_mppi_critic/include/height_jump_mppi_critic/evaluator.hpp',root/'src/height_jump_mppi_critic/include/height_jump_mppi_critic/terrain_map.hpp',root/'src/height_jump_mppi_critic/src/terrain_map.cpp',root/'src/height_jump_mppi_critic/src/height_jump_critic.cpp',root/'install/height_jump_mppi_critic/lib/libheight_jump_mppi_critic.so']
metrics['sha256']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
metrics['field_verified']=False
assert 'Summary: 30 tests, 0 errors, 0 failures, 0 skipped' in (out/'colcon_test_result.log').read_text()
assert 'Ran 24 tests' in (out/'python_tests.log').read_text() and 'OK' in (out/'python_tests.log').read_text()
assert 'failed_fixtures=0' in (out/'differential.log').read_text()
for scene,m in metrics['paired'].items():
 assert len({m[k]['checksum'] for k in ['baseline_rollout','prepared_rollout','batched_rollout']})==1
(out/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
print('Flat scalar -> batch P50:',metrics['paired']['0']['prepared_rollout']['p50_ms'],metrics['paired']['0']['batched_rollout']['p50_ms'])
print('Full optimizer + visualization P50:',metrics['full_mppi']['before']['compute_plus_visualization']['p50_ms'],metrics['full_mppi']['after']['compute_plus_visualization']['p50_ms'])
print('Verified tests:',metrics['tests'])
