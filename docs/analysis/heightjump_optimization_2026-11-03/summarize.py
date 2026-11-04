#!/usr/bin/env python3
import json,re,hashlib
from pathlib import Path
out=Path(__file__).resolve().parent
s=(out/'kernel_results.log').read_text()
metrics={}
metrics['timings']={m[1]:dict(zip(['p50_ms','p95_ms','max_ms'],map(float,m.groups()[1:]))) for m in re.finditer(r'^TIMING (\w+) p50_ms=([\d.e+-]+) p95_ms=([\d.e+-]+) max_ms=([\d.e+-]+)$',s,re.M)}
metrics['reuse']=[dict(zip(['frame','points','inside','unique_center_cells','unique_neighborhoods','center_cells_with_multiple_neighborhoods'],map(int,m.groups()))) for m in re.finditer(r'^REUSE frame=(\d+) points=(\d+) inside=(\d+) unique_center_cells=(\d+) unique_neighborhoods=(\d+) center_cells_with_multiple_neighborhoods=(\d+)$',s,re.M)]
metrics['boundaries']=[{'resolution':float(m[1]),'cells':int(m[2]),'multiple_neighborhood_cells':int(m[3]),'max_variants_per_cell':int(m[4])} for m in re.finditer(r'^BOUNDARY resolution=([\d.]+) cells=(\d+) center_cells_with_multiple_neighborhoods=(\d+) max_variants_per_cell=(\d+)$',s,re.M)]
metrics['query_count']=int(re.search(r'^QUERY_COUNT (\d+)$',s,re.M)[1])
metrics['scope']='Offline component microbenchmarks, synthetic interiors and six recorded visualization frames; not measured optimized controller performance.'
root=out.parents[2]
files=[out/'kernel_analysis.cpp',root/'src/height_jump_mppi_critic/include/height_jump_mppi_critic/terrain_map.hpp',root/'src/height_jump_mppi_critic/include/height_jump_mppi_critic/evaluator.hpp',root/'src/height_jump_mppi_critic/src/height_jump_critic.cpp',root/'install/height_jump_mppi_critic/lib/libheight_jump_mppi_critic.so']
metrics['sha256']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
assert metrics['query_count']==504000 and len(metrics['reuse'])==6 and len(metrics['timings'])==7
assert all(x['multiple_neighborhood_cells']>0 for x in metrics['boundaries'])
(out/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
print('Recorded 7 component measurements, 6 displayed-rollout reuse frames and 2 boundary audits.')
print('Total displayed points',sum(r['points'] for r in metrics['reuse']),'sum of per-frame unique neighborhoods',sum(r['unique_neighborhoods'] for r in metrics['reuse']))
