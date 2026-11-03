#!/usr/bin/env python3
"""Fixed-world ground cell comparisons across changes of robot body height."""
import json,sqlite3
from pathlib import Path
from collections import defaultdict
import numpy as np,pandas as pd
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path('/home/nvidia/MPPI_3dnavi');O=ROOT/'docs/analysis/body_height_stability';summary=json.loads((O/'summary.json').read_text());results={}
for stamp,phases in [('20-05-51',[(45,55),(65,80)]),('20-47-37',[(0,10),(45,70),(80,85)])]:
 name='teleop_flat_2026-10-19_'+stamp;B=ROOT/'rosbag_recording/bags/terrain_elevation'/name
 C=sqlite3.connect(f'file:{next(B.glob("*.db3"))}?mode=ro',uri=True);start=C.execute('select min(timestamp) from messages').fetchone()[0]/1e9
 tid=C.execute("select id from topics where name='/cloud_registered'").fetchone()[0]
 metas=C.execute('select id,timestamp from messages where topic_id=? order by timestamp',(tid,)).fetchall()[::10]
 f=pd.read_csv(O/(name+'_samples.csv'));a=dict(np.load('/tmp/'+name+'_posture.npz'));T=np.array(summary[name]['map_camera']);anchor=np.array(summary[name]['anchor_xy']);phase_data=[]
 for lo,hi in phases:
  cells=defaultdict(list);maps=defaultdict(list);body=[]
  for k,(mid,bt) in enumerate(metas):
   t=bt/1e9-start
   if not lo<=t<hi:continue
   r=f.iloc[k];raw=C.execute('select data from messages where id=?',(mid,)).fetchone()[0];m=deserialize_message(raw,PointCloud2);fd={x.name:x.offset for x in m.fields};dt=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[fd[x] for x in ['x','y','z']],'itemsize':m.point_step});p=np.frombuffer(m.data,dtype=dt);p=np.stack([p[x] for x in ['x','y','z']],1);w=p@T[:3,:3].T+T[:3,3]
   zpred=(w[:,:2]-anchor)@np.array([r.plane_a,r.plane_b])+r.cloud_ground_ref_z
   ground=(abs(w[:,2]-zpred)<.025)&(np.linalg.norm(w[:,:2]-[r.base_x,r.base_y],axis=1)<3.5)
   w=w[ground];ij=np.floor(w[:,:2]/.1).astype(int);uniq,inv=np.unique(ij,axis=0,return_inverse=True);mi=int(np.argmin(abs(a['mt']-r.source_t)));e=a['E'][mi];ctr=np.round(a['centers'][mi]/.1)*.1
   for j,key in enumerate(uniq):
    zz=w[inv==j,2]
    if len(zz)<3:continue
    key=tuple(key);cells[key].append(float(np.median(zz)));worldxy=np.array(key)*.1+.05;gmidx=np.floor((ctr+3.5-worldxy)/.1).astype(int)
    if (gmidx>=0).all() and (gmidx<70).all():
     v=e[tuple(gmidx)]
     if np.isfinite(v):maps[key].append(float(v))
   body.append(r.base_z)
  phase_data.append((cells,maps,float(np.median(body))))
 rs=[]
 for k in range(1,len(phases)):
  a0,m0,b0=phase_data[k-1];a1,m1,b1=phase_data[k];keys=[key for key in a0.keys()&a1.keys() if len(a0[key])>=2 and len(a1[key])>=2];d=np.array([np.median(a1[key])-np.median(a0[key]) for key in keys]);mk=[key for key in keys if key in m0 and key in m1];md=np.array([np.median(m1[key])-np.median(m0[key]) for key in mk]);rec={'phase_before':phases[k-1],'phase_after':phases[k],'body_z_change_m':b1-b0,'common_ground_cells':len(keys),'ground_z_change_median_m':float(np.median(d)) if len(d) else None,'ground_z_change_abs_p95_m':float(np.percentile(abs(d),95)) if len(d) else None,'map_cells':len(mk),'map_z_change_median_m':float(np.median(md)) if len(md) else None,'map_z_change_abs_p95_m':float(np.percentile(abs(md),95)) if len(md) else None};rs.append(rec)
 for rec,k in zip(rs,range(1,len(phases))):
  cb=f[(f.t>=phases[k-1][0])&(f.t<phases[k-1][1])].base_ground_clearance.median()
  ca=f[(f.t>=phases[k][0])&(f.t<phases[k][1])].base_ground_clearance.median()
  rec['body_ground_clearance_before_m']=float(cb);rec['body_ground_clearance_after_m']=float(ca);rec['body_ground_clearance_change_m']=float(ca-cb)
  c0,m0,_=phase_data[k-1];c1,m1,_=phase_data[k]
  cr=[]
  for key in c0.keys()&c1.keys():
   if len(c0[key])<2 or len(c1[key])<2 or key not in m0 or key not in m1:continue
   cr.append({'x':key[0]*.1+.05,'y':key[1]*.1+.05,'cloud_before':float(np.median(c0[key])),'cloud_after':float(np.median(c1[key])),'map_before':float(np.median(m0[key])),'map_after':float(np.median(m1[key]))})
  delta=np.array([r['map_after']-r['map_before'] for r in cr]);rec['map_delta_within_2cm_fraction']=float(np.mean(abs(delta)<.02));rec['map_delta_over_10cm_cells']=int(np.sum(abs(delta)>.1))
  (O/(name+f'_phase_{k}_cells.json')).write_text(json.dumps(cr,indent=2))
 results[name]=rs
 print(name,json.dumps(rs),flush=True)
(O/'phase_comparison.json').write_text(json.dumps(results,indent=2))
# Direct evidence plot uses a fixed observed ground tile; raw map excursions retained.
fig,axes=plt.subplots(2,2,figsize=(14,9))
for col,stamp in enumerate(['20-05-51','20-47-37']):
 name='teleop_flat_2026-10-19_'+stamp;tr=json.loads((O/(name+'_tracked_cells.json')).read_text());rec=tr[1] if col==0 else tr[0];ch=np.array(rec['cloud']);mh=np.array(rec['map']);a=dict(np.load('/tmp/'+name+'_posture.npz'))
 axes[0,col].plot(a['rt'],a['rb'][:,2],label='Body Z',color='C0');axes[0,col].set_title(stamp);axes[0,col].set_ylabel('Body Z in map (m)');axes[0,col].legend();axes[0,col].grid(alpha=.3)
 axes[1,col].scatter(ch[:,0],ch[:,1],s=12,label='Current ground cloud tile',color='C1');axes[1,col].plot(mh[:,0],mh[:,1],label='Elevation at same fixed XY',color='C2');axes[1,col].set_ylabel('Ground Z in map (m)');axes[1,col].set_xlabel('Seconds from bag start');axes[1,col].legend(fontsize=9);axes[1,col].grid(alpha=.3)
 if col==1:
  axes[0,col].set_xlim(0,85);axes[1,col].set_xlim(0,85);axes[1,col].set_ylim(-.34,-.285)
 else:
  axes[0,col].set_xlim(20,85);axes[1,col].set_xlim(20,85)
 axes[1,col].set_title(f"Fixed tile X={rec['cell']['x']:.2f}, Y={rec['cell']['y']:.2f} m")
fig.suptitle('Height compensation: body moves vertically while fixed ground stays near its original height');fig.tight_layout(rect=(0,0,1,.96));fig.savefig(O/'fixed_ground_height_compensation.png',dpi=150);plt.close(fig)
