#!/usr/bin/env python3
"""Read-only three-bag body-height/ground stability diagnostic. Requires ROS 2 Humble."""
import sqlite3,json,csv,bisect
from pathlib import Path
from collections import defaultdict
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path('/home/nvidia/MPPI_3dnavi');OUT=ROOT/'docs/analysis/body_height_stability';OUT.mkdir(exist_ok=True)
def ts(s):return s.sec+s.nanosec/1e9
def matrix(tr):
 p=tr.transform.translation;q=tr.transform.rotation;a=np.eye(4)
 a[:3,3]=[p.x,p.y,p.z];a[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();return a
def stats(x):
 x=np.array(x);x=x[np.isfinite(x)]
 return {k:float(v) for k,v in zip(['min','p05','median','p95','max'],np.percentile(x,[0,5,50,95,100]))} if len(x) else {}
def fitplane(g,anchor):
 A=np.c_[g[:,:2]-anchor[:2],np.ones(len(g))];rng=np.random.default_rng(42);best=0;keep=None
 for _ in range(120):
  ids=rng.choice(len(g),3,replace=False)
  try:coef=np.linalg.solve(A[ids],g[ids,2])
  except np.linalg.LinAlgError:continue
  if np.linalg.norm(coef[:2])>.8:continue
  mask=abs(g[:,2]-A@coef)<.025;score=mask.sum()
  if score>best:best=score;keep=mask
 if keep is None or best<30:return None
 for _ in range(3):
  coef=np.linalg.lstsq(A[keep],g[keep,2],rcond=None)[0];res=g[:,2]-A@coef;keep=abs(res)<.025
 return coef,float(np.sqrt(np.mean(res[keep]**2))),int(keep.sum())
allsummary={};allseries=[]
for B in sorted((ROOT/'rosbag_recording/bags/terrain_elevation').glob('teleop_flat*')):
 print('START',B.name,flush=True)
 C=sqlite3.connect(f'file:{next(B.glob("*.db3"))}?mode=ro',uri=True)
 topics={n:(i,get_message(t)) for i,n,t in C.execute('select id,name,type from topics')}
 start,end=C.execute('select min(timestamp),max(timestamp) from messages').fetchone();start/=1e9;end/=1e9
 edges=defaultdict(list);maps=[]
 for topic in ['/tf','/tf_static']:
  tid,typ=topics[topic]
  for bt,raw in C.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid,)):
   for tr in deserialize_message(raw,typ).transforms:edges[(tr.header.frame_id,tr.child_frame_id)].append((ts(tr.header.stamp),bt/1e9,matrix(tr)))
 def lookup(key,t):
  rr=edges[key];tt=np.array([x[0] for x in rr]);j=int(np.searchsorted(tt,t))
  if j==0:return rr[0][2]
  if j>=len(rr):return rr[-1][2]
  if tt[j]<=tt[j-1]:return rr[j][2]
  q=(t-tt[j-1])/(tt[j]-tt[j-1]);a=rr[j-1][2];b=rr[j][2];M=np.eye(4)
  M[:3,3]=(1-q)*a[:3,3]+q*b[:3,3]
  M[:3,:3]=Slerp(tt[j-1:j+1],Rotation.from_matrix(np.stack([a[:3,:3],b[:3,:3]])))(t).as_matrix();return M
 for k in edges:edges[k].sort(key=lambda x:x[0])
 def robot(t):return lookup(('map','camera_init'),t)@lookup(('camera_init','aft_mapped'),t)@lookup(('aft_mapped','base_link'),t)
 tid,typ=topics['/elevation_mapping_node/elevation_map']
 for bt,raw in C.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid,)):
  m=deserialize_message(raw,typ);d=m.data[m.layers.index('elevation')];cols=d.layout.dim[0].size;rows=d.layout.dim[1].size
  assert m.outer_start_index==m.inner_start_index==0
  e=np.array(d.data).reshape(rows,cols,order='F');p=m.info.pose.position
  maps.append((bt/1e9,ts(m.header.stamp),np.array([p.x,p.y]),e,m.info.resolution))
 mt=np.array([m[1] for m in maps]);mp=np.stack([robot(t) for t in mt]);bp=mp[:,:3,3];rpy=Rotation.from_matrix(mp[:,:3,:3]).as_euler('xyz',degrees=True)
 tref=edges[('map','camera_init')][0][0];anchor=robot(tref)[:3,3]
 # Timeline cloud poses at every original dynamic TF timestamp.
 rt=np.array([x[0] for x in edges[('camera_init','aft_mapped')]]);rp=np.stack([robot(t) for t in rt]);rb=rp[:,:3,3];re=Rotation.from_matrix(rp[:,:3,:3]).as_euler('xyz',degrees=True)
 summary={'duration_s':end-start,'map_start_s':maps[0][0]-start,'map_count':len(maps),'map_camera':edges[('map','camera_init')][0][2].tolist(),'anchor_xy':anchor[:2].tolist(),'base_z':stats(rb[:,2]),'base_xy_extent_m':np.ptp(rb[:,:2],axis=0).tolist(),'body_roll_deg':stats(re[:,0]),'body_pitch_deg':stats(re[:,1]),'tf':{('->'.join(k)):{'count':len(v),'first_matrix':v[0][2].tolist(),'first_bag_s':v[0][1]-start} for k,v in edges.items()}}
 tid,typ=topics['/cloud_registered'];metas=C.execute('select id,timestamp from messages where topic_id=? order by timestamp',(tid,)).fetchall()
 series=[];cellhistory=defaultdict(list);globalpoints=[]
 for sid,(mid,bt) in enumerate(metas[::10]):
  raw=C.execute('select data from messages where id=?',(mid,)).fetchone()[0];m=deserialize_message(raw,typ);t=ts(m.header.stamp)
  fd={f.name:f.offset for f in m.fields};dt=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[fd[x] for x in ['x','y','z']],'itemsize':m.point_step});p=np.frombuffer(m.data,dtype=dt);p=np.stack([p[x] for x in ['x','y','z']],axis=1)
  T=lookup(('map','camera_init'),t);w=p@T[:3,:3].T+T[:3,3];R=robot(t);body=R[:3,3];angles=Rotation.from_matrix(R[:3,:3]).as_euler('xyz',degrees=True)
  # Broad ground candidate band covers crouched and upright body heights. Exclude high surfaces; fit dominant low plane.
  dist=np.linalg.norm(w[:,:2]-body[:2],axis=1);candidate=(dist<3.5)&(w[:,2]>body[2]-1.2)&(w[:,2]<body[2]+.12)
  g=w[candidate]
  if len(g)<50:continue
  fitted=fitplane(g,anchor)
  if fitted is None:continue
  coef,rmse,n=fitted;pred=(w[:,:2]-anchor[:2])@coef[:2]+coef[2];ground=abs(w[:,2]-pred)<.025
  gp=w[ground&(dist<3.5)];globalpoints.append(gp[::max(1,len(gp)//500)])
  # Fixed world cells: record medians without subtracting a per-frame plane.
  cell=np.floor(gp[:,:2]/.1).astype(int);unique,inv=np.unique(cell,axis=0,return_inverse=True)
  for j,(cx,cy) in enumerate(unique):
   zz=gp[inv==j,2]
   if len(zz)>=3:cellhistory[(int(cx),int(cy))].append((t-start,float(np.median(zz)),body[2]))
  mi=int(np.argmin(abs(mt-t)));btm,tm,ctr,e,res=maps[mi];ix,iy=np.indices(e.shape);xx=ctr[0]+(e.shape[0]-1)/2*res-ix*res;yy=ctr[1]+(e.shape[1]-1)/2*res-iy*res
  me=(e<coef[0]*(xx-anchor[0])+coef[1]*(yy-anchor[1])+coef[2]+.08)&(e>coef[0]*(xx-anchor[0])+coef[1]*(yy-anchor[1])+coef[2]-.08)
  roi=(xx-body[0])**2+(yy-body[1])**2<3.5**2;valid=me&roi
  mg=np.nan
  if valid.sum()>50:
   # Keep cloud plane slope fixed when fitting map offset, so frame rotations are not silently removed.
   mg=float(np.median(e[valid]-coef[0]*(xx[valid]-anchor[0])-coef[1]*(yy[valid]-anchor[1])))
  rec={'t':bt/1e9-start,'source_t':t-start,'base_x':body[0],'base_y':body[1],'base_z':body[2],'roll_deg':angles[0],'pitch_deg':angles[1],'yaw_deg':angles[2],'cloud_ground_ref_z':coef[2],'ground_slope_deg':np.degrees(np.arctan(np.linalg.norm(coef[:2]))),'plane_a':coef[0],'plane_b':coef[1],'ground_fit_rmse':rmse,'ground_inliers':n,'map_ground_ref_z':mg,'map_cloud_time_gap_s':tm-t,'map_coverage':np.isfinite(e).mean(),'base_ground_clearance':body[2]-(coef[:2]@(body[:2]-anchor[:2])+coef[2])}
  series.append(rec)
 series.sort(key=lambda r:r['t'])
 with (OUT/(B.name+'_samples.csv')).open('w') as f:
  wr=csv.DictWriter(f,fieldnames=list(series[0]));wr.writeheader();wr.writerows(series)
 # Most revisited fixed cells, selected by observed body-height span and sample count.
 cellrows=[]
 for cell,hist in cellhistory.items():
  h=np.array(hist)
  if len(h)<8:continue
  cellrows.append({'x':cell[0]*.1+.05,'y':cell[1]*.1+.05,'n':len(h),'ground_z_range':float(np.ptp(h[:,1])),'ground_z_p95_p05':float(np.percentile(h[:,1],95)-np.percentile(h[:,1],5)),'body_z_range':float(np.ptp(h[:,2])),'first_s':float(h[0,0]),'last_s':float(h[-1,0])})
 cellrows.sort(key=lambda r:(-r['body_z_range'],-r['n']))
 (OUT/(B.name+'_fixed_cells.json')).write_text(json.dumps(cellrows,indent=2))
 if cellrows:
  summary['best_height_test_cells']=cellrows[:10]
  tracked=[]
  for cellrec in cellrows[:10]:
   key=(int(np.floor(cellrec['x']/.1)),int(np.floor(cellrec['y']/.1)))
   ch=np.array(cellhistory[key]);mh=[]
   for bt,ht,ctr,e,res in maps:
    # Infer snapped center from current zero-origin implementation. Also keep actual published geometry for comparison.
    qctr=np.round(ctr/res)*res
    ij=np.floor((qctr+np.array(e.shape)*res/2-np.array([cellrec['x'],cellrec['y']]))/res).astype(int)
    if (ij>=0).all() and (ij<np.array(e.shape)).all():
     val=e[tuple(ij)]
     if np.isfinite(val):mh.append([ht-start,float(val),float(robot(ht)[2,3])])
   tracked.append({'cell':cellrec,'cloud':ch.tolist(),'map':mh,'map_z_stats':stats([r[1] for r in mh])})
  (OUT/(B.name+'_tracked_cells.json')).write_text(json.dumps(tracked,indent=2))
 summary['samples']={key:stats([r[key] for r in series]) for key in ['base_z','cloud_ground_ref_z','map_ground_ref_z','ground_slope_deg','ground_fit_rmse','base_ground_clearance']};summary['cloud_sample_count']=len(series)
 # Evaluate shared global reference plane: all samples contribute, unlike changing local fitted offsets.
 gp=np.concatenate(globalpoints);fp=fitplane(gp,anchor);summary['aggregate_ground_plane']={'coef':fp[0].tolist(),'rmse':fp[1],'inliers':fp[2]}
 allsummary[B.name]=summary;allseries.append((B.name,series,rt-start,rb,re))
 np.savez_compressed('/tmp/'+B.name+'_posture.npz',rt=rt-start,rb=rb,re=re,mt=mt-start,E=np.stack([m[3] for m in maps]),centers=np.stack([m[2] for m in maps]))
 print('SUMMARY',B.name,json.dumps({k:v for k,v in summary.items() if k not in ['tf','map_camera','best_height_test_cells']}),flush=True)
 print('CELL_TOP',cellrows[:3],flush=True)
(OUT/'summary.json').write_text(json.dumps(allsummary,indent=2))
fig,axes=plt.subplots(5,3,figsize=(19,17),sharex='col')
for col,(name,series,t,b,rpy) in enumerate(allseries):
 tt=np.array([r['t'] for r in series]);rows=series;axes[0,col].set_title(name[-8:])
 axes[0,col].plot(t,b[:,2],label='Body Z');axes[0,col].plot(tt,[r['cloud_ground_ref_z'] for r in rows],label='Cloud ground at fixed XY');axes[0,col].plot(tt,[r['map_ground_ref_z'] if abs(r['map_cloud_time_gap_s'])<.2 else np.nan for r in rows],label='Map ground at fixed XY');axes[0,col].legend(fontsize=8)
 axes[1,col].plot(t,rpy[:,0],label='Roll');axes[1,col].plot(t,rpy[:,1],label='Pitch');axes[1,col].legend(fontsize=8)
 axes[2,col].plot(tt,[r['ground_slope_deg'] for r in rows]);axes[3,col].plot(tt,[r['base_ground_clearance'] for r in rows]);axes[4,col].plot(t,b[:,0]-b[0,0],label='Body X');axes[4,col].plot(t,b[:,1]-b[0,1],label='Body Y');axes[4,col].legend(fontsize=8);axes[4,col].set_xlabel('Seconds from bag start')
 for ax in axes[:,col]:ax.grid(alpha=.3)
for row,label in enumerate(['Z / reference height (m)','Body R/P (deg)','Cloud ground slope (deg)','Body above ground (m)','XY displacement (m)']):axes[row,0].set_ylabel(label)
fig.suptitle('Body height vs static ground: ground values are evaluated at fixed XY in each bag frame',fontsize=14);fig.tight_layout(rect=(0,0,1,.98));fig.savefig(OUT/'three_bag_posture_comparison.png',dpi=150);plt.close(fig)
print('DONE',OUT,flush=True)
