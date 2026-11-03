import sqlite3, json, collections, bisect
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
B=Path('/home/nvidia/MPPI_3dnavi/rosbag_recording/bags/terrain_elevation/teleop_flat_2026-10-19_20-47-37')
O=Path('/home/nvidia/MPPI_3dnavi/docs/analysis/teleop_flat_2026-10-19_20-47-37')
c=sqlite3.connect(f'file:{next(B.glob("*.db3"))}?mode=ro',uri=True)
topics={i:(n,get_message(t)) for i,n,t in c.execute('select id,name,type from topics')}
start=c.execute('select min(timestamp) from messages').fetchone()[0]/1e9
records=collections.defaultdict(list); edges=collections.defaultdict(list); maps=[]; clouds=[]; odom=[]; samples=[]
def ts(s):return s.sec+s.nanosec/1e9
def mat(tr):
 t=tr.transform.translation;q=tr.transform.rotation
 a=np.eye(4);a[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();a[:3,3]=[t.x,t.y,t.z];return a
for ti,t,raw in c.execute('select topic_id,timestamp,data from messages order by timestamp'):
 n,typ=topics[ti];m=deserialize_message(raw,typ);bt=t/1e9
 if n in ['/tf','/tf_static']:
  for tr in m.transforms:edges[(tr.header.frame_id,tr.child_frame_id)].append((ts(tr.header.stamp),bt,mat(tr),n))
  continue
 if n=='/map':continue
 ht=ts(m.header.stamp);records[n].append((bt,ht,m.header.frame_id))
 if 'elevation_map' in n:
  assert m.outer_start_index==m.inner_start_index==0
  arr=[np.array(d.data).reshape(70,70,order='F') for d in m.data]
  maps.append((bt,ht,np.array([m.info.pose.position.x,m.info.pose.position.y]),arr[0],arr[1]))
 elif n=='/aft_mapped_to_init':
  p=m.pose.pose.position;q=m.pose.pose.orientation
  odom.append((bt,ht,np.array([p.x,p.y,p.z]),np.array([q.x,q.y,q.z,q.w])))
 elif n=='/cloud_registered':
  fields={f.name:f.offset for f in m.fields}
  dtype=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[fields[k] for k in ['x','y','z']],'itemsize':m.point_step})
  a=np.frombuffer(m.data,dtype=dtype);p=np.stack([a[k] for k in ['x','y','z']],axis=-1)
  clouds.append((bt,ht,len(p),int(np.isfinite(p).all(axis=1).sum())))
  if (len(clouds)-1)%10==0:samples.append((bt,ht,p.copy()))
np.savez_compressed('/tmp/teleop_cloud_samples.npz',**{f'p{i}':r[2] for i,r in enumerate(samples)},times=np.array([[r[0],r[1]] for r in samples]),T=edges[('map','camera_init')][0][2])
print('loaded',len(maps),len(clouds),len(samples),flush=True)
def stats(a):
 a=np.asarray(a);a=a[np.isfinite(a)]
 return dict(zip(['min','p05','median','p95','max'],map(float,np.percentile(a,[0,5,50,95,100])))) if len(a) else {}
summary={'start_epoch':start,'topics':{},'tf':{}}
for n,rs in records.items():
 a=np.array([(r[0],r[1]) for r in rs]);summary['topics'][n]={'count':len(a),'frames':sorted(set(r[2] for r in rs)),'bag_dt_s':stats(np.diff(a[:,0])),'header_dt_s':stats(np.diff(a[:,1])),'age_s':stats(a[:,0]-a[:,1]),'duplicate_headers':int(np.sum(np.diff(a[:,1])==0)),'backward_headers':int(np.sum(np.diff(a[:,1])<0))}
for e,rs in edges.items():
 arr=np.stack([r[2] for r in rs]);eu=Rotation.from_matrix(arr[:,:3,:3]).as_euler('xyz',degrees=True)
 summary['tf']['->'.join(e)]={'count':len(rs),'translation_first':arr[0,:3,3].tolist(),'rpy_first_deg':eu[0].tolist(),'translation_range':np.ptp(arr[:,:3,3],axis=0).tolist(),'rpy_range_deg':np.ptp(np.unwrap(eu,axis=0,period=360),axis=0).tolist(),'sources':sorted(set(r[3] for r in rs))}
print(json.dumps(summary,indent=2),flush=True)
# map <- camera_init and camera_init <- aft_mapped, aft_mapped <- base_link chain
for e in edges:edges[e].sort(key=lambda r:r[0])
def lookup(parent,child,t):
 rs=edges[(parent,child)];idx=bisect.bisect_right([r[0] for r in rs],t)-1
 return rs[max(idx,0)][2]
def base(t):return lookup('map','camera_init',t)@lookup('camera_init','aft_mapped',t)@lookup('aft_mapped','base_link',t)
mt=np.array([r[1] for r in maps]); mb=np.array([r[0] for r in maps]);centers=np.array([r[2] for r in maps]);E=np.stack([r[3] for r in maps]);V=np.stack([r[4] for r in maps]);poses=np.stack([base(t) for t in mt]);bp=poses[:,:3,3];rpy=Rotation.from_matrix(poses[:,:3,:3]).as_euler('xyz',degrees=True)
summary['robot']={'distance_m':float(np.linalg.norm(np.diff(bp[:,:2],axis=0),axis=1).sum()),'net_m':float(np.linalg.norm(bp[-1,:2]-bp[0,:2])),'xyz_min':bp.min(axis=0).tolist(),'xyz_max':bp.max(axis=0).tolist(),'rpy_deg':{k:stats(rpy[:,j]) for j,k in enumerate(['roll','pitch','yaw'])},'speed_mps':stats(np.linalg.norm(np.diff(bp[:,:2],axis=0),axis=1)/np.diff(mt))}
coverage=np.isfinite(E).mean(axis=(1,2));close=[]
# decoded row=-X, col=-Y
ix,iy=np.indices((70,70));offx=3.45-.1*ix;offy=3.45-.1*iy
for i in range(len(E)):
 near=offx**2+offy**2<1.5**2;close.append(np.isfinite(E[i][near]).mean())
summary['map']={'coverage':stats(coverage),'coverage_near_1p5m':stats(close),'center_error_m':stats(np.linalg.norm(centers-bp[:,:2],axis=1)),'elevation':stats(E),'valid_variance':stats(V[np.isfinite(E)]),'identical_elevation_frames':int(np.sum([np.array_equal(E[i],E[i-1],equal_nan=True) for i in range(1,len(E))]))}
print('ROBOT',summary['robot'],'MAP',summary['map'],flush=True)
# Cloud vs maps sampled once per second. Local ground fit via iterative trimmed least squares in robot-centered ROI.
rows=[];snapshots=[]
for si,(bt,t,p) in enumerate(samples):
 T=lookup('map','camera_init',t);w=p@T[:3,:3].T+T[:3,3];b=base(t)[:3,3]
 mi=int(np.argmin(abs(mt-t)));ctr=centers[mi];e=E[mi]
 near=np.linalg.norm(w[:,:2]-b[:2],axis=1)<2.5
 # robust dominant surface near robot; limit z to ground region relative to base to exclude ceilings and tall objects
 g=w[near & (w[:,2]>b[2]-1.0)&(w[:,2]<b[2]+.1)]
 if len(g)<50:continue
 A=np.c_[g[:,:2]-b[:2],np.ones(len(g))]
 rng=np.random.default_rng(42);best=0;keep=np.ones(len(g),dtype=bool)
 for _ in range(120):
  ids=rng.choice(len(g),3,replace=False)
  try: trial=np.linalg.solve(A[ids],g[ids,2])
  except np.linalg.LinAlgError: continue
  if np.linalg.norm(trial[:2])>.15:continue
  mask=abs(g[:,2]-A@trial)<.025;count=int(mask.sum())
  if count>best:best=count;keep=mask
 for _ in range(3):
  coef=np.linalg.lstsq(A[keep],g[keep,2],rcond=None)[0];res=g[:,2]-A@coef
  keep=abs(res)<.025
 pred=np.c_[w[:,:2]-b[:2],np.ones(len(w))]@coef
 floor=abs(w[:,2]-pred)<.04
 ij=np.floor((ctr+3.5-w[:,:2])/.1).astype(int)
 inside=(ij>=0).all(axis=1)&(ij<70).all(axis=1)
 valid=inside&floor&near
 ii=ij[valid];zz=e[ii[:,0],ii[:,1]];pp=w[valid,2];finite=np.isfinite(zz)
 err=zz[finite]-pp[finite]
 # aggregate per visible ground grid cell
 ids=ii[:,0]*70+ii[:,1];unique=np.unique(ids);cellerr=[];missing=0
 for cell in unique:
  z=e.flat[cell]
  if np.isfinite(z):cellerr.append(z-np.median(pp[ids==cell]))
  else:missing+=1
 # evaluate map ground consistency in near robot ROI
 xx=ctr[0]+offx;yy=ctr[1]+offy;region=(xx-b[0])**2+(yy-b[1])**2<2.5**2
 groundz=coef[0]*(xx-b[0])+coef[1]*(yy-b[1])+coef[2]
 de=e-groundz
 origin=T[:3,3]
 rec={'t':bt-start,'header_t':t,'map_dt':mt[mi]-t,'cloud_points':len(p),'ground_points':len(g),'plane_slope_deg':float(np.degrees(np.arctan(np.linalg.norm(coef[:2])))),'plane_a':coef[0],'plane_b':coef[1],'plane_z_at_robot':coef[2],'plane_rmse':float(np.sqrt(np.mean(res[keep]**2))),'base_z':b[2],'coverage':coverage[mi],'near_coverage':close[mi],'visible_floor_cells':len(unique),'visible_floor_missing_fraction':missing/max(len(unique),1),'floor_cell_abs_error_median':float(np.median(abs(np.array(cellerr)))) if cellerr else np.nan,'floor_cell_abs_error_p95':float(np.percentile(abs(np.array(cellerr)),95)) if cellerr else np.nan,'cloud_origin_to_robot_m':float(np.linalg.norm(origin-b)),'ground_range_from_camera_init_median':float(np.median(np.linalg.norm(p[floor&near],axis=1))) if np.any(floor&near) else np.nan}
 rows.append(rec)
 if si in [0,15,30,45,60,75,90,105,120,135]:snapshots.append((si,mi,w[::3],coef,b,rec))
print('FLOOR_STATS',json.dumps({k:stats([r[k] for r in rows]) for k in rows[0] if k!='header_t'},indent=2),flush=True)
summary['clouds']={'points':stats([r[2] for r in clouds]),'nonfinite':sum(r[2]-r[3] for r in clouds)}
summary['sampled_ground']={k:stats([r[k] for r in rows]) for k in rows[0] if k!='header_t'}
(O/'metrics.json').write_text(json.dumps(summary,indent=2))
import csv
with (O/'sampled_ground.csv').open('w') as f:
 wr=csv.DictWriter(f,fieldnames=list(rows[0]));wr.writeheader();wr.writerows(rows)
np.savez_compressed('/tmp/teleop_arrays.npz',mt=mt,mb=mb,start=start,E=E,V=V,centers=centers,bp=bp,rpy=rpy)
# General time series
fig,axs=plt.subplots(5,1,figsize=(12,14),sharex=True);t=mb-start
axs[0].plot(t,bp[:,0]-bp[0,0],label='base X');axs[0].plot(t,bp[:,1]-bp[0,1],label='base Y');axs[0].set_ylabel('Travel (m)');axs[0].legend()
axs[1].plot(t,rpy[:,0],label='Roll');axs[1].plot(t,rpy[:,1],label='Pitch');axs[1].set_ylabel('Base angle (deg)');axs[1].legend()
axs[2].plot(t,coverage*100,label='7 x 7 m map');axs[2].plot(t,np.array(close)*100,label='within 1.5 m');axs[2].set_ylabel('Valid coverage (%)');axs[2].legend()
rt=[r['t'] for r in rows]
axs[3].plot(rt,[r['plane_slope_deg'] for r in rows],label='Local cloud ground slope');axs[3].set_ylabel('Slope (deg)');axs[3].legend()
axs[4].plot(rt,[r['visible_floor_missing_fraction']*100 for r in rows],label='Visible ground cells missing in map');axs[4].set_ylabel('Missing (%)');axs[4].legend();axs[4].set_xlabel('Seconds from bag start')
for ax in axs:ax.grid(alpha=.3)
fig.tight_layout();fig.savefig(O/'update_timeline.png',dpi=160);plt.close(fig)
# Map ground residual snapshots with cloud overlay and actual global axes
fig,axs=plt.subplots(2,5,figsize=(21,9))
for ax,(si,mi,w,coef,b,rec) in zip(axs.flat,snapshots):
 ctr=centers[mi];xx=ctr[0]+offx;yy=ctr[1]+offy
 plane=coef[0]*(xx-b[0])+coef[1]*(yy-b[1])+coef[2]
 a=(E[mi]-plane).T[::-1,::-1]
 im=ax.imshow(a,origin='lower',extent=[ctr[0]-3.5,ctr[0]+3.5,ctr[1]-3.5,ctr[1]+3.5],cmap='coolwarm',vmin=-.3,vmax=.3)
 # points near fitted ground only
 pred=coef[0]*(w[:,0]-b[0])+coef[1]*(w[:,1]-b[1])+coef[2]
 ground=abs(w[:,2]-pred)<.04
 ax.scatter(w[ground,0],w[ground,1],s=.3,c='black',alpha=.4);ax.plot(b[0],b[1],'g*',ms=10)
 ax.set_xlim(ctr[0]-3.5,ctr[0]+3.5);ax.set_ylim(ctr[1]-3.5,ctr[1]+3.5);ax.set_title(f"t={rec['t']:.1f}s, valid={rec['coverage']*100:.0f}%");ax.set_aspect('equal');ax.set_xlabel('Map X (m)');ax.set_ylabel('Map Y (m)')
fig.colorbar(im,ax=axs.ravel().tolist(),label='Elevation minus fitted ground (m)',shrink=.8);fig.savefig(O/'map_ground_snapshots.png',dpi=160,bbox_inches='tight');plt.close(fig)
print('SAVED',O,flush=True)
