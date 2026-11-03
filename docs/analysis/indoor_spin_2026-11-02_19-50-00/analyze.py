#!/usr/bin/env python3
"""Offline timing, TF, command and path analysis. Uses extract.py's local cache."""
import pickle,json,re,csv
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.ndimage import gaussian_filter1d
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=pickle.load(open('/tmp/mppi_indoor_spin_data.pkl','rb'))
OUT=Path(__file__).resolve().parent
S=json.loads((OUT/'initial_metrics.json').read_text())
S['status']={'navigate':D['status']['/navigate_to_pose/_action/status'],'follow_last':[[t,s[-1]] for t,s in D['status']['/follow_path/_action/status']]}
(OUT/'initial_metrics.json').write_text(json.dumps(S,indent=2))
t0=D['t0']/1e9;dur=(D['t1']-D['t0'])/1e9
A=D['tf'][('camera_init','aft_mapped')];B=D['tf'][('map','camera_init')][0];C=D['static'][('aft_mapped','base_link')]
ra=Rotation.from_quat(A[:,6:10]);rb=Rotation.from_quat(B[6:10]);rc=Rotation.from_quat(C[4:8])
# Dynamic rows are receive, stamp, x,y,z,yaw,qx,qy,qz,qw. Static rows omit receive/stamp.
xyz=rb.apply(ra.apply(C[:3])+A[:,2:5])+B[2:5]
rot=rb*ra*rc
yaw=np.unwrap(rot.as_euler('ZYX')[:,0]);pose=np.column_stack([A[:,0],A[:,1],xyz,yaw])
raw=D['cmd']['/command/cmd_twist'];final=D['cmd']['/cmd_vel']

def stats(x):
 x=np.asarray(x);return {k:float(v) for k,v in zip(['min','median','p95','max'],np.percentile(x,[0,50,95,100]))}

def zoh(t,a,col):
 i=np.searchsorted(a[:,0],t,side='right')-1
 return a[np.clip(i,0,len(a)-1),col]

# Pose derivatives on receipt time for response-lag estimates; not chassis ground truth.
grid=np.arange(A[0,0],A[-1,0],.02)
yg=np.interp(grid,A[:,0],yaw)
wg=gaussian_filter1d(np.gradient(yg,.02),sigma=5)
xg=np.interp(grid,A[:,0],xyz[:,0]);ygp=np.interp(grid,A[:,0],xyz[:,1]);
vxg=gaussian_filter1d(np.gradient(xg,.02),sigma=5);vyg=gaussian_filter1d(np.gradient(ygp,.02),sigma=5)
vg=vxg*np.cos(yg)+vyg*np.sin(yg)
curves=[]
for lag in np.arange(0,6.01,.02):
 sel=grid>raw[0,0]+lag+.3
 u=zoh(grid[sel]-lag,raw,3);v=wg[sel]
 curves.append([lag,np.corrcoef(u,v)[0,1],np.sqrt(np.mean((u-v)**2))])
curves=np.asarray(curves);best=curves[np.argmax(curves[:,1])]
# Pair each controller sample with the closest identically-valued final sample.
matched=[];used=set()
for x in raw:
 ix=np.flatnonzero((np.max(np.abs(final[:,1:]-x[1:]),axis=1)<1e-9)&(final[:,0]>=x[0]-.01)&(final[:,0]<x[0]+.03))
 if len(ix):
  j=int(ix[np.argmin(np.abs(final[ix,0]-x[0]))]);used.add(j);matched.append(final[j,0]-x[0])
other=final[[i not in used for i in range(len(final))]]
latest=np.searchsorted(raw[:,0],other[:,0],side='right')-1
sel=latest>=0;delta=other[sel,3]-raw[latest[sel],3]
# Convert each path's latest reference distance in map; frames in this bag are map.
err=[]
for row in pose:
 t=row[0];v=[]
 for name in ['/plan','/transformed_global_plan']:
  ps=D['paths'][name];idx=np.searchsorted([p['t'] for p in ps],t,side='right')-1
  v.append(np.nan if idx<0 else np.sqrt(np.min(np.sum((ps[idx]['pts'][:,:2]-row[2:4])**2,axis=1))))
 err.append(v)
err=np.array(err)
# Logs: use logger time, raw incoming IMU stamp, lidar stamp and reported mismatch.
log=Path('/home/nvidia/.ros/log/fastlivo_mapping_8016_1793619834795.log');imu=[];lid=[];diff=[];hard=[]
for line in log.open():
 m=re.search(r'\[(179\d+\.\d+)\]',line)
 if not m:continue
 wall=float(m[1]);t=wall-t0
 if not 0<=t<=dur:continue
 for key,dest in [('get imu at time:',imu),('Get LiDAR, its header time:',lid)]:
  if key in line:
   st=float(line.split(key)[1].split()[0]);dest.append([t,st-t0,wall-st])
 if 'IMU and LiDAR not synced!' in line:diff.append([t,float(re.search(r'delta time: ([-\d.]+)',line)[1])])
 if 'HARD time lag is' in line:hard.append([t,float(re.search(r'lag is ([-\d.]+)',line)[1])])
imu=np.asarray(imu);lid=np.asarray(lid);diff=np.asarray(diff);hard=np.asarray(hard)
navlog=Path('/home/nvidia/.ros/log/component_container_isolated_8531_1793619849343.log');missed=[];events=[]
for num,line in enumerate(navlog.open(),1):
 m=re.search(r'\[(179\d+\.\d+)\]',line)
 if not m:continue
 t=float(m[1])-t0
 if not 0<=t<=dur:continue
 if 'Control loop missed' in line:missed.append(t)
 if '[ERROR]' in line or any(k in line for k in ['HeightJump','spin','recover','Failed','Aborting','Goal succeeded']):events.append([num,t,line.strip()])
M={'duration_s':dur,'command_rate_hz':(len(raw)-1)/(raw[-1,0]-raw[0,0]),'command_interval_s':stats(np.diff(raw[:,0])),'missed_control_loop_warnings':len(missed),'direct_forward_match_n':len(matched),'direct_forward_receive_time_difference_s':stats(matched),'other_final_messages':len(other),'other_final_rate_hz':(len(other)-1)/(other[-1,0]-other[0,0]),'other_final_wz_diff_from_latest_raw_abs':stats(abs(delta)),'other_final_opposite_wz_count':int(((other[sel,3]*raw[latest[sel],3]<0)&(abs(other[sel,3])>.1)&(abs(raw[latest[sel],3])>.1)).sum()),'odom_twist_all_zero':bool(np.all(D['odom'][:,6:]==0)),'pose_derived_wz_abs':stats(abs(wg)),'pose_derived_forward_speed_abs':stats(abs(vg)),'trajectory_length_m':float(np.sum(np.linalg.norm(np.diff(xyz[:,:2],axis=0),axis=1))),'net_displacement_m':float(np.linalg.norm(xyz[-1,:2]-xyz[0,:2])),'base_xyz_start':xyz[0].tolist(),'base_xyz_end':xyz[-1].tolist(),'base_rpy_deg_range':[(float(a.min()),float(a.max())) for a in rot.as_euler('xyz',degrees=True).T],'yaw_unwrapped_range_deg':[float(np.rad2deg(yaw.min())),float(np.rad2deg(yaw.max()))],'path_error_m':{n:stats(err[np.isfinite(err[:,i]),i]) for i,n in enumerate(['global','controller'])},'lidar_callback_minus_stamp_s':stats(lid[:,2]),'imu_callback_minus_stamp_s':stats(imu[:,2]),'imu_lidar_mismatch_s':stats(diff[:,1]),'imu_lidar_mismatch_warning_count':len(diff),'imu_callback_n':len(imu),'imu_callback_rate_hz':(len(imu)-1)/(imu[-1,0]-imu[0,0]),'imu_stamp_rate_vs_wall':float(np.polyfit(imu[:,0],imu[:,1],1)[0]),'response_lag_estimate':{'best_lag_s':float(best[0]),'correlation_at_best':float(best[1]),'correlation_at_zero_lag':float(curves[0,1]),'rmse_at_zero':float(curves[0,2]),'rmse_at_best':float(best[2]),'note':'command to localization pose derivative, not a direct chassis actuation latency measurement'},'nav_error_or_recovery_events_in_bag':events,'local_maps_all_zero':all(np.all(x['data']==0) for x in D['maps']['/local_costmap/costmap'])}
print(json.dumps(M,indent=2))
print('per 4s: t,mean raw vx/wz, mean pose vz/wz, median IMU age, mismatch, controller_err')
for start in range(0,33,4):
 r=raw[(raw[:,0]>=start)&(raw[:,0]<start+4)];g=(grid>=start)&(grid<start+4);im=imu[(imu[:,0]>=start)&(imu[:,0]<start+4)];di=diff[(diff[:,0]>=start)&(diff[:,0]<start+4)]
 print(start,np.round([r[:,1].mean(),r[:,3].mean(),vg[g].mean(),wg[g].mean(),np.median(im[:,2]),np.median(di[:,1])],3))
print('pose every 2sec:')
for t in range(0,33,2):
 j=np.argmin(abs(pose[:,0]-t));print(np.round(np.r_[pose[j,0],pose[j,2:5],np.rad2deg(yaw[j]),err[j]],3))
ii=np.searchsorted(imu[:,0],A[:,1],side='right')-1
valid=ii>=0
M['inferred_state_age_lower_bound_s_from_latest_imu']=stats(A[valid,1]-imu[ii[valid],1])
M['cumulative_abs_yaw_deg']=float(np.rad2deg(np.abs(np.diff(yaw)).sum()))
goal=D['paths']['/plan'][-1]['pts'][-1,:2]
M['goal_distance_start_end_m']=np.linalg.norm(xyz[[0,-1],:2]-goal,axis=1).tolist()
(OUT/'metrics.json').write_text(json.dumps(M,indent=2))
for name,a,head in [('pose',np.c_[pose,err],['receipt_s','header_s','map_x','map_y','map_z','unwrapped_yaw_rad','global_error_m','controller_error_m']),('imu_timing',imu,['callback_s','imu_header_s','callback_minus_header_s']),('lidar_timing',lid,['callback_s','lidar_header_s','callback_minus_header_s']),('response_lag',curves,['lag_s','correlation','rmse'])]:
 with (OUT/(name+'.csv')).open('w') as f:w=csv.writer(f);w.writerow(head);w.writerows(a)
plt.rcParams.update({'font.size':10})
fig,axes=plt.subplots(4,1,figsize=(12,11),sharex=True,constrained_layout=True)
axes[0].plot(raw[:,0],raw[:,1],label='MPPI command vx',lw=1);axes[0].plot(grid,vg,label='Pose-derived body vx (receipt time)',lw=1);axes[0].set_ylabel('m/s');axes[0].legend(loc='upper right')
axes[1].plot(raw[:,0],raw[:,3],label='MPPI command wz',lw=1);axes[1].plot(grid,wg,label='Pose-derived wz (receipt time)',lw=1);axes[1].plot(grid,np.where(grid-best[0]>=raw[0,0],zoh(grid-best[0],raw,3),np.nan),'--',label=f'Command delayed {best[0]:.2f}s (correlation estimate)',lw=1,alpha=.6);axes[1].set_ylabel('rad/s');axes[1].legend(loc='upper right',fontsize=8)
axes[2].plot(imu[:,0],imu[:,2],label='IMU callback - raw stamp',lw=.7);axes[2].plot(lid[:,0],lid[:,2],label='LiDAR callback - raw stamp',lw=1);axes[2].plot(raw[1:,0],np.diff(raw[:,0]),label='Controller receive interval',lw=.7);axes[2].axhline(.05,color='gray',ls=':',label='20Hz target period');axes[2].set_ylabel('seconds');axes[2].legend(loc='upper left',fontsize=8)
axes[3].plot(pose[:,0],err[:,1],label='Distance to current controller reference');axes[3].plot(pose[:,0],xyz[:,2],label='TF base height in map');axes[3].set_ylabel('m');axes[3].legend();axes[3].set_xlabel('Seconds since first bag message')
for ax in axes:ax.grid(alpha=.25);ax.set_xlim(0,dur)
fig.suptitle('Indoor navigation: slow controller + delayed localization feedback')
fig.savefig(OUT/'timing_and_motion.png',dpi=160);plt.close(fig)
fig,ax=plt.subplots(figsize=(9,9),constrained_layout=True)
m=D['maps']['/map'][0];im=m['data'].reshape(m['h'],m['w']);org=m['origin'];res=m['res']
ax.imshow(im,origin='lower',extent=[org[0],org[0]+res*m['w'],org[1],org[1]+res*m['h']],cmap='gray_r',vmin=0,vmax=100,alpha=.5)
for p in D['paths']['/plan'][::5]:ax.plot(p['pts'][:,0],p['pts'][:,1],color='gray',lw=.7,alpha=.5)
sc=ax.scatter(xyz[:,0],xyz[:,1],c=A[:,0],cmap='viridis',s=14);ax.plot(xyz[:,0],xyz[:,1],lw=.5,color='black',alpha=.3);fig.colorbar(sc,ax=ax,label='Bag time (s)')
for t in range(0,33,2):
 i=np.argmin(abs(A[:,0]-t));ax.annotate(f'{t}s',xyz[i,:2],xytext=(4,4),textcoords='offset points',fontsize=8);ax.arrow(xyz[i,0],xyz[i,1],.18*np.cos(yaw[i]),.18*np.sin(yaw[i]),head_width=.05,color='tab:red',alpha=.7)
goal=D['paths']['/plan'][-1]['pts'][-1,:2];ax.plot(*goal,'r*',ms=14,label='Current goal');ax.plot(*xyz[0,:2],'go',label='Bag start');ax.plot(*xyz[-1,:2],'rx',label='Bag end')
ax.set_xlim(min(xyz[:,0].min(),goal[0])-.8,max(xyz[:,0].max(),goal[0])+.8);ax.set_ylim(min(xyz[:,1].min(),goal[1])-.8,max(xyz[:,1].max(),goal[1])+.8);ax.set_aspect('equal');ax.set_xlabel('map x (m)');ax.set_ylabel('map y (m)');ax.set_title('Trajectory reconstructed with full 3D TF composition');ax.legend();ax.grid(alpha=.2);fig.savefig(OUT/'trajectory.png',dpi=160);plt.close(fig)
fig,axes=plt.subplots(2,1,figsize=(11,6),constrained_layout=True)
for ax,col,label in zip(axes,[1,3],['vx (m/s)','wz (rad/s)']):
 ax.step(raw[:,0],raw[:,col],where='post',label='Raw /command/cmd_twist',lw=1);ax.plot(final[:,0],final[:,col],'.-',ms=3,lw=.5,label='/cmd_vel');ax.set_xlim(10,13);ax.set_ylabel(label);ax.legend();ax.grid(alpha=.3)
axes[-1].set_xlabel('Bag time (s)');fig.suptitle('Final velocity contains immediate raw values and a 20Hz stream');fig.savefig(OUT/'command_streams.png',dpi=160);plt.close(fig)
