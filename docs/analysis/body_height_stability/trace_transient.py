#!/usr/bin/env python3
"""Trace all 10 Hz input clouds for a transient mapped height change."""
import sqlite3,json,csv
from pathlib import Path
import numpy as np
from sensor_msgs.msg import PointCloud2
from rclpy.serialization import deserialize_message
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path('/home/nvidia/MPPI_3dnavi');O=R/'docs/analysis/body_height_stability';name='teleop_flat_2026-10-19_20-47-37';B=R/'rosbag_recording/bags/terrain_elevation'/name
C=sqlite3.connect(f'file:{next(B.glob("*.db3"))}?mode=ro',uri=True);start=C.execute('select min(timestamp) from messages').fetchone()[0]/1e9;tid=C.execute("select id from topics where name='/cloud_registered'").fetchone()[0]
a=dict(np.load('/tmp/'+name+'_posture.npz'));T=np.array(json.loads((O/'summary.json').read_text())[name]['map_camera']);xy=np.array([-6.25,-18.05]);rows=[]
for bt,raw in C.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp',(tid,int((start+136.9)*1e9),int((start+142)*1e9))):
 m=deserialize_message(raw,PointCloud2);t=m.header.stamp.sec+m.header.stamp.nanosec/1e9-start;fields={f.name:f.offset for f in m.fields};dt=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[fields[k] for k in ['x','y','z']],'itemsize':m.point_step});q=np.frombuffer(m.data,dtype=dt);p=np.stack([q[k] for k in ['x','y','z']],axis=1);w=p@T[:3,:3].T+T[:3,3];z=w[(abs(w[:,:2]-xy)<.05).all(1),2];i=int(np.argmin(abs(a['mt']-t)));ctr=np.round(a['centers'][i]/.1)*.1;ij=np.floor((ctr+3.5-xy)/.1).astype(int);e=float(a['E'][i][tuple(ij)]);b=float(np.interp(t,a['rt'],a['rb'][:,2]));rows.append({'t':t,'body_z':b,'cloud_z_min':float(z.min()) if len(z) else np.nan,'cloud_z_max':float(z.max()) if len(z) else np.nan,'cloud_points_above_floor':int(np.sum((z>-.2)&(z<2))),'elevation':e})
with (O/'transient_137s.csv').open('w') as f:
 wr=csv.DictWriter(f,fieldnames=list(rows[0]));wr.writeheader();wr.writerows(rows)
fig,axs=plt.subplots(2,1,figsize=(11,7),sharex=True);t=[r['t'] for r in rows]
axs[0].plot(t,[r['body_z'] for r in rows]);axs[0].set_ylabel('Body Z (m)');axs[0].set_title('Fixed cell (-6.25, -18.05): transient elevated returns explain the map rise')
axs[1].plot(t,[r['cloud_z_min'] for r in rows],label='Input cloud minimum Z');axs[1].plot(t,[r['cloud_z_max'] for r in rows],label='Input cloud maximum Z');axs[1].plot(t,[r['elevation'] for r in rows],label='Elevation');axs[1].legend();axs[1].set_ylabel('Ground / return Z (m)');axs[1].set_xlabel('Seconds from bag start')
for ax in axs:ax.grid(alpha=.3)
fig.tight_layout();fig.savefig(O/'transient_137s.png',dpi=150);plt.close(fig)
print('frames',len(rows),'body_z_range_m',np.ptp([r['body_z'] for r in rows]),'map_minmax',[min(r['elevation'] for r in rows),max(r['elevation'] for r in rows)],'input-high-return-frames',sum(r['cloud_points_above_floor']>0 for r in rows))
