#!/usr/bin/env python3
"""Read-only registered-cloud density audit; recorder data, not raw LiDAR diagnosis."""
import csv
import json
import pickle
from pathlib import Path
import sqlite3

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from scipy.spatial.transform import Rotation

OUT=Path(__file__).resolve().parent
D=pickle.load(open('/tmp/mppi_indoor_spin_201946_data.pkl','rb'))
db=next(Path(D['bag']).glob('*.db3'))
con=sqlite3.connect(f'file:{db}?mode=ro',uri=True)
topic=con.execute("SELECT id FROM topics WHERE name='/cloud_registered'").fetchone()[0]
rows=con.execute('SELECT id,timestamp FROM messages WHERE topic_id=? ORDER BY timestamp',(topic,)).fetchall()
selected=np.unique(np.linspace(0,len(rows)-1,48,dtype=int))
A=D['tf'][('camera_init','aft_mapped')]
B=D['tf'][('map','camera_init')][0]
C=D['static'][('aft_mapped','base_link')]
rb=Rotation.from_quat(B[6:10]);rc=Rotation.from_quat(C[4:8])
range_edges=np.array([0.,.5,1.,2.,3.,5.,10.,np.inf])
counts=[];range_counts=[];height_counts=[];example=None
for index in selected:
    message_id,timestamp=rows[index]
    raw=con.execute('SELECT data FROM messages WHERE id=?',(message_id,)).fetchone()[0]
    m=deserialize_message(bytes(raw),PointCloud2)
    assert m.header.frame_id=='camera_init'
    fields={f.name:f for f in m.fields}
    assert all(fields[n].datatype==7 for n in ['x','y','z'])
    endian='>' if m.is_bigendian else '<'
    dtype=np.dtype({'names':['x','y','z'],'formats':[endian+'f4']*3,
                    'offsets':[fields[n].offset for n in ['x','y','z']],
                    'itemsize':m.point_step})
    structured=np.ndarray((m.height,m.width),dtype=dtype,buffer=m.data,
                          strides=(m.row_step,m.point_step)).reshape(-1)
    pts=np.column_stack([structured[n] for n in ['x','y','z']]).astype(float)
    finite=np.all(np.isfinite(pts),axis=1);pts=pts[finite]
    t=(timestamp-D['t0'])/1e9
    ai=int(np.argmin(abs(A[:,0]-t)));a=A[ai]
    ra=Rotation.from_quat(a[6:10])
    base=rb.apply(ra.apply(C[:3])+a[2:5])+B[2:5]
    heading=(rb*ra*rc).as_euler('ZYX')[0]
    mapped=rb.apply(pts)+B[2:5]
    relative=mapped-base
    # Gravity-preserving local coordinates: yaw only, not body roll/pitch.
    leveled=Rotation.from_euler('z',-heading).apply(relative)
    planar=np.linalg.norm(relative[:,:2],axis=1)
    inside=(np.abs(relative[:,:2])<3.5).all(axis=1)
    low=inside&(relative[:,2]>=-.5)&(relative[:,2]<=1.5)
    floor_band=inside&(np.abs(relative[:,2])<.2)
    occupied=len(np.unique(np.floor((relative[inside,:2]+3.5)/.1).astype(int),axis=0))
    floor_occupied=len(np.unique(np.floor((relative[floor_band,:2]+3.5)/.1).astype(int),axis=0))
    retained=relative[low]
    voxel05=len(np.unique(np.floor(retained/.05).astype(np.int64),axis=0))
    voxel10=len(np.unique(np.floor(retained/.1).astype(np.int64),axis=0))
    row=dict(t=t,points=m.width*m.height,finite_points=int(finite.sum()),
             inside_7m_square=int(inside.sum()),outside_fraction=float(1-inside.mean()),
             inside_above_1_5m=int(np.sum(inside&(relative[:,2]>1.5))),
             inside_below_minus_0_5m=int(np.sum(inside&(relative[:,2]<-.5))),
             candidate_height_band_points=int(low.sum()),
             occupied_10cm_xy_cells=occupied,near_zero_band_points=int(floor_band.sum()),
             near_zero_band_xy_cells=floor_occupied,candidate_voxel_5cm_points=voxel05,
             candidate_voxel_10cm_points=voxel10,
             raw_bytes=len(m.data),nearest_pose_receipt_difference_s=float(a[0]-t))
    counts.append(row)
    range_counts.append(np.histogram(planar,range_edges)[0])
    height_counts.append(np.histogram(relative[:,2],[-np.inf,-.5,.2,1.5,2.5,np.inf])[0])
    if example is None or abs(t-60)<abs(example[0]-60):example=(t,leveled)

def stats(values):
    return dict(zip(['min','p50','p95','max'],map(float,np.percentile(values,[0,50,95,100]))))

metrics=dict(bag=D['bag'],sampled_frames=len(counts),total_registered_frames=len(rows),
    frame_rate_hz=(len(rows)-1)/((rows[-1][1]-rows[0][1])/1e9),
    stats={key:stats([r[key] for r in counts]) for key in counts[0] if key!='t'},
    range_bin_labels=['0-.5m','.5-1m','1-2m','2-3m','3-5m','5-10m','>10m'],
    range_bin_mean_points=np.mean(range_counts,axis=0).tolist(),
    height_bin_labels=['<-.5m','-.5 to .2m','.2 to 1.5m','1.5 to 2.5m','>2.5m'],
    height_bin_mean_points=np.mean(height_counts,axis=0).tolist(),
    caveats=['Cloud after LIO preprocessing, not raw packets; cannot establish packet loss.',
             '7m square is centered at reconstructed base, approximating rolling-map ROI.',
             'Height bands are relative to base, not measured ground; candidate band is illustrative, not a recommended safety limit.',
             'Projected occupied cells include walls and obstacles, not proven ground/free-space coverage.',
             'Single-frame decimation cannot establish scene completeness or thin obstacle detection.'])
(OUT/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
with (OUT/'frames.csv').open('w') as f:
    writer=csv.DictWriter(f,fieldnames=list(counts[0]),lineterminator='\n');writer.writeheader();writer.writerows(counts)

fig,axes=plt.subplots(1,3,figsize=(16,5),constrained_layout=True)
t,p=example
sel=(np.abs(p[:,:2])<4.).all(axis=1)
sc=axes[0].scatter(p[sel,0],p[sel,1],c=p[sel,2],s=1,cmap='viridis',vmin=-.5,vmax=2.)
axes[0].set_aspect('equal');axes[0].set_xlabel('forward (m)');axes[0].set_ylabel('left (m)')
axes[0].set_title(f'Cloud near robot at {t:.1f}s; color=relative height')
fig.colorbar(sc,ax=axes[0],label='height relative to base (m)')
axes[1].scatter(p[sel,0],p[sel,2],s=1,c='tab:blue')
axes[1].set_xlabel('forward (m)');axes[1].set_ylabel('relative height (m)')
axes[1].set_ylim(-1.,4.);axes[1].set_title('Side projection; all lateral points included')
axes[2].bar(metrics['range_bin_labels'],metrics['range_bin_mean_points'])
axes[2].set_ylabel('mean points per frame');axes[2].set_xlabel('horizontal range from robot')
axes[2].set_title('Where the registered points are');axes[2].tick_params(axis='x',rotation=35)
for ax in axes:ax.grid(alpha=.2)
fig.savefig(OUT/'cloud_density.png',dpi=160);plt.close(fig)
print(json.dumps(metrics,indent=2))
