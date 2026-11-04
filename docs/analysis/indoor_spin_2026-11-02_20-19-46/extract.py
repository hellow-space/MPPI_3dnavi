#!/usr/bin/env python3
"""Read-only extraction of the indoor bag; never initializes ROS or publishes."""
import sqlite3, math, json, pickle, csv
from pathlib import Path
from collections import defaultdict
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[2]
BAG=ROOT/'nav_experiment_toolkit/bags/nav_elevation/nav_elevation_2026-11-02_20-19-46'
db=next(BAG.glob('*.db3'))
con=sqlite3.connect(f'file:{db}?mode=ro',uri=True)
topics={r[0]:(r[1],r[2]) for r in con.execute('select id,name,type from topics')}
t0,t1,n=con.execute('select min(timestamp),max(timestamp),count(*) from messages').fetchone()
print('duration', (t1-t0)/1e9, 'messages',n,flush=True)

def stamp(s): return s.sec+s.nanosec*1e-9

def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

def pose(p): return [p.position.x,p.position.y,p.position.z,yaw(p.orientation)]

def summary(a):
 a=np.asarray(a,dtype=float);a=a[np.isfinite(a)]
 return {} if not len(a) else dict(zip(['min','p50','p95','max'],map(float,np.percentile(a,[0,50,95,100]))))
D={'t0':t0,'t1':t1,'bag':str(BAG),'cmd':defaultdict(list),'tf':defaultdict(list),'static':{},'odom':[],'paths':defaultdict(list),'ages':defaultdict(list),'times':defaultdict(list),'status':defaultdict(list),'maps':defaultdict(list),'terrain':[]}
classes={i:get_message(ty) for i,(name,ty) in topics.items() if name!='/trajectories'}
for i,t,raw in con.execute('select topic_id,timestamp,data from messages order by timestamp'):
 name,ty=topics[i]; tr=(t-t0)/1e9
 D['times'][name].append(tr)
 if i not in classes:continue
 m=deserialize_message(raw,classes[i])
 if hasattr(m,'header'):D['ages'][name].append([tr,t/1e9-stamp(m.header.stamp)])
 if name in ['/command/cmd_twist','/cmd_vel']:
  D['cmd'][name].append([tr,m.linear.x,m.linear.y,m.angular.z])
 elif name in ['/tf','/tf_static']:
  for x in m.transforms:
   key=(x.header.frame_id.lstrip('/'),x.child_frame_id.lstrip('/'))
   q=x.transform.rotation;v=x.transform.translation
   v=[v.x,v.y,v.z,yaw(q),q.x,q.y,q.z,q.w]
   if name=='/tf_static':D['static'][key]=v
   else:D['tf'][key].append([tr,stamp(x.header.stamp)-t0/1e9,*v])
 elif name=='/aft_mapped_to_init':
  w=m.twist.twist
  D['odom'].append([tr,stamp(m.header.stamp)-t0/1e9,*pose(m.pose.pose),w.linear.x,w.linear.y,w.linear.z,w.angular.x,w.angular.y,w.angular.z])
 elif ty=='nav_msgs/msg/Path':
  D['paths'][name].append(dict(t=tr,stamp=stamp(m.header.stamp)-t0/1e9,frame=m.header.frame_id,pts=np.array([pose(p.pose) for p in m.poses])))
 elif ty=='action_msgs/msg/GoalStatusArray':
  D['status'][name].append([tr,[(bytes(s.goal_info.goal_id.uuid).hex(),stamp(s.goal_info.stamp)-t0/1e9,s.status) for s in m.status_list]])
 elif ty=='nav_msgs/msg/OccupancyGrid':
  D['maps'][name].append(dict(t=tr,frame=m.header.frame_id,res=m.info.resolution,w=m.info.width,h=m.info.height,origin=pose(m.info.origin),data=np.array(m.data,dtype=np.int8)))
 elif ty=='grid_map_msgs/msg/GridMap':
  D['terrain'].append(dict(t=tr,stamp=stamp(m.header.stamp)-t0/1e9,frame=m.header.frame_id,center=pose(m.info.pose),resolution=m.info.resolution,length=[m.info.length_x,m.info.length_y],outer=m.outer_start_index,inner=m.inner_start_index,layers={layer:np.array(a.data,dtype=np.float32) for layer,a in zip(m.layers,m.data)},layout=[[(d.label,d.size,d.stride) for d in a.layout.dim] for a in m.data]))
for k in ['cmd','tf','ages']:
 for key,val in D[k].items():D[k][key]=np.asarray(val)
D['odom']=np.array(D['odom'])
with open('/tmp/mppi_indoor_spin_201946_data.pkl','wb') as f:pickle.dump(D,f)
S={'duration_s':(t1-t0)/1e9,'count':n,'topics':{},'tf':{},'static':{str(k):v for k,v in D['static'].items()},'status':{'navigate':D['status']['/navigate_to_pose/_action/status'],'follow_last':[[t,s[-1]] for t,s in D['status']['/follow_path/_action/status']]}}
for name,times in D['times'].items():
 a=np.asarray(times);s={'count':len(a),'rate_hz':(len(a)-1)/(a[-1]-a[0]) if len(a)>1 else 0,'gap_s':summary(np.diff(a))}
 if name in D['ages']:s['receive_minus_stamp_s']=summary(D['ages'][name][:,1])
 S['topics'][name]=s
for key,a in D['tf'].items():
 S['tf'][str(key)]={'n':len(a),'age_s':summary(a[:,0]-a[:,1]),'range_xyz_yaw':[(float(a[:,i].min()),float(a[:,i].max())) for i in range(2,6)]}
for name,a in D['cmd'].items():
 S['topics'][name]['vx']=summary(a[:,1]);S['topics'][name]['wz']=summary(a[:,3]);S['topics'][name]['nonzero']=int(np.any(np.abs(a[:,1:])>1e-6,axis=1).sum())
 with open(OUT/(name.split('/')[-1]+'.csv'),'w') as f:
  w=csv.writer(f);w.writerow(['bag_time_s','vx_m_s','vy_m_s','wz_rad_s']);w.writerows(a)
(OUT/'initial_metrics.json').write_text(json.dumps(S,indent=2))
print(json.dumps({k:v for k,v in S.items() if k != 'status'},indent=2),flush=True)
print('odom start/end',D['odom'][[0,-1]],flush=True)
for name,paths in D['paths'].items():
 print('path',name,'n',len(paths),'first',paths[0]['frame'],paths[0]['pts'][[0,-1]],'last',paths[-1]['pts'][[0,-1]],flush=True)
