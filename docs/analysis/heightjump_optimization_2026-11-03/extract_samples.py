#!/usr/bin/env python3
"""Read-only extraction of displayed rollout points for query reuse analysis."""
from pathlib import Path
import sqlite3,csv,bisect
from rclpy.serialization import deserialize_message
from visualization_msgs.msg import MarkerArray
from grid_map_msgs.msg import GridMap
root=Path(__file__).resolve().parents[3]
bag=root/'nav_experiment_toolkit/bags/nav_elevation/nav_elevation_2026-11-02_19-50-00'
con=sqlite3.connect(f'file:{next(bag.glob("*.db3"))}?mode=ro',uri=True)
topics={n:i for i,n in con.execute('select id,name from topics')}
t0=con.execute('select min(timestamp) from messages').fetchone()[0]
rows=[]
for k,seconds in enumerate([1,8,12,20,26,31]):
 wall=t0+int(seconds*1e9)
 t,b=con.execute('select timestamp,data from messages where topic_id=? and timestamp>=? order by timestamp limit 1',(topics['/trajectories'],wall)).fetchone()
 mb=con.execute('select data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',(topics['/elevation_mapping_node/elevation_map'],t)).fetchone()[0]
 m=deserialize_message(mb,GridMap);markers=deserialize_message(b,MarkerArray)
 count=0
 for marker in markers.markers:
  if marker.ns!='Candidate Trajectories':continue
  assert marker.header.frame_id==m.header.frame_id=='map'
  p=marker.pose.position;i=m.info
  rows.append([k,(t-t0)/1e9,i.pose.position.x,i.pose.position.y,i.resolution,i.length_x,i.length_y,m.outer_start_index,m.inner_start_index,p.x,p.y]);count+=1
 print('frame',k,'time',(t-t0)/1e9,'points',count)
out=Path(__file__).resolve().parent/'displayed_rollout_samples.csv'
with out.open('w') as f:
 w=csv.writer(f);w.writerow(['frame','bag_time_s','center_x','center_y','resolution','length_x','length_y','start_x','start_y','point_x','point_y']);w.writerows(rows)
