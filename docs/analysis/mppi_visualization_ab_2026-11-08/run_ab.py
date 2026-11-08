import collections,datetime,json,math,os,statistics,subprocess,time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters,SetParameters
from rcl_interfaces.msg import Parameter,ParameterValue,Log
from nav2_msgs.action import NavigateToPose,ComputePathToPose
from nav_msgs.msg import Path as NavPath
from geometry_msgs.msg import Twist,PoseStamped
from visualization_msgs.msg import MarkerArray
OUT=Path('/tmp/mppi_ab_profile');poses=json.loads((OUT/'poses.json').read_text());target=poses['goal_candidates'][0]
rclpy.init();n=rclpy.create_node('mppi_visualization_ab_test');active=None;state=None;results=[];subs=[]
def spin_for(s):
 end=time.monotonic()+s
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
def wait(f,timeout=8):
 end=time.monotonic()+timeout
 while not f.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
 if not f.done():raise RuntimeError('ROS operation timeout')
 return f.result()
getc=n.create_client(GetParameters,'/controller_server/get_parameters');setc=n.create_client(SetParameters,'/controller_server/set_parameters')
assert getc.wait_for_service(timeout_sec=12) and setc.wait_for_service(timeout_sec=5)
def get_params(names):
 req=GetParameters.Request();req.names=names;return wait(getc.call_async(req)).values
def set_visual(value):
 req=SetParameters.Request();req.parameters=[Parameter(name='FollowPath.visualize',value=ParameterValue(type=1,bool_value=value))]
 response=wait(setc.call_async(req));assert all(r.successful for r in response.results),str(response)
 assert get_params(['FollowPath.visualize'])[0].bool_value==value

def callback(topic):
 def cb(msg):
  if state is None:return
  state['times'][topic].append(time.monotonic())
  if isinstance(msg,Twist):state['nonzero'][topic]+=int(abs(msg.linear.x)+abs(msg.angular.z)>1e-5)
  if isinstance(msg,bytes):state['bytes'][topic]+=len(msg)
 return cb
for topic,typ,raw in [('/command/cmd_twist',Twist,False),('/command/cmd_twist_smoothed',Twist,False),('/cmd_vel',Twist,False),('/transformed_global_plan',NavPath,False),('/trajectories',MarkerArray,True)]:
 subs.append(n.create_subscription(typ,topic,callback(topic),qos_profile_sensor_data,raw=raw))
def logcb(msg):
 if state is not None and msg.level>=30:state['warnings'][(msg.name,msg.msg)]+=1
subs.append(n.create_subscription(Log,'/rosout',logcb,qos_profile_sensor_data))
nav=ActionClient(n,NavigateToPose,'navigate_to_pose');planner=ActionClient(n,ComputePathToPose,'compute_path_to_pose')
assert nav.wait_for_server(timeout_sec=10) and planner.wait_for_server(timeout_sec=5)
def target_pose():
 p=PoseStamped();p.header.frame_id='map';p.header.stamp=n.get_clock().now().to_msg();p.pose.position.x=target['x'];p.pose.position.y=target['y'];p.pose.orientation.z=math.sin(target['yaw']/2);p.pose.orientation.w=math.cos(target['yaw']/2);return p
pg=ComputePathToPose.Goal();pg.goal=target_pose();pg.use_start=False
ph=wait(planner.send_goal_async(pg));assert ph.accepted
pr=wait(ph.get_result_async());assert pr.status==4 and len(pr.result.path.poses)>10, str(pr)
(OUT/'planned_path.json').write_text(json.dumps([[p.pose.position.x,p.pose.position.y] for p in pr.result.path.poses]))
print('PLANNED_PATH',len(pr.result.path.poses),'poses',flush=True)
original=get_params(['FollowPath.visualize'])[0].bool_value
parameters=get_params(['FollowPath.batch_size','FollowPath.time_steps','FollowPath.model_dt','FollowPath.HeightJumpCritic.enabled'])
print('PARAMETERS',[(p.integer_value,p.double_value,p.bool_value) for p in parameters],flush=True)

def proc_snapshot():
 out={}
 for p in Path('/proc').glob('[0-9]*/stat'):
  try:
   text=p.read_text();end=text.rfind(')');comm=text[text.find('(')+1:end];v=text[end+2:].split()
   if comm in ('component_conta','rviz2') or p.parent.name==str(os.getpid()):out[int(p.parent.name)]={'comm':comm,'ticks':int(v[11])+int(v[12])}
  except (OSError,ValueError,IndexError):pass
 return out

def percentile(x,p):
 x=sorted(x);return x[int((len(x)-1)*p)] if x else None

def stage_stats(stage,start,end):
 rows=[]
 if (OUT/'timings.csv').exists():
  for line in (OUT/'timings.csv').read_text().splitlines():
   kind,stamp,ms,ok=line.split(',')
   if kind==stage and start<=float(stamp)<=end:rows.append((float(stamp),float(ms),int(ok)))
 durations=[x[1] for x in rows if x[2]];intervals=[1000*(b[0]-a[0]) for a,b in zip(rows,rows[1:])]
 return {'count':len(rows),'failed':sum(not x[2] for x in rows),'mean_ms':statistics.mean(durations) if durations else None,'p50_ms':percentile(durations,.5),'p95_ms':percentile(durations,.95),'p99_ms':percentile(durations,.99),'max_ms':max(durations) if durations else None,'over_50ms':sum(v>50 for v in durations),'cycle_p95_ms':percentile(intervals,.95),'cycle_max_ms':max(intervals) if intervals else None,'cycle_over_60ms':sum(v>60 for v in intervals)}

try:
 for index,show in enumerate([True,False,False,True],1):
  set_visual(show);spin_for(.5)
  goal=NavigateToPose.Goal();goal.pose=target_pose();active=wait(nav.send_goal_async(goal));assert active.accepted
  result_future=active.get_result_async();spin_for(2)
  if result_future.done():raise RuntimeError('Goal ended during warmup: '+str(result_future.result().status))
  start=time.monotonic();wall_start=time.time();procs=proc_snapshot();cpu_start=time.process_time()
  state={'times':collections.defaultdict(list),'bytes':collections.Counter(),'nonzero':collections.Counter(),'warnings':collections.Counter()}
  spin_for(8)
  end=time.monotonic();wall_end=time.time();procs_end=proc_snapshot();observer_cpu=100*(time.process_time()-cpu_start)/(end-start)
  observed=state;state=None
  assert not result_future.done(),'Goal ended during measurement'
  cancellation=wait(active.cancel_goal_async());assert cancellation.goals_canceling
  status=wait(result_future).status;active=None
  trial={'trial':index,'visualize':show,'start_monotonic':start,'end_monotonic':end,'wall_start':wall_start,'wall_end':wall_end,'duration_s':end-start,'goal_result_status':status,'observer_cpu_one_core':observer_cpu,'controller':stage_stats('controller',start,end),'optimizer':stage_stats('optimizer',start,end),'topics':{},'warnings':[{'node':k[0],'text':k[1],'count':v} for k,v in observed['warnings'].items()],'process_cpu_one_core':{}}
  for topic in ['/command/cmd_twist','/command/cmd_twist_smoothed','/cmd_vel','/transformed_global_plan','/trajectories']:
   ts=observed['times'][topic];dt=[(b-a)*1000 for a,b in zip(ts,ts[1:])]
   trial['topics'][topic]={'count':len(ts),'hz':(len(ts)-1)/(ts[-1]-ts[0]) if len(ts)>1 else 0,'interval_p95_ms':percentile(dt,.95),'interval_max_ms':max(dt) if dt else None,'nonzero':observed['nonzero'][topic],'mib_s':observed['bytes'][topic]/(end-start)/2**20}
  for pid,a in procs.items():
   if pid in procs_end:trial['process_cpu_one_core'][pid]={'comm':a['comm'],'cpu':100*(procs_end[pid]['ticks']-a['ticks'])/os.sysconf('SC_CLK_TCK')/(end-start)}
  assert trial['controller']['count']>100,'Timing hook is not recording computeVelocityCommands'
  assert trial['optimizer']['count']>100,'Timing hook is not recording evalControl'
  assert (trial['topics']['/trajectories']['count']>100) if show else (trial['topics']['/trajectories']['count']==0),'Visualization did not match setting'
  results.append(trial);(OUT/'results.json').write_text(json.dumps({'poses':poses,'parameters':{'batch_size':parameters[0].integer_value,'time_steps':parameters[1].integer_value,'model_dt':parameters[2].double_value,'height_critic_enabled':parameters[3].bool_value},'trials':results},indent=2))
  print('TRIAL',index,'visualize',show,'controller',trial['controller'],'optimizer',trial['optimizer'],'cmd_hz',trial['topics']['/cmd_vel']['hz'],flush=True)
  spin_for(1)
finally:
 state=None
 if active is not None:
  try:wait(active.cancel_goal_async(),timeout=3)
  except Exception as error:print('CANCEL_ERROR',error,flush=True)
 try:set_visual(original);print('RESTORED_VISUALIZE',original,flush=True)
 except Exception as error:print('RESTORE_ERROR',error,flush=True)
 n.destroy_node();rclpy.shutdown()
