import numpy as np,pandas as pd,json
from pathlib import Path
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path('/home/nvidia/MPPI_3dnavi/docs/analysis/teleop_flat_2026-10-19_20-47-37')
a=np.load('/tmp/teleop_arrays.npz');p=np.load('/tmp/teleop_cloud_samples.npz');f=pd.read_csv(O/'sampled_ground.csv');T=p['T'];times=p['times'];rows=[]
x,y=np.indices((70,70));ox=3.45-.1*x;oy=3.45-.1*y
for k,r in f.iterrows():
 t=r.header_t;i=int(np.argmin(abs(a['mt']-t)));q=p[f'p{k}'];w=q@T[:3,:3].T+T[:3,3];b=a['bp'][i];e=a['E'][i];ctr=a['centers'][i]
 ground=r.plane_a*(w[:,0]-b[0])+r.plane_b*(w[:,1]-b[1])+r.plane_z_at_robot
 floor=abs(w[:,2]-ground)<.04
 near=np.linalg.norm(w[:,:2]-b[:2],axis=1)<2.5
 lidar=(a['bp'][i]+ __import__('scipy').spatial.transform.Rotation.from_euler('xyz',a['rpy'][i],degrees=True).apply([.535,0,.31]))
 rr=np.linalg.norm(w-lidar,axis=1);wr=np.linalg.norm(q,axis=1)
 rec={'t':r.t,'origin_to_lidar_m':float(np.linalg.norm(lidar-T[:3,3])),'noise_ratio_ground_median':float(np.median((wr[near&floor]/rr[near&floor])**2)),'actual_ground_range_median':float(np.median(rr[near&floor])),'fixed_origin_ground_range_median':float(np.median(wr[near&floor]))}
 for tag,c in [('published',ctr),('quantized',np.round(ctr/.1)*.1)]:
  ij=np.floor((c+3.5-w[:,:2])/.1).astype(int);inside=(ij>=0).all(1)&(ij<70).all(1)
  sel=inside&near&floor;ids=ij[sel,0]*70+ij[sel,1];pp=w[sel,2];errs=[];flags=[]
  for cell in np.unique(ids):
   ii,jj=divmod(int(cell),70);z=e[ii,jj]
   if not np.isfinite(z):continue
   err=z-np.median(pp[ids==cell]);errs.append(err)
   if err>.3 and np.sum(ids==cell)>=3:
    cellxy=c+3.45-.1*np.array([ii,jj]);dxy=np.linalg.norm(w[:,:2]-cellxy,axis=1)
    # Any high return within 20cm of cell accounts for adjacent wall/overhang support.
    support=np.any((dxy<.2)&(w[:,2]>z-.2))
    if not support:flags.append((ii,jj))
  errs=np.array(errs);rec[tag+'_cell_abs_error_p95']=float(np.percentile(abs(errs),95));rec[tag+'_high_floor_cells']=int((errs>.3).sum());rec[tag+'_unsupported_high_cells']=len(flags)
 rows.append(rec)
 if k==127:
  fig,axs=plt.subplots(1,3,figsize=(18,6));cx,cy=ctr
  lim=[cx-3.5,cx+3.5,cy-3.5,cy+3.5]
  zz=e.T[::-1,::-1]
  im=axs[0].imshow(zz,origin='lower',extent=lim,vmin=-.5,vmax=2,cmap='turbo');axs[0].set_title('Recorded elevation')
  sel=(abs(w[:,0]-cx)<3.5)&(abs(w[:,1]-cy)<3.5)
  axs[1].scatter(w[sel,0],w[sel,1],c=w[sel,2],vmin=-.5,vmax=2,cmap='turbo',s=1);axs[1].set_title('Current cloud in map frame')
  xx=ctr[0]+ox;yy=ctr[1]+oy
  pe=r.plane_a*(xx-b[0])+r.plane_b*(yy-b[1])+r.plane_z_at_robot
  axs[2].imshow((e-pe).T[::-1,::-1],origin='lower',extent=lim,vmin=-.3,vmax=.3,cmap='coolwarm');axs[2].scatter(w[floor,0],w[floor,1],s=.4,c='k');axs[2].set_title('Map minus ground; black=current ground')
  for ax in axs:
   ax.set_xlim(lim[:2]);ax.set_ylim(lim[2:]);ax.plot(*b[:2],'g*',ms=10);ax.set_aspect('equal');ax.set_xlabel('Map X (m)');ax.set_ylabel('Map Y (m)')
  fig.suptitle(f't={r.t:.2f}s (20:49:45 local), positive residuals also include real walls/objects')
  fig.colorbar(im,ax=axs[:2],label='Height (m)',shrink=.7);fig.savefig(O/'detail_127s.png',dpi=150,bbox_inches='tight');plt.close(fig)
g=pd.DataFrame(rows);g.to_csv(O/'registration_checks.csv',index=False)
print(g.iloc[[0,30,60,90,110,127,140]].to_string(index=False))
print('Error ratio:',g.noise_ratio_ground_median.quantile([0,.5,.95,1]).to_dict())
print('unsupported published max',g.published_unsupported_high_cells.max(),'quantized max',g.quantized_unsupported_high_cells.max())
print('UNSUPPORTED',g.sort_values('quantized_unsupported_high_cells',ascending=False).head(12)[['t','published_high_floor_cells','quantized_high_floor_cells','quantized_unsupported_high_cells']].to_string(index=False))
print('published vs quantized P95:',g[['published_cell_abs_error_p95','quantized_cell_abs_error_p95']].quantile([.5,.95,1]))
# Local map alignment implied by current source. No assertion of measured internal center.
print('center quantization delta stats',np.percentile(np.linalg.norm(a['centers']-np.round(a['centers']/.1)*.1,axis=1),[50,95,100]))
