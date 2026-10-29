import csv,json,sys
from pathlib import Path
import numpy as np
src=Path(sys.argv[1]);out=Path(sys.argv[2]);out.mkdir(parents=True,exist_ok=True)
raw={i:[] for i in range(1,5)};cmd=[];diag=[];status=[];base=None;counts={}
with src.open() as f:
 for r in csv.DictReader(f):
  try:t=int(r['device_us'])/1e6;b=bytes.fromhex(r['data_hex']);cid=int(r['can_id'],16)
  except (ValueError,TypeError,KeyError):continue
  if base is None:base=t
  t-=base;counts[hex(cid)]=counts.get(hex(cid),0)+1
  if len(b)!=8:continue
  s=lambda j:int.from_bytes(b[j:j+2],'big',signed=True)
  if 0x201<=cid<=0x204:raw[cid-0x200].append([t,s(2),s(4),b[6]])
  elif cid==0x200:cmd.append([t,*[s(j) for j in (0,2,4,6)]])
  elif cid==0x1f2:diag.append([t,b[0],b[1],s(2),s(4),s(6)])
  elif cid==0x1f1:status.append([t,b[0],b[5]])
raw={i:np.array(v,dtype=float) for i,v in raw.items()};cmd=np.array(cmd,dtype=float);diag=np.array(diag,dtype=float)
def window(a,lo,hi):return a[(a[:,0]>=lo)&(a[:,0]<=hi)]
def gap(a):return float(np.max(np.diff(a[:,0]))*1000) if len(a)>1 else None
sessions=[];active=[]
for d in diag:
 if d[2]==1 and d[1] in raw:
  if active and d[1]!=active[-1][1]:sessions.append((active,None));active=[]
  active.append(d)
 elif active:sessions.append((active,d));active=[]
if active:sessions.append((active,None))
results=[]
for idx,(rows,exitrow) in enumerate(sessions,1):
 a=np.array(rows);motor=int(a[0,1]);groups=[];start=0
 for k in range(1,len(a)+1):
  if k==len(a) or a[k,3]!=a[start,3]:groups.append(a[start:k]);start=k
 targets=[int(g[0,3]) for g in groups];duration=float(exitrow[0]-a[0,0]) if exitrow is not None else None
 complete=exitrow is not None and int(exitrow[2])==0 and targets==[600,0,-600,0] and abs(duration-18)<=.08
 if complete:
  edges=[float(g[0,0]) for g in groups]+[float(exitrow[0])]
  complete=all(abs((edges[k+1]-edges[k])-expected)<=.08 for k,expected in enumerate([8,1,8,1]))
 item={'sequence':idx,'motor':motor,'start_s':float(a[0,0]),'duration_s':duration,'targets':targets,'complete':complete,'exit_status':int(exitrow[2]) if exitrow is not None else None,'phases':[]}
 for g in groups:
  target=int(g[0,3]);lo=float(g[0,0]);hi=float(g[-1,0]);r=window(raw[motor],lo,hi)
  if len(r)==0:continue
  phase={'target':target,'first_diagnostic_s':lo,'last_diagnostic_s':hi,'raw_samples':len(r),'feedback_max_gap_ms':gap(r)}
  if target:
   tail=window(raw[motor],hi-2,hi);values=tail[:,1];err=values-target
   mean=float(np.mean(values));std=float(np.std(values));overshoot=max(0.,float(np.max(np.sign(target)*r[:,1]))-abs(target))/abs(target)*100
   times=r[:,0];v=r[:,1];cs=np.concatenate(([0.],np.cumsum(v)));left=np.searchsorted(times,times-.25,side='left');den=np.arange(len(v))+1-left;ma=(cs[1:]-cs[left])/den
   eligible=np.flatnonzero(times>=lo+.25);bad=eligible[np.abs(ma[eligible]-target)>30];candidate=(int(bad[-1])+1) if len(bad) else (int(eligible[0]) if len(eligible) else len(v))
   settling=float(times[candidate]-lo) if candidate<len(v) and hi-times[candidate]>=1 else None
   phase.update({'tail_mean_rpm':mean,'mean_error_pct':abs(mean-target)/6,'tail_std_rpm':std,'tail_std_pct':std/6,'tail_min_rpm':float(min(values)),'tail_max_rpm':float(max(values)),'tail_error_95pct_rpm':np.percentile(err,[2.5,97.5]).tolist(),'overshoot_pct':overshoot,'settling_025s_s':settling,'checks':{'mean':abs(mean-target)<=12,'std':std<=30,'overshoot':overshoot<=20,'settling':settling is not None and settling<=3}})
  else:
   tail=window(raw[motor],hi-.5,hi);peak=float(np.max(np.abs(tail[:,1])));phase.update({'tail_half_second_peak_abs_rpm':peak,'checks':{'zero_speed':peak<=5}})
  item['phases'].append(phase)
 activecmd=window(cmd,float(a[0,0]),float(a[-1,0]));others=[j for j in range(1,5) if j!=motor]
 item['command_peak']=float(np.max(np.abs(activecmd[:,1:]))) if len(activecmd) else None
 item['saturated_frames']=int(np.sum(np.any(np.abs(activecmd[:,1:])>=1500,axis=1)))
 item['other_motor_nonzero_command_frames']=int(np.sum(np.any(activecmd[:,others]!=0,axis=1)))
 item['command_max_gap_ms']=gap(activecmd)
 if exitrow is not None:
  nextstart=next((float(x[0][0][0]) for x in sessions[idx:] if x[0][0][0]>exitrow[0]),float(cmd[-1,0]))
  released=window(cmd,float(exitrow[0])+.03,min(float(exitrow[0])+1,nextstart-.05))
  item['release_samples']=len(released);item['release_nonzero_frames']=int(np.sum(np.any(released[:,1:]!=0,axis=1))) if len(released) else None
 item['pass']=bool(complete and len(item['phases'])==4 and all(all(p['checks'].values()) for p in item['phases']) and item.get('release_samples',0)>0 and item['release_nonzero_frames']==0 and item['other_motor_nonzero_command_frames']==0 and item['saturated_frames']==0)
 results.append(item)
summary={'input':str(src),'duration_s':float(cmd[-1,0]) if len(cmd) else None,'counts':counts,'method':'Raw feedback; tail windows end at last diagnostic sample of each target phase, up to one diagnostic period before true phase end. Moving average is trailing 0.25s; no interpolation across dropped samples. Phase and settling timing uncertain by about 26ms.','status_values':sorted(set((int(s[1]),int(s[2])) for s in status)),'fault_diagnostic_samples':int(np.sum(diag[:,2]>=2)),'runs':results,'complete_runs_per_motor':{i:sum(r['complete'] and r['motor']==i for r in results) for i in raw}}
summary['acceptance_pass']=all(summary['complete_runs_per_motor'][i]>=2 for i in raw) and all(r['pass'] for r in results) and summary['fault_diagnostic_samples']==0
(out/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf8')
with (out/'telemetry.csv').open('w',newline='') as f:
 w=csv.writer(f);w.writerow(['time_s','motor','status','target_rpm','measured_rpm','current_command']);w.writerows(diag)
print(json.dumps({'duration_s':summary['duration_s'],'complete_runs_per_motor':summary['complete_runs_per_motor'],'acceptance_pass':summary['acceptance_pass'],'runs':[{'motor':r['motor'],'complete':r['complete'],'pass':r['pass'],'phases':[{'target':p['target'],'mean_error_pct':p.get('mean_error_pct'),'std_rpm':p.get('tail_std_rpm'),'overshoot_pct':p.get('overshoot_pct'),'settling_s':p.get('settling_025s_s'),'zero_peak':p.get('tail_half_second_peak_abs_rpm'),'checks':p['checks']} for p in r['phases']]} for r in results]},indent=2))
