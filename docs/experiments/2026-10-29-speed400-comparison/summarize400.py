import csv,json,sys
from pathlib import Path
import numpy as np
# Explicit input/output so repeat runs never overwrite run1.
# usage: python summarize400.py <capture.csv> <out_dir> <out_prefix> [reset_recovery_pc_s] [expected_motors]
if len(sys.argv)<4:
    sys.exit('usage: summarize400.py <capture.csv> <out_dir> <prefix> [reset_recovery_pc_s] [expected_motors]')
src=Path(sys.argv[1]);out=Path(sys.argv[2]);prefix=sys.argv[3]
expected=[int(m) for m in sys.argv[5].split(',')] if len(sys.argv)>5 else [1,2,4]
reset_recovery=None if len(sys.argv)<=4 or sys.argv[4].lower()=='none' else float(sys.argv[4])
raw={i:[] for i in range(1,5)};commands=[];diag=[];base=None;counts={}
for row in csv.DictReader(src.open()):
 t=int(row['device_us'])/1e6;b=bytes.fromhex(row['data_hex']);cid=int(row['can_id'],16)
 if base is None:base=t
 t-=base;counts[hex(cid)]=counts.get(hex(cid),0)+1
 if len(b)!=8:continue
 s=lambda j:int.from_bytes(b[j:j+2],'big',signed=True)
 if 0x201<=cid<=0x204:raw[cid-0x200].append([t,s(2)])
 elif cid==0x200:commands.append([t,*[s(j) for j in (0,2,4,6)]])
 elif cid==0x1f2:diag.append([t,b[0],b[1],s(2),s(4),s(6)])
raw={i:np.array(v) for i,v in raw.items()};commands=np.array(commands);sessions=[];active=[]
for d in diag:
 if d[2]==1:active.append(d)
 elif active:sessions.append((active,d));active=[]
if active:sessions.append((active,None))
runs=[]
for rows,exitrow in sessions:
 groups=[]
 for d in rows:
  if not groups or groups[-1][-1][3]!=d[3]:groups.append([])
  groups[-1].append(d)
 mid=int(rows[0][1]);ends=[g[0][0] for g in groups[1:]]+[exitrow[0] if exitrow else None];durations=[e-g[0][0] if e else None for g,e in zip(groups,ends)]
 run={'motor':mid,'start_s':rows[0][0],'end_s':exitrow[0] if exitrow else None,'targets':[g[0][3] for g in groups],'durations_s':durations,'phases':[]}
 run['complete']=run['targets']==[400,0,-400,0] and all(d is not None and abs(d-w)<.08 for d,w in zip(durations,[8,1,8,1])) and exitrow is not None and exitrow[2]==0
 for g in groups:
  target=g[0][3];hi=g[-1][0];tail=raw[mid][(raw[mid][:,0]>=hi-(2 if target else .5))&(raw[mid][:,0]<=hi),1]
  item={'target':target,'first_diagnostic_s':g[0][0],'last_diagnostic_s':hi,'tail_samples':len(tail),'tail_mean_rpm':float(np.mean(tail)),'tail_std_rpm':float(np.std(tail)),'tail_min_rpm':float(min(tail)),'tail_max_rpm':float(max(tail)),'tail_peak_abs_rpm':float(max(abs(tail)))}
  if target:item['mean_error_pct']=float(abs(np.mean(tail)-target)/abs(target)*100);item['error_95pct_rpm']=np.percentile(tail-target,[2.5,97.5]).tolist()
  run['phases'].append(item)
 a=commands[(commands[:,0]>=rows[0][0])&(commands[:,0]<=rows[-1][0])];others=[i for i in range(1,5) if i!=mid]
 run['command_peak']=float(abs(a[:,1:]).max());run['saturation_frames']=int(np.any(abs(a[:,1:])>=1500,axis=1).sum());run['other_motor_nonzero_frames']=int(np.any(a[:,others]!=0,axis=1).sum());run['command_max_gap_ms']=float(np.diff(a[:,0]).max()*1000)
 if exitrow:
  a=commands[(commands[:,0]>=exitrow[0]+.03)&(commands[:,0]<=exitrow[0]+1)]
  run['release_samples']=len(a);run['release_nonzero_frames']=int(np.any(a[:,1:]!=0,axis=1).sum())
 runs.append(run)
meta=json.loads(src.with_suffix('.json').read_text());assert counts==meta['counts'];assert sum(counts.values())==meta['frames'];assert [r['motor'] for r in runs]==expected;assert all(r['complete'] for r in runs)
result={'input':str(src),'duration_s':t,'frames':meta['frames'],'diagnostic_samples':len(diag),'fault_diagnostic_samples':sum(d[2]>=2 for d in diag),'runs':runs,'purpose':'400rpm diagnostic comparison only; no acceptance conclusion','boundary':'tail ends at last diagnostic sample; about 26ms uncertainty','reset_recovery_pc_s':reset_recovery,'expected_motors':expected}
(out/f'{prefix}-summary.json').write_text(json.dumps(result,indent=2),encoding='utf8')
with (out/f'{prefix}-telemetry.csv').open('w',newline='') as f:w=csv.writer(f);w.writerow(['device_elapsed_s','motor','status','target_rpm','measured_rpm','current_command']);w.writerows(diag)
print(json.dumps(result,indent=2))
