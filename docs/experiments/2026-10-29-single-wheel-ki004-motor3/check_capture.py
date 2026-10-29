import csv,json
from pathlib import Path
import numpy as np

here=Path(__file__).resolve().parent
root=here.parents[2]
src=root/'_probe/can_capture_ki004_motor3/capture.csv'
commands=[];raw={i:[] for i in range(1,5)};telemetry=[];base=None
for row in csv.DictReader(src.open()):
    stamp=int(row['device_us'])/1e6
    if base is None:base=stamp
    t=stamp-base;b=bytes.fromhex(row['data_hex']);cid=int(row['can_id'],16)
    if len(b)!=8:continue
    signed=lambda j:int.from_bytes(b[j:j+2],'big',signed=True)
    if cid==0x200:commands.append([t,*[signed(j) for j in (0,2,4,6)]])
    elif 0x201<=cid<=0x204:raw[cid-0x200].append([t,signed(2)])
    elif cid==0x1f2:telemetry.append([t,b[0],b[1],signed(2),signed(4),signed(6)])
commands=np.asarray(commands);raw={i:np.asarray(a) for i,a in raw.items()};telemetry=np.asarray(telemetry)
active=telemetry[telemetry[:,2]==1];start=active[0,0]
end=next(r[0] for r in telemetry if r[0]>active[-1,0] and r[2]==0)
late=commands[commands[:,0]>=end]
def timing(a):
    d=np.diff(a[:,0])*1000
    k=int(np.argmax(d))
    return {'max_ms':float(max(d)),'median_ms':float(np.median(d)),'gaps_over4ms':int(sum(d>4)),
            'largest_gap_start_s':float(a[k,0]),'largest_gap_end_s':float(a[k+1,0])}
result={'duration_s':t,'active_id':sorted(set(active[:,1].astype(int).tolist())),
        'active_start_s':float(start),'active_end_s':float(end),'active_duration_s':float(end-start),
        'command_timing':timing(commands),'command_peak':float(abs(commands[:,1:]).max()),
        'other_motor_nonzero_command_frames':int(sum(np.any(commands[:,[1,2,4]]!=0,axis=1))),
        'post_exit_nonzero_command_frames':int(sum(np.any(late[:,1:]!=0,axis=1))),
        'saturation_frames':int(sum(np.any(abs(commands[:,1:])>=1500,axis=1))),
        'feedback_timing':{i:timing(a) for i,a in raw.items()},'raw_motor_peaks':{}}
result['active_command_timing']=timing(commands[(commands[:,0]>=start)&(commands[:,0]<end)])
result['active_motor3_feedback_timing']=timing(raw[3][(raw[3][:,0]>=start)&(raw[3][:,0]<end)])
for i,a in raw.items():
    at=int(np.argmax(abs(a[:,1])))
    sel=a[(a[:,0]>=start)&(a[:,0]<end)]
    result['raw_motor_peaks'][i]={'peak_rpm':float(a[at,1]),'at_s':float(a[at,0]),'during_test_peak_abs_rpm':float(abs(sel[:,1]).max())}
assert result['active_id']==[3]
assert result['other_motor_nonzero_command_frames']==0
assert result['post_exit_nonzero_command_frames']==0
assert result['saturation_frames']==0
assert abs(result['active_duration_s']-18)<.08
assert sum(json.loads((here/'summary.json').read_text())['counts'].values())==json.loads((src.parent/'capture.json').read_text())['frames']
(here/'quality.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result,indent=2))
