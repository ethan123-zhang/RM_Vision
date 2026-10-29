"""Ki=0.004采集：保留每次尝试，按完整性选择可比较速度段。"""
import csv
import json
from pathlib import Path
import numpy as np
from reportlab.graphics.shapes import Drawing, String
from reportlab.graphics.charts.lineplots import LinePlot
from reportlab.lib import colors
from reportlab.graphics import renderPDF
import pypdfium2 as pdfium

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
src=ROOT/'_probe/can_capture_ki004/capture.csv'
baseline=json.loads((HERE.parent/'2026-10-29-single-wheel-long8s/summary.json').read_text())
counts={};telemetry=[];commands=[];raw={i:[] for i in range(1,5)};base=None
for row in csv.DictReader(src.open()):
    t=int(row['device_us'])/1e6
    if base is None:base=t
    t-=base
    cid=int(row['can_id'],16);counts[row['can_id']]=counts.get(row['can_id'],0)+1
    b=bytes.fromhex(row['data_hex'])
    if len(b)!=8:continue
    signed=lambda j:int.from_bytes(b[j:j+2],'big',signed=True)
    if cid==0x1f2:telemetry.append([t,b[0],b[1],signed(2),signed(4),signed(6)])
    elif cid==0x200:commands.append([t,*[signed(j) for j in (0,2,4,6)]])
    elif 0x201<=cid<=0x204:raw[cid-0x200].append([t,int.from_bytes(b[:2],'big'),signed(2),signed(4),b[6]])
commands=np.asarray(commands);raw={i:np.asarray(v) for i,v in raw.items()}

def stats(rows,target):
    a=np.asarray(rows);mean=float(np.mean(a[:,4]))
    return {'n':len(rows),'mean_rpm':mean,'error_rpm':mean-target,
            'error_pct':abs(mean-target)/abs(target)*100 if target else None,
            'sd_rpm':float(np.std(a[:,4])),'min_rpm':float(min(a[:,4])),
            'max_rpm':float(max(a[:,4])),'mean_current':float(np.mean(a[:,5]))}

groups=[];active=[]
for r in telemetry:
    if r[2]==1:
        if active and active[-1][1]!=r[1]:groups.append((active,r));active=[]
        active.append(r)
    elif active:groups.append((active,r));active=[]
if active:groups.append((active,None))
all_runs=[];run_rows={}
for index,(rows,following) in enumerate(groups,1):
    segments=[]
    for r in rows:
        if not segments or segments[-1][0][3]!=r[3]:segments.append([])
        segments[-1].append(r)
    ends=[s[0][0] for s in segments[1:]]+[following[0] if following else None]
    durations=[end-s[0][0] if end is not None else None for s,end in zip(segments,ends)]
    targets=[s[0][3] for s in segments]
    complete=targets==[600,0,-600,0] and all(d is not None and abs(d-w)<.08 for d,w in zip(durations,[8,1,8,1]))
    pair_complete=targets==[600,0,-600,0] and all(d is not None and abs(d-w)<.08 for d,w in zip(durations[:3],[8,1,8]))
    record={'run':index,'id':rows[0][1],'start_s':rows[0][0],
            'duration_s':following[0]-rows[0][0] if following else None,
            'complete':complete,'speed_pair_complete':pair_complete,
            'end_state':following[2] if following else None,'segments':[]}
    for segment,end,duration in zip(segments,ends,durations):
        target=segment[0][3]
        item={'target':target,'start_s':segment[0][0],'end_s':end,'duration_s':duration,
              'last_half_s':stats([r for r in segment if r[0]>=segment[-1][0]-.5],target),
              'peak_abs_current':max(abs(r[5]) for r in segment)}
        if target and duration is not None and abs(duration-8)<.08:
            item['last_2s']=stats([r for r in segment if r[0]>=segment[-1][0]-2],target)
            item['seconds']=[stats([r for r in segment if k<=r[0]-segment[0][0]<k+1],target) for k in range(8)]
        record['segments'].append(item)
    all_runs.append(record);run_rows[index]=rows
selected=[]
for i in range(1,5):
    candidates=[r for r in all_runs if r['id']==i and r['complete'] and r['end_state']==0]
    if not candidates:candidates=[r for r in all_runs if r['id']==i and r['speed_pair_complete'] and r['end_state']==0]
    if candidates:selected.append(candidates[-1])
assert {r['id'] for r in selected}=={1,2,3,4}
assert all(r['end_state']==0 and r['speed_pair_complete'] for r in selected)

def intervals(a):
    dt=np.diff(a[:,0])*1000
    index=int(np.argmax(dt))
    return {'median_ms':float(np.median(dt)),'max_ms':float(max(dt)),
            'largest_gap_start_s':float(a[index,0]),'largest_gap_end_s':float(a[index+1,0]),
            'gaps_over4ms':int(sum(dt>4))}

result={'duration_s':t,'counts':counts,'telemetry_n':len(telemetry),
        'ki':.004,'user_reported_change':'更换2号CAN通信线，换线时刻未精确标记',
        'fault_samples':sum(r[2]==2 for r in telemetry),'all_runs':all_runs,'runs':selected,
        'raw_feedback_intervals':{i:intervals(a) for i,a in raw.items()},
        'command_interval':intervals(commands),'command_peak':float(abs(commands[:,1:]).max()),
        'command_saturation_frames':int(sum(np.any(abs(commands[:,1:])>=1500,axis=1))),
        'comparison':[]}
for r in all_runs:
    subset=commands[(commands[:,0]>=r['start_s'])&(commands[:,0]<r['start_s']+r['duration_s'])]
    r['command_saturation_frames']=int(sum(abs(subset[:,r['id']])>=1500))
for r in selected:
    b=[x for x in baseline['runs'] if x['id']==r['id']][-1]
    command_subset=commands[(commands[:,0]>=r['start_s'])&(commands[:,0]<r['start_s']+r['duration_s'])]
    r['raw_command_peak']=float(abs(command_subset[:,r['id']]).max())
    r['command_interval']=intervals(command_subset)
    feedback_subset=raw[r['id']][(raw[r['id']][:,0]>=r['start_s'])&(raw[r['id']][:,0]<r['start_s']+r['duration_s'])]
    r['feedback_interval']=intervals(feedback_subset)
    r['peak_abs_rotor_rpm']=float(abs(feedback_subset[:,2]).max())
    r['temperature_range']=[int(min(feedback_subset[:,4])),int(max(feedback_subset[:,4]))]
    assert r['command_saturation_frames']==0
    for step in (600,-600):
        old=next(x for x in b['segments'] if x['target']==step)
        new=next(x for x in r['segments'] if x['target']==step)
        result['comparison'].append({'id':r['id'],'target':step,
            'baseline_last2_mean':old['last_2s']['mean_rpm'],'new_last2_mean':new['last_2s']['mean_rpm'],
            'baseline_last2_sd':old['last_2s']['sd_rpm'],'new_last2_sd':new['last_2s']['sd_rpm'],
            'baseline_seconds':[x['mean_rpm'] for x in old['seconds']],
            'new_seconds':[x['mean_rpm'] for x in new['seconds']]})

with (HERE/'telemetry.csv').open('w',newline='') as f:
    w=csv.writer(f);w.writerow(['elapsed_s','motor_id','state','target_rotor_rpm','measured_rotor_rpm','current_command']);w.writerows(telemetry)
(HERE/'summary.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
assert len(telemetry)==counts['0x1f2']
assert sum(counts.values())==json.loads((src.parent/'capture.json').read_text())['frames']
d=Drawing(980,660);d.add(String(45,635,'Ki 0.004 | selected full speed steps | target red, feedback blue',fontSize=15))
for index,r in enumerate(selected):
    rows=run_rows[r['run']];p=LinePlot();p.x=55+(index%2)*480;p.y=355-(index//2)*290;p.width=410;p.height=210
    p.data=[[(x[0]-rows[0][0],x[col]) for x in rows] for col in (3,4)]
    p.xValueAxis.valueMin=0;p.xValueAxis.valueMax=18;p.xValueAxis.valueStep=2
    p.yValueAxis.valueMin=-800;p.yValueAxis.valueMax=800;p.yValueAxis.valueStep=200
    p.lines[0].strokeColor=colors.red;p.lines[1].strokeColor=colors.blue
    d.add(p);d.add(String(p.x,p.y+225,f"Motor {r['id']} run {r['run']} | full cycle: {r['complete']}",fontSize=11))
pdf=pdfium.PdfDocument(renderPDF.drawToString(d));pdf[0].render(scale=1.3).to_pil().save(HERE/'speed-traces.png')
print('ALL_RUNS',[(r['run'],r['id'],round(r['start_s'],3),r['complete'],r['speed_pair_complete'],r['end_state']) for r in all_runs])
print('SELECTED',[(r['run'],r['id'],r['complete'],r['raw_command_peak']) for r in selected])
print('TIMING',result['raw_feedback_intervals'],'COMMAND_PEAK',result['command_peak'],'SATURATION',result['command_saturation_frames'])
for r in result['comparison']:print('COMPARE',r)
print('VERIFIED: 4 motors have complete speed pairs; final zero incomplete for motor 3; no silent inclusion of short speed steps')
