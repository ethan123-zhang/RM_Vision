import csv
import json
import statistics as st
from pathlib import Path

root = Path(r'E:\RM')
src = root / '_probe/can_capture_ki004_motor3/capture.csv'
out = root / 'docs/experiments/2026-10-29-single-wheel-ki004-motor3'
out.mkdir(parents=True, exist_ok=True)
telemetry, counts = [], {}
motor = {i: {'n': 0, 'peak_rpm': 0, 'temp_min': 255, 'temp_max': 0} for i in range(1, 5)}
base = None
for row in csv.DictReader(src.open()):
    stamp = int(row['device_us']) / 1e6
    if base is None:
        base = stamp
    t = stamp - base
    cid = int(row['can_id'], 16)
    counts[row['can_id']] = counts.get(row['can_id'], 0) + 1
    b = bytes.fromhex(row['data_hex'])
    if len(b) != 8:
        continue
    if cid == 0x1f2:
        telemetry.append([t, b[0], b[1], *[int.from_bytes(b[j:j+2], 'big', signed=True) for j in (2, 4, 6)]])
    elif 0x201 <= cid <= 0x204:
        m = motor[cid - 0x200]
        m['n'] += 1
        m['peak_rpm'] = max(m['peak_rpm'], abs(int.from_bytes(b[2:4], 'big', signed=True)))
        m['temp_min'] = min(m['temp_min'], b[6])
        m['temp_max'] = max(m['temp_max'], b[6])

runs, active = [], []
for r in telemetry:
    if r[2] == 1:
        if active and active[-1][1] != r[1]:
            runs.append((active, r[0])); active = []
        active.append(r)
    elif active:
        runs.append((active, r[0])); active = []
if active:
    runs.append((active, None))

def stats(rows, target):
    values = [r[4] for r in rows]
    mean = st.mean(values)
    return {'n': len(rows), 'mean_rpm': round(mean, 2), 'error_rpm': round(mean-target, 2),
            'error_pct': round(abs(mean-target)/abs(target)*100, 2) if target else None,
            'sd_rpm': round(st.pstdev(values), 2), 'min_rpm': min(values), 'max_rpm': max(values),
            'mean_current': round(st.mean(r[5] for r in rows), 2)}

result = {'duration_s': round(t, 3), 'counts': counts, 'motor_raw': motor,
          'telemetry_n': len(telemetry), 'fault_samples': sum(r[2] in (2,3) for r in telemetry),
          'telemetry_interval_median_s': st.median(b[0]-a[0] for a,b in zip(telemetry,telemetry[1:])),
          'telemetry_interval_max_s': max(b[0]-a[0] for a,b in zip(telemetry,telemetry[1:])), 'runs': []}
for index, (run, end) in enumerate(runs, 1):
    segments = []
    for r in run:
        if not segments or segments[-1][0][3] != r[3]:
            segments.append([])
        segments[-1].append(r)
    ends = [s[0][0] for s in segments[1:]] + [end]
    durations = [e-s[0][0] if e is not None else None for s,e in zip(segments,ends)]
    targets = [s[0][3] for s in segments]
    complete = targets == [600,0,-600,0] and all(d is not None and abs(d-w)<.08 for d,w in zip(durations,[8,1,8,1]))
    record = {'run': index, 'id': run[0][1], 'start_s': round(run[0][0],3), 'complete': complete, 'segments': []}
    for seg, end_s, duration in zip(segments, ends, durations):
        target = seg[0][3]
        entry = {'target': target, 'start_s': seg[0][0], 'end_s': end_s, 'duration_s': round(duration,3) if duration is not None else None,
                 'peak_abs_current': max(abs(r[5]) for r in seg),
                 'saturation_samples': sum(abs(r[5])>=1500 for r in seg),
                 'last_half_s': stats([r for r in seg if r[0]>=seg[-1][0]-.5], target)}
        if target:
            entry['last_2s'] = stats([r for r in seg if r[0]>=seg[-1][0]-2],target)
            entry['seconds'] = [stats([r for r in seg if second<=r[0]-seg[0][0]<second+1],target) for second in range(8)]
            last_outside = max((i for i,r in enumerate(seg) if abs(r[4]-target)>30), default=-1)
            candidate = last_outside+1
            entry['settling_5pct_s'] = round(seg[candidate][0]-seg[0][0],3) if candidate<len(seg) and seg[-1][0]-seg[candidate][0]>=1 else None
        record['segments'].append(entry)
    result['runs'].append(record)

windows = [(run['id'], seg, []) for run in result['runs'] for seg in run['segments'] if seg['target']]
for row in csv.DictReader(src.open()):
    cid=int(row['can_id'],16)
    if not 0x201<=cid<=0x204:
        continue
    when=int(row['device_us'])/1e6-base
    b=bytes.fromhex(row['data_hex'])
    if len(b)!=8:
        continue
    for motor_id,seg,values in windows:
        # 遥测边界约有26ms不确定性，提前0.2秒结束原始反馈统计，避开切零制动。
        if cid-0x200==motor_id and seg['end_s']-2.2<=when<seg['end_s']-.2:
            values.append(int.from_bytes(b[2:4],'big',signed=True))
for motor_id,seg,values in windows:
    seg['raw_late_2s_before_transition_margin']={'n':len(values),'mean_rpm':round(st.mean(values),2),
                        'sd_rpm':round(st.pstdev(values),2),'min_rpm':min(values),'max_rpm':max(values)}

with (out/'telemetry.csv').open('w',newline='') as f:
    w=csv.writer(f); w.writerow(['elapsed_s','motor_id','state','target_rotor_rpm','measured_rotor_rpm','current_command']); w.writerows(telemetry)
(out/'summary.json').write_text(json.dumps(result,indent=2))
print('SUMMARY', {k:v for k,v in result.items() if k not in ('runs','counts')})
for run in result['runs']:
    print('RUN',run['run'],'ID',run['id'],'complete',run['complete'])
    for seg in run['segments']:
        print('SEG',seg['target'],seg['duration_s'],'tail0.5',seg['last_half_s'],'tail2',seg.get('last_2s'), 'peak_current',seg['peak_abs_current'],'saturation',seg['saturation_samples'])
        if seg['target']:
            print('SECONDS',[s['mean_rpm'] for s in seg['seconds']],'settling5%',seg['settling_5pct_s'])
            print('RAW_LATE_2S_MARGIN_0.2S',seg['raw_late_2s_before_transition_margin'])

# 绘制3号完整补测序列。
from reportlab.graphics.shapes import Drawing, String
from reportlab.graphics.charts.lineplots import LinePlot
from reportlab.lib import colors
from reportlab.graphics import renderPDF
import pypdfium2 as pdfium
d = Drawing(980, 370)
d.add(String(45,345,'8-second speed steps | target (red), measured (blue)',fontSize=17))
for index, ((run,end), summary) in enumerate(zip(runs,result['runs'])):
    x=55+(index%2)*480; y=100-(index//2)*290
    p=LinePlot();p.x=x;p.y=y;p.width=410;p.height=210
    p.data=[[(r[0]-run[0][0],r[col]) for r in run] for col in (3,4)]
    p.xValueAxis.valueMin=0;p.xValueAxis.valueMax=18;p.xValueAxis.valueStep=2
    p.yValueAxis.valueMin=-800;p.yValueAxis.valueMax=800;p.yValueAxis.valueStep=200
    p.lines[0].strokeColor=colors.red;p.lines[1].strokeColor=colors.blue
    d.add(p);d.add(String(x,y+227,f"Motor {run[0][1]} | run {index+1}",fontSize=12))
pdf=pdfium.PdfDocument(renderPDF.drawToString(d))
pdf[0].render(scale=1.4).to_pil().save(out/'speed-traces.png')
assert all(r['complete'] for r in result['runs']), '存在不完整测试，请单独检查'
assert {r['id'] for r in result['runs']} == {3}
assert len(telemetry)==counts['0x1f2']
print('VERIFIED motor 3 full cycle; telemetry export and plot complete')
