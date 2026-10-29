"""从已保存的只接收CAN记录分析控制时序、角度/轮速一致性和降速补偿。

运行：bundled-python analyze_dynamics.py；不接触硬件，不修改固件。
"""
import csv
import json
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
SOURCE = ROOT / '_probe/can_capture_long8s/capture.csv'
summary = json.loads((HERE / 'summary.json').read_text())
raw = {i: [] for i in range(1, 5)}
commands, telemetry = [], []
base = None
for row in csv.DictReader(SOURCE.open()):
    timestamp = int(row['device_us']) / 1e6
    if base is None:
        base = timestamp
    t = timestamp - base
    b = bytes.fromhex(row['data_hex'])
    if len(b) != 8:
        continue
    cid = int(row['can_id'], 16)
    signed = lambda j: int.from_bytes(b[j:j+2], 'big', signed=True)
    if cid == 0x200:
        commands.append([t, *[signed(j) for j in (0, 2, 4, 6)]])
    elif 0x201 <= cid <= 0x204:
        raw[cid-0x200].append([t, int.from_bytes(b[:2], 'big'), signed(2), signed(4)])
    elif cid == 0x1f2:
        telemetry.append([t, b[0], b[1], signed(2), signed(4), signed(6)])
commands = np.asarray(commands)
telemetry = np.asarray(telemetry)
raw = {i: np.asarray(rows) for i, rows in raw.items()}
assert all(np.all(np.diff(rows[:,0]) > 0) for rows in [commands, *raw.values()])

def interval(a):
    v = np.diff(a[:,0]) * 1000
    return {'median_ms': float(np.median(v)), 'max_ms': float(max(v)),
            'p99_ms': float(np.quantile(v,.99)), 'gaps_over_4ms': int(sum(v>4))}

result = {'source': str(SOURCE.relative_to(ROOT)), 'command_interval': interval(commands),
          'feedback_intervals': {i: interval(a) for i,a in raw.items()},
          'max_abs_command': int(abs(commands[:,1:]).max()),
          'command_saturation_samples': int(sum(np.any(abs(commands[:,1:])>=1500,axis=1))),
          'commands_with_multiple_nonzero_motors': int(sum(np.count_nonzero(commands[:,1:],axis=1)>1)),
          'segments': []}
plots = []
for run in summary['runs']:
    motor_id = run['id']
    m = raw[motor_id]
    # 数据角度应在0..8191，且模8192展开后与报告rpm进行一致性校验。
    assert 0 <= m[:,1].min() and m[:,1].max() <= 8191
    delta = (np.diff(m[:,1]) + 4096) % 8192 - 4096
    unwrapped = np.r_[m[0,1], m[0,1] + np.cumsum(delta)]
    for seg in run['segments']:
        target = seg['target']
        if not target:
            continue
        sign = 1 if target>0 else -1
        begin, end = seg['start_s'], seg['end_s']
        grid = np.arange(begin+1, end-.2, .002)
        speed = np.interp(grid,m[:,0],m[:,2])
        angle = np.interp(grid,m[:,0],unwrapped)
        # 20ms中心差分；速度反馈同样做20ms平均后比较，避免把平滑差异误作测量误差。
        encoder_speed = (np.interp(grid+.01,m[:,0],unwrapped)-np.interp(grid-.01,m[:,0],unwrapped))*60/(8192*.02)
        averaged_speed = np.mean([np.interp(grid+d,m[:,0],m[:,2]) for d in np.arange(-.009,.010,.002)],axis=0)
        cmd_index = np.searchsorted(commands[:,0],grid,side='right')-1
        command = commands[cmd_index,motor_id]
        measured_current = np.interp(grid,m[:,0],m[:,3])
        late = grid>=end-2.2
        detrended = speed[late]-np.polyval(np.polyfit(grid[late],speed[late],1),grid[late])
        power = abs(np.fft.rfft(detrended*np.hanning(len(detrended))))**2
        frequencies = np.fft.rfftfreq(len(detrended),.002)
        eligible = (frequencies>=2)&(frequencies<=100)
        peak_hz = float(frequencies[eligible][np.argmax(power[eligible])])
        phase = angle[late]*2*np.pi/8192
        # 用角度锁定的正余弦回归检查是否与转子每转的固定相位有关。
        harmonics = []
        for order in range(1,13):
            design = np.column_stack([np.ones(len(phase)),grid[late]-grid[late].mean(),np.sin(order*phase),np.cos(order*phase)])
            coef = np.linalg.lstsq(design,speed[late],rcond=None)[0]
            residual = speed[late]-design@coef
            harmonics.append({'order': order, 'amplitude_rpm': float(np.hypot(*coef[-2:])),
                              'explained_fraction_vs_linear_trend': float(1-np.sum(residual**2)/np.sum(detrended**2))})
        tm = telemetry[(telemetry[:,1]==motor_id)&(telemetry[:,2]==1)&(telemetry[:,0]>=begin)&(telemetry[:,0]<end)&(telemetry[:,3]==target)]
        inferred_integral = tm[:,5]-2*(target-tm[:,4])
        integral_seconds = []
        for second in range(8):
            selected = (tm[:,0]>=begin+second)&(tm[:,0]<begin+second+1)
            integral_seconds.append(float(np.mean(inferred_integral[selected])))
        frequency_seconds=[]
        for second in range(1,7):
            choose=(grid>=begin+second)&(grid<begin+second+1)
            wave=speed[choose]; wave=wave-np.polyval(np.polyfit(grid[choose],wave,1),grid[choose])
            f=np.fft.rfftfreq(len(wave),.002); p=abs(np.fft.rfft(wave*np.hanning(len(wave))))**2
            band=(f>=30)&(f<=120)
            frequency_seconds.append({'second':second,'mean_abs_rpm':float(abs(np.mean(speed[choose]))),
                                      'peak_30to120_hz':float(f[band][np.argmax(p[band])])})
        crossed=np.flatnonzero(sign*inferred_integral>=0)
        events=[]
        low = np.flatnonzero(sign*speed<500)
        groups = np.split(low,np.where(np.diff(low)>.08/.002)[0]+1) if len(low) else []
        for group in groups:
            if not len(group):
                continue
            k = int(group[np.argmin(sign*speed[group])])
            center = grid[k]
            before = (grid>=center-.1)&(grid<center-.04)
            after = (grid>=center)&(grid<center+.02)
            if not before.any() or not after.any():
                continue
            recovery = np.flatnonzero((grid>center)&(sign*speed>=570))
            events.append({'time_in_step_s': float(center-begin), 'min_oriented_rpm': float(sign*speed[k]),
                           'command_before':float(np.mean(sign*command[before])),
                           'command_next20ms_mean':float(np.mean(sign*command[after])),
                           'command_next20ms_peak':float(max(sign*command[after])),
                           'feedback_current_before':float(np.mean(sign*measured_current[before])),
                           'feedback_current_next20ms_mean':float(np.mean(sign*measured_current[after])),
                           'encoder20ms_rpm_at_min':float(sign*encoder_speed[k]),
                           'recovery_to570_ms':float((grid[recovery[0]]-center)*1000) if len(recovery) else None})
        events.sort(key=lambda e:e['min_oriented_rpm'])
        record={'run':run['run'],'id':motor_id,'target':target,
                'late_window_s':[float(grid[late][0]-begin),float(grid[-1]-begin)],
                'dominant_frequency_hz':peak_hz,'mean_rotor_hz':float(abs(np.mean(speed[late]))/60),
                'angle_locked_harmonics':harmonics,
                'encoder20ms_vs_reported20ms_rmse_rpm':float(np.sqrt(np.mean((encoder_speed-averaged_speed)**2))),
                'encoder20ms_vs_reported20ms_corr':float(np.corrcoef(encoder_speed,averaged_speed)[0,1]),
                'late_command_mean':float(np.mean(command[late])), 'late_command_sd':float(np.std(command[late])),
                'late_feedback_current_mean':float(np.mean(measured_current[late])),
                'late_feedback_current_sd':float(np.std(measured_current[late])),
                'inferred_integral_second_means':integral_seconds,
                'inferred_integral_first':float(inferred_integral[0]),
                'integral_first_same_sign_s':float(tm[crossed[0],0]-begin) if len(crossed) else None,
                'frequency_seconds':frequency_seconds,
                'inferred_integral_peak_abs':float(max(abs(inferred_integral))),
                'drops_below500_after1s':len(events),'deepest_events':events[:3]}
        result['segments'].append(record)
        plots.append((record,grid-begin,speed,encoder_speed,command,measured_current))
        print(json.dumps(record,ensure_ascii=False),flush=True)

(HERE/'dynamics-summary.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print('TIMING',json.dumps({k:v for k,v in result.items() if k!='segments'}),flush=True)

from reportlab.graphics.shapes import Drawing, String
from reportlab.graphics.charts.lineplots import LinePlot
from reportlab.lib import colors
from reportlab.graphics import renderPDF
import pypdfium2 as pdfium

def add_plot(d,x,y,series,title,ylabel):
    p=LinePlot();p.x=x;p.y=y;p.width=415;p.height=190
    p.data=[[(float(t),float(v)) for t,v in zip(tx,values)] for tx,values in series]
    for j,color in enumerate((colors.blue,colors.green,colors.orange)):
        if j<len(series):p.lines[j].strokeColor=color;p.lines[j].strokeWidth=.8
    d.add(p);d.add(String(x,y+205,title,fontSize=11));d.add(String(x,y-24,ylabel,fontSize=9))

d=Drawing(1020,940)
d.add(String(45,910,'Speed drops and current response | left: rpm; right: protocol current units',fontSize=15))
for index,motor_id in enumerate((1,2,4)):
    rec,t,s,e,u,actual=next(p for p in plots if p[0]['id']==motor_id and p[0]['target']==600)
    late_events=[ev for ev in rec['deepest_events'] if ev['time_in_step_s']>=3]
    if motor_id==1:
        center=6.5
    elif late_events:
        center=late_events[0]['time_in_step_s']
    elif rec['deepest_events']:
        center=rec['deepest_events'][0]['time_in_step_s']
    else:
        center=6.5
    use=(t>=center-.15)&(t<=center+.25)
    tx=(t[use]-center)*1000;y=660-index*285
    add_plot(d,50,y,[(tx,s[use]),(tx,e[use])],f'Motor {motor_id}: rpm blue, encoder 20ms green','Relative time (ms)')
    add_plot(d,550,y,[(tx,u[use]),(tx,actual[use])],f'Motor {motor_id}: command blue, feedback current green','Relative time (ms)')
pdf=pdfium.PdfDocument(renderPDF.drawToString(d))
pdf[0].render(scale=1.3).to_pil().save(HERE/'drop-response.png')
print('VERIFIED: decoded commands, feedback and 10 speed steps; plots saved')
