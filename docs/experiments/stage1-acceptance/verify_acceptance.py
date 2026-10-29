import csv,json,math,sys
from pathlib import Path
root=Path(__file__).resolve().parent
for name in ('run1','run2'):
 s=json.loads((root/name/'summary.json').read_text());src=Path(s['input']);meta=json.loads(src.with_suffix('.json').read_text());base=None;counts={};slots=[]
 for run in s['runs']:
  assert run['complete'] and run['targets']==[600,0,-600,0]
  for phase in run['phases']:
   if phase['target']:slots.append({'id':run['motor'],'lo':phase['last_diagnostic_s']-2,'hi':phase['last_diagnostic_s'],'n':0,'total':0,'square':0,'phase':phase})
 for row in csv.DictReader(src.open()):
  stamp=int(row['device_us'])/1e6
  if base is None:base=stamp
  t=stamp-base;cid=int(row['can_id'],16);b=bytes.fromhex(row['data_hex']);counts[hex(cid)]=counts.get(hex(cid),0)+1
  if 0x201<=cid<=0x204 and len(b)==8:
   speed=int.from_bytes(b[2:4],'big',signed=True)
   for w in slots:
    if cid-0x200==w['id'] and w['lo']<=t<=w['hi']:w['n']+=1;w['total']+=speed;w['square']+=speed*speed
 assert counts==meta['counts']==s['counts']
 assert sum(counts.values())==meta['frames']
 for w in slots:
  mean=w['total']/w['n'];sd=math.sqrt(w['square']/w['n']-mean*mean)
  assert abs(mean-w['phase']['tail_mean_rpm'])<1e-8
  assert abs(sd-w['phase']['tail_std_rpm'])<1e-8
 print(name,'raw counts and independent mean/std verified;',meta['frames'],'frames')
assert all(sum(json.loads((root/n/'summary.json').read_text())['complete_runs_per_motor'][str(i)] for n in ('run1','run2'))==2 for i in range(1,5))
print('8 complete cycles verified; 16 raw speed tails independently checked')
