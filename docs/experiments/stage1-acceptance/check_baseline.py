import csv,json,sys
from pathlib import Path
p=Path(sys.argv[1]); counts={}; first=None;last=None;prev={};gaps={};rpm={i:0 for i in range(1,5)};nz=0;badstates=0;statuses={}; malformed=0
for r in csv.DictReader(p.open()):
 t=int(r['device_us']); cid=int(r['can_id'],16);b=bytes.fromhex(r['data_hex'])
 if first is None:first=t
 last=t;counts[hex(cid)]=counts.get(hex(cid),0)+1
 if cid in prev:gaps[hex(cid)]=max(gaps.get(hex(cid),0),t-prev[cid])
 prev[cid]=t
 if len(b)!=8:malformed+=1;continue
 s=lambda j:int.from_bytes(b[j:j+2],'big',signed=True)
 if cid==0x200:nz+=int(any(s(j)!=0 for j in (0,2,4,6)))
 elif 0x201<=cid<=0x204:rpm[cid-0x200]=max(rpm[cid-0x200],abs(s(2)))
 elif cid==0x1f2:badstates+=int(b[0]!=0 or b[1]!=0 or any(s(j)!=0 for j in (2,4,6)))
 elif cid==0x1f1:
  k=f'online={b[0]:02x},mode={b[5]}';statuses[k]=statuses.get(k,0)+1
result={'duration_s':(last-first)/1e6 if first is not None else 0,'frames':sum(counts.values()),'counts':counts,'max_gap_ms':{k:v/1000 for k,v in gaps.items()},'peak_abs_rpm':rpm,'nonzero_command_frames':nz,'nonidle_diagnostic_frames':badstates,'statuses':statuses,'non8byte_frames':malformed}
result['communication_baseline_ok']=result['duration_s']>=59.5 and all(counts.get(hex(0x200+i),0)>50000 for i in range(1,5)) and nz==0 and badstates==0 and set(statuses)=={'online=0f,mode=3'}
p.with_name('baseline_quality.json').write_text(json.dumps(result,indent=2),encoding='utf8')
print(json.dumps(result,indent=2))
