import hashlib,json,zipfile
from pathlib import Path
out=Path(__file__).resolve().parent;cap=Path('E:/RM/_probe/speed400_capture/run1-1793268106993');snap=Path('E:/RM/_probe/speed400-1793267528728');dest=Path('E:/RM/_probe/archives/2026-10-29-speed400');dest.mkdir(parents=True,exist_ok=True);manifest=[]
for label,files in [('capture-run1',list(cap.iterdir())),('firmware-snapshots',[p for p in snap.iterdir() if p.is_file() and p.suffix in ('.hex','.axf','.h','.c','.json','.log','.txt')])]:
 zipname=dest/(label+'.zip');members=[]
 with zipfile.ZipFile(zipname,'x',zipfile.ZIP_DEFLATED) as z:
  for p in files:
   if not p.is_file():continue
   name=label+'/'+p.name;digest=hashlib.sha256(p.read_bytes()).hexdigest();z.write(p,name);members.append({'name':name,'bytes':p.stat().st_size,'sha256':digest})
 with zipfile.ZipFile(zipname) as z:
  for m in members:assert hashlib.sha256(z.read(m['name'])).hexdigest()==m['sha256']
 manifest.append({'path':str(zipname),'sha256':hashlib.sha256(zipname.read_bytes()).hexdigest(),'bytes':zipname.stat().st_size,'members':members})
(out/'archive-manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf8');print('2 archives verified; '+str(sum(m['bytes'] for m in manifest))+' bytes')
