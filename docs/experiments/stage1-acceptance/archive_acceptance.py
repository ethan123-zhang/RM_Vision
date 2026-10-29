import hashlib,json,zipfile
from pathlib import Path
root=Path(__file__).resolve().parent;data=Path('E:/RM/_probe/stage1_acceptance_20261029');out=Path('E:/RM/_probe/archives/2026-10-29-stage1-acceptance');out.mkdir(parents=True,exist_ok=True)
folders=['baseline-1793266272305','repeat-1793266358266','repeat2-1793266687532'];manifest=[]
for folder in folders:
 files=[p for p in (data/folder).iterdir() if p.is_file()];dest=out/(folder+'.zip');members=[]
 with zipfile.ZipFile(dest,'x',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
  for p in files:
   name=folder+'/'+p.name;digest=hashlib.sha256(p.read_bytes()).hexdigest();z.write(p,name);members.append({'name':name,'bytes':p.stat().st_size,'sha256':digest})
 with zipfile.ZipFile(dest) as z:
  for m in members:assert hashlib.sha256(z.read(m['name'])).hexdigest()==m['sha256']
 manifest.append({'path':str(dest),'bytes':dest.stat().st_size,'sha256':hashlib.sha256(dest.read_bytes()).hexdigest(),'members':members})
(root/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf8')
print('3 archives verified by SHA-256 readback;',sum(m['bytes'] for m in manifest),'compressed bytes')
