"""Archive the 400rpm second-round raw capture plus the unchanged 400rpm firmware snapshot.

Exclusive creation only: refuses to overwrite an existing archive directory/ZIP.
Reads every ZIP member back and verifies SHA-256 against the manifest.
"""
import hashlib, json, sys, zipfile
from pathlib import Path

ROOT = Path('E:/RM')
cap = ROOT / '_probe/speed400_capture/run2-1793274040118'
snap = ROOT / '_probe/speed400-1793267528728'
dest = ROOT / '_probe/archives/2026-10-29-speed400-run2'
if dest.exists():
    sys.exit(f'refusing to overwrite existing {dest}')
dest.mkdir(parents=True)

manifest = []
for label, files in [
    ('capture-run2', [p for p in cap.iterdir() if p.is_file()]),
    ('firmware-snapshots', [p for p in snap.iterdir()
                            if p.is_file() and p.suffix in ('.hex', '.axf', '.h', '.c', '.json', '.log', '.txt')]),
]:
    zipname = dest / (label + '.zip')
    members = []
    with zipfile.ZipFile(zipname, 'x', zipfile.ZIP_DEFLATED) as z:
        for p in files:
            name = label + '/' + p.name
            digest = hashlib.sha256(p.read_bytes()).hexdigest()
            z.write(p, name)
            members.append({'name': name, 'bytes': p.stat().st_size, 'sha256': digest})
    with zipfile.ZipFile(zipname) as z:
        for m in members:
            assert hashlib.sha256(z.read(m['name'])).hexdigest() == m['sha256'], m['name']
    manifest.append({'path': str(zipname), 'sha256': hashlib.sha256(zipname.read_bytes()).hexdigest(),
                     'bytes': zipname.stat().st_size, 'members': members})

manifest_path = Path(__file__).resolve().parent / 'archive-manifest-run2.json'
manifest_path.write_text(json.dumps(manifest, indent=2), encoding='utf8')
print('2 archives verified;', sum(m['bytes'] for m in manifest), 'bytes total ->', manifest_path)
