"""制作本地原始数据/固件快照，并逐文件校验压缩包内容；清单可提交Git。"""
import hashlib
import json
import zipfile
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
DEST=ROOT/'_probe/archives/2026-10-29-single-wheel'
DEST.mkdir(parents=True,exist_ok=True)
experiments=[
    ('can_capture','2026-10-29-single-wheel'),
    ('can_capture_run2','2026-10-29-single-wheel-run2'),
    ('can_capture_long8s','2026-10-29-single-wheel-long8s'),
    ('can_capture_ki004','2026-10-29-single-wheel-ki004'),
    ('can_capture_ki004_motor3','2026-10-29-single-wheel-ki004-motor3'),
]

def digest(stream):
    h=hashlib.sha256()
    for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
    return h.hexdigest()

def file_digest(path):
    with path.open('rb') as stream:return digest(stream)

def make_archive(name,files):
    target=DEST/(name+'.zip')
    # 清单存在时只校验既有包，避免覆盖已归档快照。
    if not target.exists():
        with zipfile.ZipFile(target,'x',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as archive:
            for path,member in files:archive.write(path,member)
    members=[]
    with zipfile.ZipFile(target) as archive:
        assert set(archive.namelist())=={member for _,member in files}
        for path,member in files:
            expected=file_digest(path)
            with archive.open(member) as stream:actual=digest(stream)
            assert expected==actual,(path,member,'归档内容与源文件不一致，禁止覆盖')
            members.append({'source':path.relative_to(ROOT).as_posix(),'member':member,
                            'bytes':path.stat().st_size,'sha256':actual})
    return {'archive':target.relative_to(ROOT).as_posix(),'archive_bytes':target.stat().st_size,
            'archive_sha256':file_digest(target),'verified_members':members}

result={'session_date':'2026-10-29','raw_archive_policy':'原始压缩包保留本地_probe，不提交Git；清单、报告与分析脚本提交Git',
        'archives':[]}
names=('capture.csv','capture.json','capture.py','analyze.py','analyze_complete.py','export.py',
       'export_analysis.py','analysis.json','summary.json')
for folder,report in experiments:
    source=ROOT/'_probe'/folder
    assert (source/'capture.csv').exists() and (source/'capture.json').exists()
    record=make_archive(folder,[(source/name,f'{folder}/{name}') for name in names if (source/name).is_file()])
    meta=json.loads((source/'capture.json').read_text())
    assert meta['frames']==sum(meta['counts'].values())
    record.update({'report':f'docs/experiments/{report}/README.md','frames':meta['frames'],
                   'telemetry_frames':meta['counts'].get('0x1f2',0)})
    result['archives'].append(record)
    print('VERIFIED',folder,meta['frames'],'frames',record['archive_bytes'],'compressed bytes',flush=True)

firmware_paths=[
 'cboard_chassis/application/chassis_speed_test.h',
 'cboard_chassis/application/chassis_bringup.h',
 'cboard_chassis/application/chassis_bringup.c',
 'cboard_chassis/application/chassis_bringup_map.c',
 'cboard_chassis/application/chassis_bringup_map.h',
 'cboard_chassis/MDK-ARM/chassis_task.uvprojx',
 'cboard_chassis/MDK-ARM/chassis_task/chassis_task.hex',
 'cboard_chassis/MDK-ARM/chassis_task/chassis_task.axf',
 'cboard_chassis/MDK-ARM/build-ki004.log',
]
files=[(ROOT/p,p) for p in firmware_paths if (ROOT/p).is_file()]
assert any(p.endswith('.hex') for _,p in files)
result['firmware_snapshot']=make_archive('firmware-ki004',files)
result['total_frames']=sum(a['frames'] for a in result['archives'])
result['total_telemetry_frames']=sum(a['telemetry_frames'] for a in result['archives'])
(HERE/'manifest.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print('VERIFIED_TOTAL',result['total_frames'],'frames',result['total_telemetry_frames'],'telemetry frames')
print('LOCAL_ARCHIVES',DEST)
