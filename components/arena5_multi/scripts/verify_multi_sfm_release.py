#!/usr/bin/env python3
"""Bind multi-SFM release claims to explicit, hashed validation artifacts."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

ROOT=Path(__file__).resolve().parents[1]
REQUIRED=('offline','cpu_interactions','faults','six_runtime_30min','six_platform_runtime',
          'six_navigation','six_clearing','one_interaction','six_interaction','legacy_six')

def source_digest(root=ROOT):
    names=subprocess.check_output(['git','-C',str(root),'ls-files','-z','src','scripts','config'],text=True).split('\0')
    digest=hashlib.sha256()
    for name in sorted(n for n in names if n):
        digest.update(name.encode()+b'\0');digest.update((root/name).read_bytes());digest.update(b'\0')
    return digest.hexdigest()

def verify(manifest_path):
    manifest=json.loads(manifest_path.read_text());failures=[]
    if manifest.get('source_sha256')!=source_digest():failures.append('validation source differs from current tracked code')
    reports=manifest.get('reports',{})
    for name in REQUIRED:
        if name not in reports:failures.append(f'missing report: {name}');continue
        record=reports[name];path=(ROOT/record['path']).resolve()
        if not path.is_relative_to(ROOT) or not path.is_file():failures.append(f'invalid report path: {name}');continue
        if hashlib.sha256(path.read_bytes()).hexdigest()!=record['sha256']:failures.append(f'report hash mismatch: {name}')
        data=json.loads(path.read_text())
        if data.get('passed') is not True:failures.append(f'failed report: {name}')
        if name=='six_runtime_30min' and data.get('duration_wall_s',0)<1800:failures.append('continuous run shorter than 30 minutes')
        if name=='cpu_interactions' and len(data.get('trials',[]))!=120:failures.append('CPU interaction matrix incomplete')
        if name=='six_navigation' and data.get('rounds_completed',0)!=10:failures.append('navigation rounds incomplete')
    if manifest.get('passed') is not True:failures.append('acceptance is not marked passed')
    return {'passed':not failures,'failures':failures,'source_sha256':source_digest()}

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--manifest',default=str(ROOT/'evidence/multi_sfm/acceptance.json'));ap.add_argument('--source-digest',action='store_true');args=ap.parse_args()
    if args.source_digest:print(source_digest());return 0
    try:result=verify(Path(args.manifest))
    except (OSError,ValueError,KeyError) as error:result={'passed':False,'failures':[str(error)]}
    print(json.dumps(result,indent=2));return 0 if result['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
