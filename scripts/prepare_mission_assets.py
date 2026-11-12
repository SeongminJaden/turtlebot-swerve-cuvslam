"""Verify user-provided archives before extracting to an isolated asset directory."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser()
p.add_argument('--downloads',type=Path,default=Path.home()/'Downloads')
a=p.parse_args()
manifest=json.loads((ROOT/'config/mission/sources.json').read_text())
target=ROOT/'assets/mission'
for name in ('maze.zip','R12_NO_USB_URDF_USD.zip'):
    archive=a.downloads/name
    with archive.open('rb') as stream:
        actual=hashlib.file_digest(stream,'sha256').hexdigest()
    if actual != manifest[name]['sha256']: raise SystemExit(f'Source hash mismatch: {name}')
    with zipfile.ZipFile(archive) as z:
        for member in z.infolist():
            if not (target/member.filename).resolve().is_relative_to(target.resolve()):
                raise SystemExit('Unsafe archive path')
        z.extractall(target)
    print(f'Verified and extracted {name}')
