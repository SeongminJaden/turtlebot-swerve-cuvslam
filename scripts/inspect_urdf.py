"""Team-independent URDF inventory and structural checks (no physics claim)."""
import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

p = argparse.ArgumentParser()
p.add_argument('--urdf', required=True, type=Path)
p.add_argument('--package', action='append', default=[], metavar='NAME=PATH')
p.add_argument('--output', type=Path, default=Path('checks/team_urdf.json'))
a = p.parse_args()
packages = {}
for mapping in a.package:
    if '=' not in mapping:
        p.error('--package must be NAME=PATH')
    name, folder = mapping.split('=', 1)
    packages[name] = Path(folder).resolve()
robot = ET.parse(a.urdf).getroot()
links = [x.get('name') for x in robot.findall('link')]
errors = []
if len(links) != len(set(links)):
    errors.append('Duplicate link names')
joints, parents = [], {}
for j in robot.findall('joint'):
    parent, child = j.find('parent'), j.find('child')
    if parent is None or child is None:
        errors.append(f'{j.get("name")}: missing parent or child')
        continue
    parent, child = parent.get('link'), child.get('link')
    if parent not in links or child not in links:
        errors.append(f'{j.get("name")}: unknown link')
    if child in parents:
        errors.append(f'{child}: multiple parents')
    parents[child] = parent
    joints.append({'name': j.get('name'), 'type': j.get('type'), 'parent': parent,
                   'child': child, **{key: dict(j.find(key).attrib) if j.find(key) is not None else None
                                     for key in ('origin', 'axis', 'limit', 'mimic')}})
roots = sorted(set(links)-set(parents))
if len(roots) != 1:
    errors.append(f'Expected one root, found {len(roots)}')
for link in links:
    seen = set()
    while link in parents:
        if link in seen:
            errors.append(f'Cycle at {link}')
            break
        seen.add(link)
        link = parents[link]
names = [j['name'] for j in joints]
if len(set(names)) != len(names):
    errors.append('Duplicate joint names')
meshes = []
for m in robot.findall('.//mesh'):
    uri = m.get('filename', '')
    if uri.startswith('package://'):
        package, _, relative = uri[10:].partition('/')
        target = packages[package]/relative if package in packages else None
    elif uri.startswith('file://'):
        target = Path(uri[7:])
    else:
        target = a.urdf.parent/uri
    exists = target is not None and target.is_file()
    meshes.append({'uri': uri, 'resolved': str(target.resolve()) if target else None, 'exists': exists})
    if not exists:
        errors.append(f'Unresolved mesh: {uri}')
report = {'robot': robot.get('name'), 'source': str(a.urdf.resolve()),
          'scope': 'URDF names, graph and mesh paths only; USD equivalence, axes, inertia and dynamics need separate checks',
          'status': 'PASS_STRUCTURE_ONLY' if not errors else 'FAIL',
          'roots': roots, 'links': links, 'joints': joints, 'meshes': meshes, 'errors': errors}
a.output.parent.mkdir(parents=True, exist_ok=True)
a.output.write_text(json.dumps(report, ensure_ascii=False, indent=2))
print(json.dumps({'status': report['status'], 'links':len(links), 'joints':len(joints), 'errors':errors},indent=2))
sys.exit(bool(errors))
