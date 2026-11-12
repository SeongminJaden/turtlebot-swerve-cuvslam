"""Keep 800 train / 100 validation frames; replace the disclosed test with 100 fresh frames."""
import argparse,json
from pathlib import Path
import yaml
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--source',type=Path,required=True);p.add_argument('--heldout',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
a=p.parse_args();source=a.source.resolve();heldout=a.heldout.resolve();out=a.output.resolve()
old=json.loads((source/'manifest.json').read_text());new=json.loads((heldout/'manifest.json').read_text())
for key in ('classes','camera_intrinsics','tag_family','tag_size_m'):
    if old[key]!=new[key]:raise ValueError(f'Mismatched dataset setting: {key}')
training=[s for s in old['samples'] if s['split'] in ('train','val')]
test=[s for s in new['samples'] if s['split']=='test']
assert len([s for s in training if s['split']=='train'])==800
assert len([s for s in training if s['split']=='val'])==100
assert len(test)==100
assert not {s['seed'] for s in old['samples']} & {s['seed'] for s in test}
assert (source/'COMPLETE').exists() and (heldout/'COMPLETE').exists()
out.mkdir(parents=True,exist_ok=False)
for kind in ('images','labels'):
    (out/kind).mkdir()
    for split in ('train','val','test'):
        (out/kind/split).symlink_to((heldout if split=='test' else source)/kind/split,target_is_directory=True)
manifest=dict(old);manifest['samples']=training+test
manifest['evaluation_provenance']={'train_val_source':str(source),'new_test_source':str(heldout),'old_test_excluded':True}
(out/'manifest.json').write_text(json.dumps(manifest,indent=2))
(out/'dataset.yaml').write_text(yaml.safe_dump(dict(path=str(out),train='images/train',val='images/val',test='images/test',names=dict(enumerate(manifest['classes']))),sort_keys=False))
(out/'COMPLETE').write_text('800 unchanged training / 100 unchanged validation / 100 fresh independent test frames\n')
print(out)
