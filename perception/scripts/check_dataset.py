import argparse,json
from collections import Counter
from pathlib import Path
import cv2,numpy as np
p=argparse.ArgumentParser();p.add_argument('dataset',type=Path);p.add_argument('--expected-splits',nargs=3,type=int);a=p.parse_args()
d=json.loads((a.dataset/'manifest.json').read_text());counts=Counter();splits={k:set() for k in ('train','val','test')};decoded=[];surface_checks=0
detector=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11))
for sample in d['samples']:
    image=cv2.imread(str(a.dataset/sample['image']))
    assert image is not None and image.shape==(640,640,3)
    splits.setdefault(sample['split'],set()).add(sample['seed'])
    labels=(a.dataset/sample['image'].replace('images/','labels/').replace('.png','.txt')).read_text().splitlines()
    for line in labels:
        c,x,y,w,h=map(float,line.split());assert c.is_integer() and 0<=c<6
        assert all(np.isfinite([x,y,w,h])) and 0<=x<=1 and 0<=y<=1 and 0<w<=1 and 0<h<=1
        counts[int(c)]+=1
    assert len(labels)==len(sample['boxes'])
    for label,box in zip(labels,sample['boxes']):
        c,x,y,w,h=map(float,label.split());x1,y1,x2,y2=box['xyxy']
        assert int(c)==box['class_id']
        np.testing.assert_allclose([x,y,w,h],[(x1+x2)/1280,(y1+y2)/1280,(x2-x1)/640,(y2-y1)/640],atol=1e-7)
        if 'surface_bounds_px' in box:
            bound=box['surface_bounds_px']
            assert x1>=bound[0]-2 and y1>=bound[1]-2 and x2<=bound[2]+2 and y2<=bound[3]+2,box
            assert 0<box['mask_pixel_count']<=(x2-x1)*(y2-y1)
            surface_checks+=1
        if box['class_id']<5:
            pose=next(p for p in sample['poses'] if p['class_id']==box['class_id'])
            uv=np.array(pose['projected_cuboid'])[1:]
            assert x1>=uv[:,0].min()-2 and y1>=uv[:,1].min()-2 and x2<=uv[:,0].max()+2 and y2<=uv[:,1].max()+2,box
    for pose in sample['poses']:
        if pose['class_id']==4:continue
        assert len(pose['tag_faces'])==(3 if pose['class_id']%2==0 else 6)
        assert len({f['tag_id'] for f in pose['tag_faces']})==1
    if 'layout' in sample:
        layout=sample['layout'];half=layout['box_size_m'][0]/2
        for puck in layout['pucks']:
            assert abs(puck['x'])+puck['extent_x']<=half and abs(puck['y'])+puck['extent_y']<=half
        for i,puck in enumerate(layout['pucks']):
            for other in layout['pucks'][i+1:]:
                assert np.hypot(puck['x']-other['x'],puck['y']-other['y'])>=puck['radius']+other['radius']
    if len(decoded)<12:
        corners,ids,_=detector.detectMarkers(image)
        if ids is not None:
            actual=ids.flatten().tolist();expected={f['tag_id'] for pose in sample['poses'] for f in pose['tag_faces']}
            if sample.get('layout',{}).get('center_tag_id') is not None:expected.add(sample['layout']['center_tag_id'])
            assert set(actual)<=expected,(actual,expected)
            decoded.append({'image':sample['image'],'decoded_ids':actual})
assert not splits['train']&splits['val'] and not splits['train']&splits['test'] and not splits['val']&splits['test']
assert set(counts)==set(range(6)) and decoded
if a.expected_splits:assert [len(splits[k]) for k in ('train','val','test')]==a.expected_splits
if 'instance segmentation' in d['annotation']:assert surface_checks==counts[5]
result={'tag_surface_bound_checks':surface_checks,'status':'PASS_DATASET','frames':len(d['samples']),'split_counts':{k:len(v) for k,v in splits.items()},'class_counts':{d['classes'][k]:v for k,v in counts.items()},'actual_rgb_tag_decoding_examples':decoded,'note':'Decode examples are a sanity check, not a tag recall metric; curved side tag pose is not planar PnP.'}
(a.dataset/'validation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
