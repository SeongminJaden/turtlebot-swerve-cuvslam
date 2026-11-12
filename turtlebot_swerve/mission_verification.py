"""Fail-closed final-placement checks over fresh camera observation records.

This contract checker does not perform perception or certify provenance itself.
The adapter must provide actual frames, camera poses, and ID detections.
"""
import math


def verify_placements(allocation, observations, completed_placement_at):
    errors=[];all_ids=[]
    expected={loc['location']:loc for loc in allocation['locations']}
    if set(observations)!=set(expected):
        return {'passed':False,'errors':['Missing or extra location observations']}
    for name,location in expected.items():
        frames=observations[name]
        if len(frames)<2:
            errors.append(f'{name}: at least two observations required');continue
        timestamps=[f['time_s'] for f in frames]
        if not all(math.isfinite(t) for t in timestamps) or timestamps[0]<=completed_placement_at or any(b<=a or b-a>.25 for a,b in zip(timestamps,timestamps[1:])):
            errors.append(f'{name}: stale/nonmonotonic/gapped observations')
        if timestamps[-1]-timestamps[0]<2.-1e-9:
            errors.append(f'{name}: stability duration under 2 seconds')
        if len({f['frame_id'] for f in frames})!=len(frames):errors.append(f'{name}: reused frames')
        targets={t['id']:t['center_m'] for t in location['targets']}
        for frame in frames:
            if frame.get('source')!='camera':errors.append(f'{name}: non-camera evidence')
            pucks=frame['pucks'];ids=[p['id'] for p in pucks]
            if len(set(ids))!=len(ids) or set(ids)!=set(targets):
                errors.append(f'{name}: extra/missing/duplicate IDs');continue
            for puck in pucks:
                xyz=puck['center_m']
                if len(xyz)!=3 or not all(math.isfinite(v) for v in xyz) or math.dist(xyz,targets[puck['id']])>.01+1e-9:
                    errors.append(f'{name}: position tolerance exceeded')
        all_ids.extend(p['id'] for p in frames[-1]['pucks'])
    if len(all_ids)!=12 or set(all_ids)!=set(range(12)):
        errors.append('Global duplicate or missing puck')
    return {'passed':not errors,'errors':sorted(set(errors)), 'scope':'observation-record validation, not proof of camera provenance or collision-free handling'}


def verify_return(samples,zone):
    """Require full XY chassis polygon inside a configured zone for >=3 seconds."""
    if len(samples)<2:return False
    x0,y0,x1,y1=zone
    if not all(math.isfinite(v) for v in zone) or not x0<x1 or not y0<y1:return False
    if samples[-1]['time_s']-samples[0]['time_s']<3.-1e-9:return False
    previous=None
    for s in samples:
        if not math.isfinite(s['time_s']):return False
        if previous is not None and not 0<s['time_s']-previous<=.25:return False
        previous=s['time_s']
        if not math.isfinite(s['speed_mps']) or abs(s['speed_mps'])>.001:return False
        if not math.isfinite(s['yaw_rate_radps']) or abs(s['yaw_rate_radps'])>.001:return False
        polygon=s['chassis_polygon_m']
        if len(polygon)<3 or not all(math.isfinite(x) and math.isfinite(y) and x0<=x<=x1 and y0<=y<=y1 for x,y in polygon):return False
    return True
