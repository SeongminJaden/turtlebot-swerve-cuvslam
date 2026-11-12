"""Keep a bounded arm's rear-facing branch stable across base yaw noise."""
import math

def target_for_heading(heading, base_yaw, previous_target, limit=3.05):
    raw=heading-base_yaw
    candidates=[max(-limit,min(limit,raw+2*math.pi*k)) for k in [-1,0,1]]
    def error(q):return abs(math.atan2(math.sin(q-raw),math.cos(q-raw)))
    best=min(error(q) for q in candidates)
    # Within 10 degrees of the best view, prefer continuity. Never ask a
    # bounded joint to cross from +pi to -pi because of a small yaw change.
    usable=[q for q in candidates if error(q)<=best+math.radians(10)]
    return min(usable,key=lambda q:abs(q-previous_target))
