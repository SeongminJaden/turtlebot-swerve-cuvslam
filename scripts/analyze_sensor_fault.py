"""Check fail-stop/recovery from recorded real ROS + PhysX controller rows."""
import argparse,json
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('directory',type=Path);p.add_argument('--kind',choices=['imu2','camera'],required=True);p.add_argument('--expect-latched',action='store_true');p.add_argument('--start',type=float,required=True);p.add_argument('--duration',type=float,required=True);a=p.parse_args()
r=json.loads((a.directory/'report.json').read_text());rows=r['rows'];flag='sensor_ok' if a.kind=='imu2' else 'tracking_fresh'
bad=[x for x in rows if a.start+.35<=x['sim_s']<=a.start+a.duration and not x[flag]]
stop=[x for x in bad if x['speed_m_s']<.03];recover=[x for x in rows if x['sim_s']>a.start+a.duration+.2 and x[flag] and x['speed_m_s']>.03]
# .92/.6 seconds maximum commanded ramp plus sensor detection/control latency.
budget=.92/.6+.4
result={'kind':a.kind,'start_s':a.start,'duration_s':a.duration,'invalid_rows':len(bad),'stop_latency_s':stop[0]['sim_s']-a.start if stop else None,'recovery_motion':bool(recover),'max_speed_m_s':r['physics_peak_speed_m_s'],'peak_nonwheel_contact_force_N':r['peak_nonwheel_contact_force_N'],'scope':'injected ROS measurement outage in simulation, not physical USB fault test'}
result['passed']=bool(bad and stop and result['stop_latency_s']<=budget and ((r['outcome']=='tracking_quality_fault' and not recover) if a.expect_latched else bool(recover)) and r['physics_peak_speed_m_s']<=.951 and r['peak_nonwheel_contact_force_N']<=.5)
(a.directory/'fault-evaluation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
