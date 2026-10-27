"""Save the exact runtime sources/configuration before a verification run."""
import hashlib,json,os,shutil,sys
from pathlib import Path
root=Path(__file__).resolve().parents[1]
def snapshot(destination):
 destination=Path(destination);source=destination/'source';source.mkdir(parents=True,exist_ok=True)
 files=list((root/'turtlebot_swerve').glob('*.py'))+list((root/'rl').glob('*.py'))
 files += [root/'scripts'/name for name in ('active_camera_bridge.py','active_camera_renderer.py','isaac_sensor_bridge.py','robot_observation_adapter.py','depth_clearance.py','pose_quality.py','pose_covariance.py','pose_math.py','tracking_recovery.py','activate_localization.py','run_predictive_async.sh','nav2_policy_path.py','nvblox_map_slice.py')]
 files += list((root/'config').glob('*.yaml'))+list((root/'launch').glob('*.py'))
 hashes={}
 for path in files:
  relative=path.relative_to(root);target=source/relative;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(path,target);hashes[str(relative)]=hashlib.sha256(path.read_bytes()).hexdigest()
 for path in [root/'assets/m38_active_camera_demo.usda',root/'assets/m38_runtime.usda',root/'assets/m38_balanced_wheels.usda',root/'assets/maze/maze.usda',root/'assets/maze/route_footprint.json',root/'assets/maze/route_clearance.json',root/'assets/maze/route_clearance_second.json',root/'assets/maze/localization/maze.pgm',root/'assets/maze/localization/maze.yaml',root/'assets/maze/slam_floor_texture.png',root/'assets/m38_arm_description/urdf/m38_arm.urdf']:
  hashes[str(path.relative_to(root))]=hashlib.sha256(path.read_bytes()).hexdigest()
 (destination/'runtime-provenance.json').write_text(json.dumps({'source_sha256':hashes,'ros_domain_id':os.getenv('ROS_DOMAIN_ID'),'fault_imu2_start':os.getenv('FAULT_IMU2_START'),'fault_camera_start':os.getenv('FAULT_CAMERA_START')},indent=2))
if __name__=='__main__':snapshot(sys.argv[1])
