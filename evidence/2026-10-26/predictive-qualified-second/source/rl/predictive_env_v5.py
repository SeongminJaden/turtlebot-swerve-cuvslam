"""v4 sensor policy task with consistent balanced wheel collision-proxy inertia.
Original CAD remains untouched. Mass/torque still require physical calibration.
"""
from predictive_env_v4 import PredictiveEnv,PredictiveEnvCfg,configure as previous_configure
from maze_env import ROOT

def configure(cfg):
 cfg=previous_configure(cfg)
 cfg.scene.robot.spawn.usd_path=str(ROOT/'assets/m38_balanced_wheels.usda')
 return cfg
