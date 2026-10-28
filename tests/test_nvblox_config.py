from pathlib import Path
import yaml

def test_image_time_tf_and_costmap_plugin():
    nv=yaml.safe_load(Path('config/nvblox_arm.yaml').read_text())['nvblox_node']['ros__parameters']
    assert nv['use_tf_transforms'] and nv['global_frame']=='odom'
    assert nv['static_mapper']['esdf_slice_min_height']>0
    nav=yaml.safe_load(Path('config/nav2_nvblox.yaml').read_text())
    p=nav['global_costmap']['global_costmap']['ros__parameters']
    assert p['plugins']==['static_layer','nvblox_layer','inflation_layer']
    assert p['nvblox_layer']['plugin']=='nvblox::nav2::NvbloxCostmapLayer'
    assert not nav['planner_server']['ros__parameters']['GridBased']['allow_unknown']
