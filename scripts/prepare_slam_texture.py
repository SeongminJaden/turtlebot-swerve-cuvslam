"""Deterministic synthetic floor texture for a disclosed visual-SLAM experiment.
Does not alter maze collision geometry. Not a photograph of the physical maze.
"""
from pathlib import Path
import numpy as np
from PIL import Image,ImageFilter
rng=np.random.default_rng(42)
grid=rng.integers(55,215,(128,128),dtype=np.uint8)
im=Image.fromarray(grid).resize((2048,2048),Image.Resampling.NEAREST).filter(ImageFilter.GaussianBlur(.8))
p=Path('assets/maze/slam_floor_texture.png');im.save(p);print(p)
