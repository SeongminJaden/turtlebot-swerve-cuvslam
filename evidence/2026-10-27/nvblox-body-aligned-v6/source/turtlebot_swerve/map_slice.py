"""Reproject a 2-D distance slice without inventing unobserved free space."""
import numpy as np


def reproject_slice(data, origin, resolution, transform, unknown):
    source = np.asarray(data, dtype=float)
    transform = np.asarray(transform, dtype=float)
    if source.ndim != 2 or not source.size or resolution <= 0:
        raise ValueError('invalid slice dimensions or resolution')
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError('invalid transform')
    if (not np.allclose(transform[2], [0, 0, 1, 0], atol=.01)
            or not np.allclose(transform[:2, 2], 0, atol=.01)
            or not np.allclose(transform[:2, :2].T @ transform[:2, :2], np.eye(2), atol=1e-5)):
        raise ValueError('slice transform must be planar and rigid')
    height, width = source.shape
    rotation, translation = transform[:2, :2], transform[:2, 3]
    origin = np.asarray(origin, dtype=float)[:2]
    # Slice coordinates denote sample centers, matching NvbloxCostmapLayer.
    corners = origin + resolution * np.array([[0, 0], [width-1, 0], [0, height-1], [width-1, height-1]])
    corners = corners @ rotation.T + translation
    lower = np.floor(corners.min(axis=0) / resolution) * resolution
    shape = np.ceil((corners.max(axis=0) - lower) / resolution).astype(int) + 1
    yy, xx = np.indices((shape[1], shape[0]))
    world = lower + resolution * np.stack([xx, yy], axis=-1)
    local = (world - translation) @ rotation
    index = np.floor((local - origin) / resolution + .5).astype(int)
    valid = (index[..., 0] >= 0) & (index[..., 0] < width) & (index[..., 1] >= 0) & (index[..., 1] < height)
    output = np.full(xx.shape, unknown, dtype=float)
    output[valid] = source[index[..., 1][valid], index[..., 0][valid]]
    output[~np.isfinite(output)] = unknown
    return output, lower
