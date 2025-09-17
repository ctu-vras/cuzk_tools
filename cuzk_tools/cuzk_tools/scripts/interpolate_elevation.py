import numpy as np
from scipy.interpolate import LinearNDInterpolator, NearestNDInterpolator

def interpolate_elevation(points, near_points):
    """
    Interpolate elevation from an array of xyz points to an array of xy points.

    Parameters
    ----------
    points : np.ndarray
        Points (xy) which elevation (z coord) we want to obtain by interpolation.
    near_points : np.ndarray
        Points (xyz) with known elevation which we use for the interpolation.
    """

    interpolator = LinearNDInterpolator(near_points[:,:2], near_points[:,2])

    elevation = interpolator(points)        

    # Some points on the outskirts cannot be linearly interpolated to. Use 'nearest' instead.
    nan_points = points[np.isnan(elevation)]

    if len(nan_points) > 0:
        interpolator = NearestNDInterpolator(near_points[:,:2], near_points[:,2])
        nan_elevation = interpolator(nan_points)
        elevation[np.isnan(elevation)] = nan_elevation
    
    return elevation