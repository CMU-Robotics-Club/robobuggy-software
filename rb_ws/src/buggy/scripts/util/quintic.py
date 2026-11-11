
from __future__ import annotations

import numpy as np
import numpy.typing as npt
import matplotlib.pyplot as plt
from scipy.interpolate import RegularGridInterpolator, interp1d
import json
from pyproj import Transformer

with open("rb_ws\\src\\buggy\\paths\\buggycourse_safe.json", "r") as f:
    json_data = json.load(f)

# 2. Setup LLA to UTM Zone 17N (EPSG:32617) transformation
# EPSG:4326 is WGS84 Geodetic (Lat/Lon)
# EPSG:32617 is WGS84 / UTM Zone 17N
# always_xy=True ensures output is (Easting/x, Northing/y) matching Julia's UTM expectations
transformer = Transformer.from_crs("EPSG:4326", "EPSG:32617", always_xy=True)

# 3. Transform coordinates and extract x (Easting) and y (Northing)
x_traj_raw = []
y_traj_raw = []

for item in json_data:
    # pyproj expects longitude first when always_xy=True
    x, y = transformer.transform(item["lon"], item["lat"])
    x_traj_raw.append(x)
    y_traj_raw.append(y)

# Ensure the inputs are numpy arrays for easy math operations
x_arr = np.array(x_traj_raw)
y_arr = np.array(y_traj_raw)

# 1. Calculate cumulative arc-length distances vectorised
dx = np.diff(x_arr)
dy = np.diff(y_arr)
step_lengths = np.sqrt(dx**2 + dy**2)

# Construct the distance array starting at 0.0
distances = np.zeros(len(x_arr))
distances[1:] = np.cumsum(step_lengths)

tot_len = distances[-1]

# 2. Set up linear interpolations (equivalent to linear_interpolation)
# fill_value="extrapolate" mimics Julia's default interpolation bounds handling
x_traj_lin = interp1d(distances, x_arr, kind='linear', fill_value="extrapolate")
y_traj_lin = interp1d(distances, y_arr, kind='linear', fill_value="extrapolate")

# 3. Create the evaluation grid from 0.0 to tot_len with a step of 10.0
# np.arange excludes the endpoint if it doesn't land perfectly on it, 
# just like Julia's 0.0:10.0:tot_len range syntax
query_distances = np.arange(0.0, tot_len + 1e-9, 10.0)

x_resampled = x_traj_lin(query_distances)
y_resampled = y_traj_lin(query_distances)

from scipy.interpolate import CubicSpline
def cubic_interpolation_parameterized():
    d = np.linspace(0.0, tot_len, num=len(x_traj_raw))

    # Evaluate the previous linear interpolations on this new grid
    x_d = x_traj_lin(d)
    y_d = y_traj_lin(d)

    # 2. Fit Cubic Splines
    x_traj_cubic = CubicSpline(d, x_d, bc_type='not-a-knot')
    y_traj_cubic = CubicSpline(d, y_d, bc_type='not-a-knot')

    # 3. Extract coefficients [a, b, c, d] for both x and y
    # x_traj_cubic.c has shape (4, N-1). We transpose (.T) to get (N-1, 4)
    coef_x = x_traj_cubic.c.T
    coef_y = y_traj_cubic.c.T

    # 4. Pad the final row for the last point
    # N grid points generate N-1 intervals. We append NaNs for the final row
    pad_row = np.array([[np.nan, np.nan, np.nan, np.nan]])
    coef_x_padded = np.vstack([coef_x, pad_row])
    coef_y_padded = np.vstack([coef_y, pad_row])

    # 5. Construct the final matrix
    # Format: [x_coord, y_coord, ax, bx, cx, dx, ay, by, cy, dy]
    output_matrix = np.hstack([
        x_d.reshape(-1, 1),
        y_d.reshape(-1, 1),
        coef_x_padded,
        coef_y_padded
    ])

    return output_matrix

def cubic_interpolation_tangent_curvature():
    d = np.linspace(0.0, tot_len, num=len(x_traj_raw))

    # Evaluate the previous linear interpolations on this new grid
    x_d = x_traj_lin(d)
    y_d = y_traj_lin(d)

    # 2. Fit Cubic Splines
    x_traj_cubic = CubicSpline(d, x_d, bc_type='not-a-knot')
    y_traj_cubic = CubicSpline(d, y_d, bc_type='not-a-knot')

    # 3. Compute derivatives for tangents and curvatures
    dx_dt = x_traj_cubic.derivative()(d)
    dy_dt = y_traj_cubic.derivative()(d)
    ddx_dt2 = x_traj_cubic.derivative(2)(d)
    ddy_dt2 = y_traj_cubic.derivative(2)(d)

    # Tangent vectors (dx/dt, dy/dt)
    tangents = np.column_stack((dx_dt, dy_dt))

    # Curvature using the formula: kappa = (x'y'' - y'x'') / (x'^2 + y'^2)^(3/2)
    curvature_numerator = dx_dt * ddy_dt2 - dy_dt * ddx_dt2
    curvature_denominator = (dx_dt**2 + dy_dt**2)**(3/2)
    
    # Avoid division by zero; set curvature to zero where denominator is zero
    curvature = np.where(curvature_denominator != 0, curvature_numerator / curvature_denominator, 0.0)

    # 4. Construct the final matrix
    # Format: [x_coord, y_coord, tangent_x, tangent_y, curvature]
    output_matrix = np.column_stack([
        x_d,
        y_d,
        tangents,
        curvature
    ])

    return output_matrix

def quintic_interpolate(waypoints: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
    """Fits a quintic polynomial between every consecutive pair of waypoints.

    Each segment i (from waypoint i to waypoint i+1) is parameterized by a local
    parameter t in [0, 1] (t = 0 at waypoint i, t = 1 at waypoint i+1), matching
    the index-based parameterization used by Trajectory:

        x(t) = a_x t^5 + b_x t^4 + c_x t^3 + d_x t^2 + e_x t + f_x
        y(t) = a_y t^5 + b_y t^4 + c_y t^3 + d_y t^2 + e_y t + f_y

    The six coefficients per axis are solved from six boundary conditions:
    position, first derivative, and second derivative at both ends.

    - First derivative: the tangent direction is normalized and scaled by the chord
      length L between the two waypoints, so |r'(t)| ~ L (t is roughly arc length / L).
    - Second derivative: chosen perpendicular to the tangent such that the signed
      curvature (x'y'' - y'x'') / |r'|^3 equals the given curvature, i.e.
      r'' = kappa * L^2 * n_hat, where n_hat is the tangent rotated +90 degrees.
      Positive curvature = turning left (counter-clockwise).

    Args:
        waypoints (num_pts x 5): columns are
            [x, y, tangent_x, tangent_y, curvature]

    Returns:
        (num_pts - 1 x 14): one row per segment, columns are
            [x_i, y_i, a_x, b_x, c_x, d_x, e_x, f_x, a_y, b_y, c_y, d_y, e_y, f_y]
        where (x_i, y_i) is the starting waypoint of the segment.
    """
    waypoints = np.asarray(waypoints, dtype=np.float64)
    if waypoints.ndim != 2 or waypoints.shape[1] != 5:
        raise ValueError(f"waypoints must be (num_pts x 5), got {waypoints.shape}")
    if waypoints.shape[0] < 2:
        raise ValueError("need at least 2 waypoints to interpolate")

    positions: npt.NDArray[np.float64] = waypoints[:, 0:2]  # (N x 2)
    tangents: npt.NDArray[np.float64] = waypoints[:, 2:4]  # (N x 2)
    curvatures: npt.NDArray[np.float64] = waypoints[:, 4]  # (N,)

    # unit tangents and left-hand unit normals at each waypoint
    tangent_norms: npt.NDArray[np.float64] = np.linalg.norm(tangents, axis=1, keepdims=True)  # (N x 1)
    if np.any(tangent_norms == 0):
        raise ValueError("tangent vectors must be nonzero")
    unit_tangents: npt.NDArray[np.float64] = tangents / tangent_norms  # (N x 2)
    unit_normals: npt.NDArray[np.float64] = np.column_stack(
        (-unit_tangents[:, 1], unit_tangents[:, 0])
    )  # (N x 2), tangent rotated +90 deg

    # per-segment boundary values
    p0: npt.NDArray[np.float64] = positions[:-1]  # (N-1 x 2)
    p1: npt.NDArray[np.float64] = positions[1:]  # (N-1 x 2)
    delta: npt.NDArray[np.float64] = p1 - p0  # (N-1 x 2)
    chord_lengths: npt.NDArray[np.float64] = np.linalg.norm(delta, axis=1, keepdims=True)  # (N-1 x 1)

    v0: npt.NDArray[np.float64] = unit_tangents[:-1] * chord_lengths  # r'(0), (N-1 x 2)
    v1: npt.NDArray[np.float64] = unit_tangents[1:] * chord_lengths  # r'(1), (N-1 x 2)
    a0: npt.NDArray[np.float64] = (
        curvatures[:-1, None] * chord_lengths**2 * unit_normals[:-1]
    )  # r''(0), (N-1 x 2)
    a1: npt.NDArray[np.float64] = (
        curvatures[1:, None] * chord_lengths**2 * unit_normals[1:]
    )  # r''(1), (N-1 x 2)

    # closed-form quintic Hermite coefficients on t in [0, 1]
    a: npt.NDArray[np.float64] = 6.0 * delta - 3.0 * v0 - 3.0 * v1 - 0.5 * a0 + 0.5 * a1
    b: npt.NDArray[np.float64] = -15.0 * delta + 8.0 * v0 + 7.0 * v1 + 1.5 * a0 - a1
    c: npt.NDArray[np.float64] = 10.0 * delta - 6.0 * v0 - 4.0 * v1 - 1.5 * a0 + 0.5 * a1
    d: npt.NDArray[np.float64] = 0.5 * a0
    e: npt.NDArray[np.float64] = v0
    f: npt.NDArray[np.float64] = p0

    # stack into (N-1 x 6 x 2): coefficient index x axis
    coeffs: npt.NDArray[np.float64] = np.stack((a, b, c, d, e, f), axis=1)

    result: npt.NDArray[np.float64] = np.hstack(
        (
            p0,  # x_i, y_i
            coeffs[:, :, 0],  # a_x .. f_x
            coeffs[:, :, 1],  # a_y .. f_y
        )
    )  # (N-1 x 14)
    return result


def evaluate_quintic_segment(
    segment: npt.NDArray[np.float64], t: npt.NDArray[np.float64] | float, nu: int = 0
) -> npt.NDArray[np.float64]:
    """Evaluates one row of quintic_interpolate's output at local parameter t in [0, 1].

    Args:
        segment (14,): one row of the quintic_interpolate output
        t: scalar or array of local parameters in [0, 1]
        nu: derivative order (0 = position, 1 = first derivative, ...)

    Returns:
        (..., 2): [x, y] (or their nu-th derivative w.r.t. t)
    """
    x_poly: np.poly1d = np.poly1d(segment[2:8])
    y_poly: np.poly1d = np.poly1d(segment[8:14])
    x_eval: npt.NDArray[np.float64] = np.asarray(x_poly.deriv(nu)(t) if nu else x_poly(t))
    y_eval: npt.NDArray[np.float64] = np.asarray(y_poly.deriv(nu)(t) if nu else y_poly(t))
    return np.stack((x_eval, y_eval), axis=-1)


def cubic_plot(cubicArray):
    #cubicArray should be a (numPoints-1)???x10 Array in the format xyabcdabcd
    
    #Get numPoints-1
    cubicPlotLength = len(cubicArray)

    for equation in cubicArray:
        #Get the X Parameterization and Y Parameterization
        xEquation=equation[2:6]
        yEquation=equation[6:10]

        #Create 100 time intervals in each path section
        tArray=np.linspace(0,10,100)

        #Get Points with parameterizations and time
        xParameter=[(xEquation[0]*t**3+xEquation[1]*t**2+xEquation[2]*t+xEquation[3]) for t in tArray]
        yParameter=[(yEquation[0]*t**3+yEquation[1]*t**2+yEquation[2]*t+yEquation[3]) for t in tArray]

        #Plot Onto A Graph
        plt.plot(xParameter,yParameter,"b-")




def quintic_plot(quinticArray):
    #quinticArray should be a (numPoints-1)???x14 Array in the format xyabcdefabcdef

    #Get numPoints-1
    quinticPlotLength = len(quinticArray)

    for equation in quinticArray:
        #Get the X Parameterization and Y Parameterization
        xEquation=equation[2:8]
        yEquation=equation[8:14]

        #Create 100 time intervals in each path section
        tArray=np.linspace(0,1,100)

        #Get Points with parameterizations and time
        xParameter=[(xEquation[0]*t**5+xEquation[1]*t**4+xEquation[2]*t**3+xEquation[3]*t**2+xEquation[4]*t+xEquation[5]) for t in tArray]
        yParameter=[(yEquation[0]*t**5+yEquation[1]*t**4+yEquation[2]*t**3+yEquation[3]*t**2+yEquation[4]*t+yEquation[5]) for t in tArray]
        print(xEquation[0])
        print(xEquation[1])
        print(yEquation[0])
        print(yEquation[1])

        #Plot Onto A Graph
        plt.plot(xParameter,yParameter,"r-")
        plt.scatter(equation[0],equation[1],c="k")
        

def main():
    cubicVisualArray = cubic_interpolation_parameterized()
    cubicArray = cubic_interpolation_tangent_curvature()
    quinticArray = quintic_interpolate(cubicArray)
    cubic_plot(cubicVisualArray)
    quintic_plot(quinticArray)
    plt.show()

if __name__ == "__main__":
    main()