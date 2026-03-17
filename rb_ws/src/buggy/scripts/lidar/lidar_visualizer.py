from time import time
import numpy as np
import open3d as o3d
from time import time
from sklearn import linear_model

# TODO: add all constsants to constants config file

def ground_plane_segmentation(data):
    # center the data to improve RANSAC stability (not strictly necessary since data is near origin and has small dimensions
    data_mean = np.mean(data, axis=0)
    centered_data = data - data_mean
  
    mask = centered_data[:, 2] < 0
    ground = centered_data[mask]
    nonground = centered_data[~mask]

    # Robustly fit linear model with RANSAC algorithm
    ransac = linear_model.RANSACRegressor(residual_threshold=0.1)   # 10 cm threshold for inliers
    ransac.fit(ground[:, :2], ground[:, 2])
    inlier_mask = ransac.inlier_mask_
    outlier_mask = ~inlier_mask

    g = ground[inlier_mask]
    ng = np.concatenate([ground[outlier_mask], nonground])

    # plane parameters: z = ax + by + c
    a, b = ransac.estimator_.coef_
    c = ransac.estimator_.intercept_

    # compute signed distance to plane
    # positive = above ground, negative = below ground
    z_pred = a * ng[:, 0] + b * ng[:, 1] + c
    dist = ng[:, 2] - z_pred

    # keep only points above the plane (with small margin)
    # TODO: consider if we need to do this
    ng_clean = ng[dist > -0.02]  # allow 2 cm below as tolerance

    return g + data_mean, ng_clean + data_mean


def cluster_volume(cluster_points):
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(cluster_points)

    bbox = pc.get_axis_aligned_bounding_box()
    bbox.color = (1, 0, 0)

    return bbox.volume(), bbox

def euclidean_clustering(data, eps=0.35, min_points=20, min_height=0.0):
    """
    data: (N,3) numpy array of non-ground points
    min_height: reject clusters with bounding-box height below this threshold
    returns: list of clusters, each a (Ki,3) numpy array
    """

    # Convert to Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(data)

    # Run DBSCAN = Euclidean clustering in Open3D
    # TODO: add weighted distance function to account for dbscan assumption that density of points is uniform
    labels = np.array(
        pcd.cluster_dbscan(
            eps=eps,          # radius for neighborhood search
            min_points=min_points,
            print_progress=False
        )
    )

    max_label = labels.max()
    print(f"Found {max_label + 1} clusters")

    clusters = []

    for i in range(max_label + 1):
        cluster_i = data[labels == i]
        clusters.append(cluster_i)
    MIN_VOL = 0.1
    MAX_VOL = 30.0
    MIN_HEIGHT = float(min_height)

    filtered_clusters = []
    filtered_boxes = []

    for c in clusters:
        vol, bbox = cluster_volume(c)
        height = bbox.get_extent()[2]

        if MIN_VOL < vol < MAX_VOL and height >= MIN_HEIGHT:
            filtered_clusters.append(c)
            filtered_boxes.append(bbox)

    print(f"Kept {len(filtered_clusters)} / {len(clusters)} clusters")

    return filtered_clusters, filtered_boxes, labels

# check if point is an ellipse of width and height
# (move the ellipse's center forward by forward offset)
def point_in_ellipse(point: np.ndarray, half_width, half_depth, forward_offset):
    """
    Checks whether a point is in ellipse
    forward direction is -x
    left direction is -y
    """
    forward = -point[0]
    lateral = point[1]
    return ((lateral / half_width)**2 + ((forward+forward_offset) / half_depth)**2) <= 1

def cluster_mean_in_ellipse (cluster, half_width, half_depth, forward_offset):
    mean_point = np.mean(cluster, axis=0) # get the x,y,z mean
    return point_in_ellipse(mean_point, half_width, half_depth, forward_offset)

def filter_clusters (clusters, half_width, half_depth, forward_offset):
    return [c for c in clusters if cluster_mean_in_ellipse(c, half_width, half_depth, forward_offset)]


#TODO: create a function that creates points to show the ellipse we are bounding to

def filter_points_in_circle(points: np.ndarray, radius):
    distances = np.linalg.norm(points[:, :2], axis=1)
    mask = distances < radius
    return points[mask]

def filter_points_by_angle(points: np.ndarray, min_angle=0.0, max_angle=0.0):
    """
    Filters points based on their radial angle in the XY plane.
    Robot frame: Straight ahead is negative X, left is negative Y.
    0 is straight ahead (negative X axis), and angles wrap counter-clockwise.
    Allows input range from 0 to 2pi.
    """
    # If the range makes a complete circle (e.g., 0 to 2pi), keep everything
    if min_angle % (2 * np.pi) == max_angle % (2 * np.pi):  # captures min_angle == max_angle
        return points
    
    # Calculate angles: 0 -> (-x=1, -y=0), pi/2 -> (-x=0, -y=1)
    angles = np.arctan2(-points[:, 1], -points[:, 0])
    
    # Normalize angles and constraints into 0 to 2pi
    angles = np.mod(angles, 2 * np.pi)
    min_a = np.mod(min_angle, 2 * np.pi)
    max_a = np.mod(max_angle, 2 * np.pi)
    
    if min_a < max_a:
        mask = (angles >= min_a) & (angles <= max_a)
    else:
        # Handles wrapping around the 0 / 2pi boundary
        mask = (angles >= min_a) | (angles <= max_a)

    return points[mask]

def create_grid(size=15, n=15, z=0.0):
    """Creates a line grid on the XY plane for visualization."""
    scale = size / n
    points = []
    lines = []
    for i in range(-n, n + 1):
        # Lines parallel to Y axis
        points.append([i * scale, -size, z])
        points.append([i * scale, size, z])
        lines.append([len(points)-2, len(points)-1])
        # Lines parallel to X axis
        points.append([-size, i * scale, z])
        points.append([size, i * scale, z])
        lines.append([len(points)-2, len(points)-1])
    
    grid = o3d.geometry.LineSet()
    grid.points = o3d.utility.Vector3dVector(points)
    grid.lines = o3d.utility.Vector2iVector(lines)
    grid.colors = o3d.utility.Vector3dVector([[0.5, 0.5, 0.5]] * len(lines))
    return grid


def create_ellipse_torus(half_width, half_depth, forward_offset=0.0, z=0.0,
                         tube_radius=0.05, color=(0.0, 1.0, 0.0)):
    torus = o3d.geometry.TriangleMesh.create_torus(torus_radius=1.0, tube_radius=tube_radius)
    v = np.asarray(torus.vertices)
    v[:, 0] *= half_depth  # forward axis
    v[:, 1] *= half_width   # lateral axis
    torus.vertices = o3d.utility.Vector3dVector(v)
    torus.compute_vertex_normals()
    torus.paint_uniform_color(color)
    torus.translate([forward_offset, 0.0, z])
    return torus

def main():
    data = np.load('velodyne_points.npy', allow_pickle=True) # 'sc_feb_21_26_roll_1.npz' 'velodyne_points.npy'
    # data = np.vstack([file['frames'][600], file['frames'][601], file['frames'][602], file['frames'][603], file['frames'][604]])  # merge 5 frames for complete rotation

    print(data.shape)
    print(np.max(data, axis=0))

    time_start = time()
    g, ng_clean = ground_plane_segmentation(data)

    pcd_g = o3d.geometry.PointCloud()
    pcd_g.points = o3d.utility.Vector3dVector(g)
    pcd_g.paint_uniform_color([1, 0, 0])

    ng_filtered = filter_points_in_circle(ng_clean, radius=8)
    # ng_filtered = filter_points_by_angle(ng_filtered, min_angle=17/16*np.pi, max_angle=0)   # filter out left side due to left-passing behavior; ignores pushers on left side
    clusters, boxes, labels = euclidean_clustering(ng_filtered)

    ellipse_half_width = 2
    ellipse_half_depth = 5
    ellipse_forward_offset = 1.5
    ranged_clusters = filter_clusters(clusters,
                                      half_width=ellipse_half_width,
                                      half_depth=ellipse_half_depth,
                                      forward_offset=ellipse_forward_offset)

    # simple coloring: each cluster gets a different color
    # create a list of every point present in the filtered clusters (in range)

    in_range_ng = []
    colors = []
    for cluster in ranged_clusters:
        color = np.random.rand(3)
        for point in cluster:
            in_range_ng.append(point)
            colors.append(color)

    pcd_ng = o3d.geometry.PointCloud()
    pcd_ng.points = o3d.utility.Vector3dVector(ng_clean)
    pcd_ng.paint_uniform_color([0, 0, 0])

    # color clusters differently for visualization

    pcd_ranged_ng = o3d.geometry.PointCloud()
    pcd_ranged_ng.points = o3d.utility.Vector3dVector(in_range_ng)
    pcd_ranged_ng.colors = o3d.utility.Vector3dVector(colors)

    boxes = []
    for c in ranged_clusters:
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(c)
        bbox = pc.get_axis_aligned_bounding_box()
        bbox.color = (1, 0, 0)
        boxes.append(bbox)

    time_end = time()
    print(f"Processing time: {time_end - time_start:.2f} seconds")


    # pcd_g = pcd_g.voxel_down_sample(0.05)
    # pcd_ranged_ng = pcd_ranged_ng.voxel_down_sample(0.1)

    center = [0, 0, -0.5]
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2.0, origin=center)
    grid = create_grid(size=15, n=15, z=center[2])

    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.get_render_option().line_width = 100.0
    # Match your filter_clusters call exactly
    ellipse = create_ellipse_torus(
        half_width=ellipse_half_width, 
        half_depth=ellipse_half_depth, 
        forward_offset=ellipse_forward_offset,
        z=center[2],          # same z as your grid/axes
        color=(0.0, 1.0, 0.0) # green
    )

    for g in [pcd_ng, pcd_g, pcd_ranged_ng, axes, grid, ellipse] + boxes:
        vis.add_geometry(g)

    ctr = vis.get_view_control()
    ctr.set_lookat(center)
    ctr.set_zoom(0.8)
    ctr.set_up([0, 0, 1])  # Z is up
    ctr.set_front([1, 0, 0.5])  # camera behind and above the origin (look straight ahead with downward tilt)

    vis.run()
    vis.destroy_window()


if __name__ == '__main__':
    main()
