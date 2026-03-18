""" Lidar processing helpers for obstacle detection and visualization."""

from time import time
import numpy as np
import open3d as o3d
from time import time
from sklearn import linear_model
from scipy.spatial import cKDTree

# TODO: add all constsants to constants config file

def ground_plane_segmentation(data):
    # center the data to improve RANSAC stability (not strictly necessary since data is near origin and has small dimensions
    data_mean = np.mean(data, axis=0)
    centered_data = data - data_mean
  
    mask = centered_data[:, 2] < 0
    ground = centered_data[mask]
    nonground = centered_data[~mask]

    # Robustly fit linear model with RANSAC algorithm
    ransac = linear_model.RANSACRegressor(residual_threshold=0.1, max_trials=1000)   # 10 cm threshold for inliers
    ransac.fit(ground[:, :2], ground[:, 2])
    inlier_mask = ransac.inlier_mask_
    outlier_mask = ~inlier_mask

    g = ground[inlier_mask]
    ng = np.concatenate([ground[outlier_mask], nonground])

    # plane parameters: z = ax + by + c
    # a, b = ransac.estimator_.coef_
    # c = ransac.estimator_.intercept_

    # # compute signed distance to plane
    # # positive = above ground, negative = below ground
    # z_pred = a * ng[:, 0] + b * ng[:, 1] + c
    # dist = ng[:, 2] - z_pred

    # # keep only points above the plane (with small margin)
    # # TODO: consider if we need to do this
    # ng_clean = ng[dist > -0.02]  # allow 2 cm below as tolerance

    return g + data_mean, ng + data_mean

def ground_plane_segmentation2(data):
    """ Optimized ground plane segmentation using Open3D's built-in RANSAC plane fitting.
    Note: this may be less accurate than our custom RANSAC implementation, but is twice as fast."""  
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(data)
    
    # distance_threshold: max distance a point can be from the plane to be an inlier
    # ransac_n: number of points sampled to estimate a plane
    # num_iterations: more = more accurate but slower
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.12,
                                             ransac_n=4,
                                             num_iterations=1000)
    
    # [a, b, c, d] where ax + by + cz + d = 0
    # [a, b, c, d] = plane_model
    
    # Extract points
    ground_cloud = pcd.select_by_index(inliers)
    nonground_cloud = pcd.select_by_index(inliers, invert=True)
    
    return np.asarray(ground_cloud.points), np.asarray(nonground_cloud.points)


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
    MIN_VOL = 0.05
    MAX_VOL = 10.0
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

def euclidean_clustering2(data, eps=0.35, min_points=20, min_height=0.0):
    """Optimized DBSCAN using Voxel Downsampling."""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(data)

    # 1. Downsample the cloud (10cm voxels)
    voxel_size = 0.1
    downpcd = pcd.voxel_down_sample(voxel_size)
    down_data = np.asarray(downpcd.points)

    # 2. Run DBSCAN on the sparse point cloud
    sparse_labels = np.array(
        downpcd.cluster_dbscan(
            eps=eps,          
            min_points=min_points,
            print_progress=False
        )
    )

    # 3. Map labels back to the dense cloud 
    # A KDTree lookup takes ~2ms for 50k points.
    tree = cKDTree(down_data)
    _, nearest_indices = tree.query(data)
    dense_labels = sparse_labels[nearest_indices]

    max_label = dense_labels.max()
    print(f"Found {max_label + 1} clusters")

    clusters = []
    for i in range(max_label + 1):
        # We group the ORIGINAL dense points using the mapped labels
        cluster_i = data[dense_labels == i]
        clusters.append(cluster_i)

    MIN_VOL = 0.1
    MAX_VOL = 30.0
    MIN_HEIGHT = float(min_height)

    filtered_clusters = []
    filtered_boxes = []

    for c in clusters:
        # Open3D bounding boxes require at least 4 points
        if len(c) < 4:
            continue
            
        vol, bbox = cluster_volume(c)
        height = bbox.get_extent()[2]

        if MIN_VOL < vol < MAX_VOL and height >= MIN_HEIGHT:
            filtered_clusters.append(c)
            filtered_boxes.append(bbox)

    print(f"Kept {len(filtered_clusters)} / {len(clusters)} clusters")

    return filtered_clusters, filtered_boxes, dense_labels

def identify_best_cluster(clusters, boxes):
    """ Identify the cluster most likely to be an obstacle we need to detect.
    We use the following heuristics:
    - cluster closest to the buggy (smallest mean distance in XY plane)
    - cluster with reasonable size (volume or any dimension not too small/large)
    - prioritize clusters in front of the buggy (negative X direction) over sides or back, weighted as a forward-heavy ellipse
    """
    best_cluster = None
    best_score = float('inf')

    for (cluster, box) in zip(clusters, boxes):
        center_point = box.get_center()
        distance = np.linalg.norm(center_point[:2])  # distance in XY plane
        size = box.volume()  # volume of bounding box
        # Heuristic scoring function: prioritize closer clusters, penalize large clusters, and weight front clusters more
        angle = np.arctan2(-center_point[1], -center_point[0])  # angle in XY plane (0 = straight ahead)
        angle_weight = 1.0 + 0.5 * np.cos(angle)  # front clusters (angle ~ 0) get weight ~1.5, side clusters (angle ~ ±pi/2) get weight ~1.0, back clusters (angle ~ pi) get weight ~0.5
        large_cluster_penalty = 1.0 * max(0, size - 1.0) + 1.0 * max(0, box.get_max_extent() - 3.0)  # penalize clusters larger than 1.0 m^3 or longer than 3.0 m in any dimension
        small_cluster_penalty = 1.5 * max(0, 0.2 - size) + 1.5 * max(0, 0.2 - np.min(box.get_extent()))  # penalize clusters smaller than 0.2 m^3 or shorter than 0.2 m in any dimension
        score = distance / angle_weight + large_cluster_penalty + small_cluster_penalty
        if score < best_score:
            best_score = score
            best_cluster = cluster

    if best_score <= 8.0:  # only consider clusters with reasonable score
      return best_cluster
    else:
      return None

def point_in_circle(point : np.ndarray, radius):
    distance = np.linalg.norm(point[:2]) #x, y
    if distance < radius:
        return True
    return False

def cluster_mean_in_circle (cluster, radius):
    mean_point = np.mean(cluster, axis=0) # get the x,y,z mean
    return point_in_circle(mean_point, radius)

def filter_clusters (clusters, radius=5):
    return [c for c in clusters if cluster_mean_in_circle(c, radius)]

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

def main():
    file = np.load('sc_feb_21_26_roll_1.npz', allow_pickle=True) # 'sc_feb_21_26_roll_1.npz' 'velodyne_points.npy'
    data = np.vstack([file['frames'][600], file['frames'][601], file['frames'][602], file['frames'][603], file['frames'][604]])  # merge 5 frames for complete rotation

    print(data.shape)
    print(np.max(data, axis=0))

    time_start = time()
    g, ng = ground_plane_segmentation(data)

    pcd_g = o3d.geometry.PointCloud()
    pcd_g.points = o3d.utility.Vector3dVector(g)
    pcd_g.paint_uniform_color([1, 0, 0])

    ng_filtered = filter_points_in_circle(ng, radius=8)
    # ng_filtered = filter_points_by_angle(ng_filtered, min_angle=17/16*np.pi, max_angle=1/4*np.pi)   # filter out left side due to left-passing behavior; ignores pushers on left side
    clusters, boxes, labels = euclidean_clustering(ng_filtered)

    # simple coloring: each cluster gets a different color
    # create a list of every point present in the filtered clusters (in range)
    in_range_ng = []
    colors = []
    for cluster in clusters:
        color = np.random.rand(3)
        for point in cluster:
            in_range_ng.append(point)
            colors.append(color)

    pcd_ng = o3d.geometry.PointCloud()
    pcd_ng.points = o3d.utility.Vector3dVector(ng)
    pcd_ng.paint_uniform_color([0, 0, 0])

    # color clusters differently for visualization
    pcd_ranged_ng = o3d.geometry.PointCloud()
    pcd_ranged_ng.points = o3d.utility.Vector3dVector(in_range_ng)
    pcd_ranged_ng.colors = o3d.utility.Vector3dVector(colors)

    boxes = []
    for c in clusters:
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
    for g in [pcd_ng, pcd_g, pcd_ranged_ng, axes, grid] + boxes:
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
