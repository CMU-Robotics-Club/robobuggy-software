from time import time
import numpy as np
import open3d as o3d
from sklearn import linear_model
from time import time

def ground_plane_segmentation(data):
    data -= np.mean(data, axis=0)
    mask = data[:, 2] < 0
    ground = data[mask]
    nonground = data[~mask]

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

    # keep only points ABOVE the plane (with small margin)
    ng_clean = ng[dist > -0.02]  # allow 2 cm below as tolerance

    return g, ng_clean

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

def main():
    file = np.load('sc_feb_21_26_roll_1.npz', allow_pickle=True)
    data = file['frames'][0]

    print(data.shape)
    print(np.max(data, axis=0))

    time_start = time()
    g, ng_clean = ground_plane_segmentation(data)

    pcd_g = o3d.geometry.PointCloud()
    pcd_g.points = o3d.utility.Vector3dVector(g)
    pcd_g.paint_uniform_color([1, 0, 0])

    clusters, boxes, labels = euclidean_clustering(ng_clean)
    clusters = filter_clusters(clusters)

    # simple coloring: each cluster gets a different color
    # create a list of every point present in the filtered clusters (in range)
    in_range_ng = []
    colors = []
    for cluster in clusters:
        color = np.random.rand(3)
        for point in cluster:
            in_range_ng.append(point)
            colors.append(color)

    # # color clusters differently for visualization
    pcd_ng = o3d.geometry.PointCloud()
    pcd_ng.points = o3d.utility.Vector3dVector(ng_clean)
    pcd_ng.paint_uniform_color([0, 0, 0])

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


    pcd_g = pcd_g.voxel_down_sample(0.05)
    pcd_ranged_ng = pcd_ranged_ng.voxel_down_sample(0.1)

    o3d.visualization.draw_geometries([pcd_ng, pcd_g])
    o3d.visualization.draw_geometries([pcd_ranged_ng] + boxes)

if __name__ == '__main__':
    main()
