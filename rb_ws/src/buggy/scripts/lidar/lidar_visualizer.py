""" Lidar processing helpers for obstacle detection and visualization."""

from time import time
import numpy as np
import open3d as o3d
from time import time
from sklearn import linear_model
from scipy.spatial import cKDTree
from lidar_helpers import *

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
    pcd_ng.points = o3d.utility.Vector3dVector(ng)
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
