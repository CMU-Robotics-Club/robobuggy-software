import numpy as np
import open3d as o3d
from sklearn import linear_model

def main():
    data = np.load('../../../../velodyne_points.npy')

    print(data.shape)
    data -= np.mean(data, axis=0)
    print(np.mean(data, axis=0))
    mask = data[:, 2] < 0
    ground = data[mask]
    nonground = data[~mask]

    # Robustly fit linear model with RANSAC algorithm
    ransac = linear_model.RANSACRegressor()
    ransac.fit(ground[:, :2], ground[:, 2])
    inlier_mask = ransac.inlier_mask_
    outlier_mask = ~inlier_mask

    g = ground[inlier_mask]
    ng = np.concatenate([ground[outlier_mask], nonground])

    pcd_g = o3d.geometry.PointCloud()
    pcd_g.points = o3d.utility.Vector3dVector(g)
    pcd_g.paint_uniform_color([1, 0, 0])

    pcd_ng = o3d.geometry.PointCloud()
    pcd_ng.points = o3d.utility.Vector3dVector(ng)
    pcd_ng.paint_uniform_color([0.7, 0.7, 0.7])

    # pcd_g = pcd_g.voxel_down_sample(0.05)
    # pcd_ng = pcd_ng.voxel_down_sample(0.1)

    o3d.visualization.draw_geometries([pcd_ng, pcd_g])

if __name__ == '__main__':
    main()