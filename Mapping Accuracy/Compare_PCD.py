import open3d as o3d
import numpy as np

def load_and_preprocess_pcd(file_path):
    # Load the point cloud
    pcd = o3d.io.read_point_cloud(file_path)
    # Downsample the point cloud for faster processing
    pcd = pcd.voxel_down_sample(voxel_size=0.05)
    # Estimate normals (helpful for alignment)
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    return pcd

def align_and_compare_pcds(source_file, target_file):
    # Load and preprocess the point clouds
    source_pcd = load_and_preprocess_pcd(source_file)
    target_pcd = load_and_preprocess_pcd(target_file)

    # Perform initial alignment with the global registration
    threshold = 0.1  # Distance threshold for ICP
    transformation_init = np.identity(4)
    
    # Apply ICP for point cloud alignment
    icp_result = o3d.pipelines.registration.registration_icp(
        source_pcd, target_pcd, threshold, transformation_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )

    print("Transformation matrix:\n", icp_result.transformation)
    print("Fitness (alignment quality):", icp_result.fitness)
    print("RMSE (root mean square error):", icp_result.inlier_rmse)

    # Visualize the aligned point clouds
    source_pcd.transform(icp_result.transformation)
    o3d.visualization.draw_geometries([source_pcd.paint_uniform_color([1, 0, 0]), 
                                       target_pcd.paint_uniform_color([0, 1, 0])])

    # Compute average distance between aligned point clouds
    distances = source_pcd.compute_point_cloud_distance(target_pcd)
    avg_distance = np.mean(distances)
    print("Average distance between point clouds:", avg_distance)

# Replace 'source.pcd' and 'target.pcd' with your file paths
align_and_compare_pcds("/home/nithish8055/FLOAM.pcd", "/home/nithish8055/ISCLOAM.pcd")

