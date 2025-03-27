import open3d as o3d
import numpy as np

def pointcloud_to_voxelgrid(pcd, voxel_size):
    # Convert to Open3D PointCloud if input is a NumPy array or list
    if isinstance(pcd, (np.ndarray, list)):
        pcd_np = np.asarray(pcd)
        if pcd_np.ndim != 2 or pcd_np.shape[1] != 3:
            raise ValueError("Input array must be of shape (N, 3)")
        pcd_converted = o3d.geometry.PointCloud()
        pcd_converted.points = o3d.utility.Vector3dVector(pcd_np)
        pcd = pcd_converted
    elif not isinstance(pcd, o3d.geometry.PointCloud):
        raise TypeError("Input must be an Open3D PointCloud, NumPy array, or list")

    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=voxel_size)
    #o3d.visualization.draw_geometries([voxel_grid])
    # centers = np.asarray([
    #     voxel_grid.get_voxel_center_coordinate(voxel.grid_index)
    #     for voxel in voxel_grid.get_voxels()
    # ])
    return voxel_grid

def voxelgrid_to_mesh(voxel_grid):
    voxel_size = voxel_grid.voxel_size
    mesh = o3d.geometry.TriangleMesh()

    for voxel in voxel_grid.get_voxels():
        voxel_center = voxel_grid.get_voxel_center_coordinate(voxel.grid_index)
        cube = o3d.geometry.TriangleMesh.create_box(width=voxel_size, height=voxel_size, depth=voxel_size)
        cube.translate(voxel_center - np.array([voxel_size, voxel_size, voxel_size]) / 2)
        mesh += cube

    #mesh.compute_vertex_normals()
    return mesh

def mesh_file_to_world_config_dict(mesh_path):
    world_config_dict = {
        "mesh": {
            "obs_mesh": {
                "file_path": mesh_path,
                "pose": np.asarray([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]),
                "scale": 1.0,
            },
        }
    } 
    return world_config_dict

# def voxels_to_world_config_dict(voxel_centers, voxel_size):
#     """
#     Create a world_config_dict with a cuboid entry for each voxel center.

#     Args:
#         voxel_centers (np.ndarray): Nx3 array of voxel centers.
#         voxel_size (float or list/tuple): Size of the voxel cuboid. Can be scalar or [x, y, z].

#     Returns:
#         dict: A world_config_dict with each voxel as a named cuboid entry.
#     """
#     if isinstance(voxel_size, (int, float)):
#         dims = [voxel_size] * 3
#     else:
#         dims = list(voxel_size)

#     cuboids = {}
#     for i, center in enumerate(voxel_centers):
#         name = f"cuboid_{i+1}"
#         pose = np.concatenate([center, [1.0, 0.0, 0.0, 0.0]])  # identity quaternion
#         cuboids[name] = {
#             "dims": dims,
#             "pose": pose,
#         }

#     world_config_dict = {"cuboid": cuboids}
#     return world_config_dict

if __name__=='__main__':
    import timeit
    N = 10000
    pcd = o3d.io.read_triangle_mesh(o3d.data.BunnyMesh().path).sample_points_poisson_disk(N)

    # fit to unit cube
    pcd.scale(1 / np.max(pcd.get_max_bound() - pcd.get_min_bound()), center=pcd.get_center())
    pcd.colors = o3d.utility.Vector3dVector(np.random.uniform(0, 1, size=(N, 3)))
    o3d.visualization.draw_geometries([pcd])

    voxel_size = 0.1
    wrapped = lambda: pointcloud_to_voxels(pcd, voxel_size=voxel_size) 
    time = timeit.timeit(wrapped, number=1)

    print(f"Average time: {time / 1:.6f} seconds")
