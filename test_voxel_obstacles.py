from RobotSimulator import RobotSimulator
import time 
import numpy as np
import open3d as o3d
from utils import pointcloud_to_voxelgrid, voxelgrid_to_mesh, \
                mesh_file_to_world_config_dict
from copy import deepcopy
import tempfile
import open3d as o3d

world_config_dict = {}

if __name__=='__main__':
    # Load bunny pcd
    N = 10000
    pcd = o3d.io.read_triangle_mesh(o3d.data.BunnyMesh().path).sample_points_poisson_disk(N)
    # fit to unit cube
    pcd.scale(0.5 / np.max(pcd.get_max_bound() - pcd.get_min_bound()), center=pcd.get_center())
    pcd.colors = o3d.utility.Vector3dVector(np.random.uniform(0, 1, size=(N, 3)))
    voxel_size = 0.01

    urdf_path = "./ur_description/ur5e.urdf"
    sim = RobotSimulator(urdf_path)

    # Initialize the simulation in a background thread
    sim.start_simulation()
    sim.set_target_position([0, -1, 1, -1, -1.57, 0])
    time.sleep(1)

    try:
        for i in range(10):
            js = sim.get_joint_state()
            t0 = time.time()
            sim.update_world(world_config_dict)
            print('update world:', time.time()-t0)
            pcd_copy = deepcopy(pcd) # Make a copy of pcd
            # Sample separately from each axis range
            dx = np.random.uniform(low=-0.5, high=0.5)
            dy = np.random.uniform(low=-0.5, high=0.5)
            dz = np.random.uniform(low=0.0, high=0.8)
            pcd_copy.translate(np.array([dx, dy, dz]))
            t0 = time.time()
            voxel_grid = pointcloud_to_voxelgrid(pcd_copy, voxel_size)
            print('pointcloud_to_voxelgrid:', time.time()-t0)
            t0 = time.time()
            mesh = voxelgrid_to_mesh(voxel_grid)
            print('voxelgrid_to_mesh:', time.time()-t0)
            t0 = time.time()
            with tempfile.NamedTemporaryFile(suffix=".obj", delete=False) as tmp_file:
                mesh_path = tmp_file.name
                o3d.io.write_triangle_mesh(mesh_path, mesh)
            print('temp_file:', time.time()-t0)
            print()
            world_config_dict = mesh_file_to_world_config_dict(mesh_path)
            time.sleep(1)

    except KeyboardInterrupt:
        print("Stopping simulation...")
    finally:
        sim.close()