from src.RobotSimulator import RobotSimulator
from src.MotionPlanner import MotionPlanner
import time 
import numpy as np
import open3d as o3d
from src.utils import pointcloud_to_voxelgrid, voxelgrid_to_mesh, mesh_file_to_world_config_dict
from copy import deepcopy
import tempfile
from scipy.spatial.transform import Rotation as R
import random
import os

if __name__=='__main__':

    # Create a straight line point cloud along the z-axis
    num_points = 100
    z = np.linspace(0.3, 0.6, num_points)  # 0 to 1 in z
    points = np.zeros((num_points, 3))
    points[:, 2] = z  # only z changes
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    voxel_size = 0.01

    urdf_path = os.path.abspath("./assets/ur_description/ur5e.urdf")
    robot_yml = os.path.abspath("./config/ur5e.yml")
    sim = RobotSimulator(urdf_path)
    planner = MotionPlanner(robot_yml)

    # Initialize the simulation in a background thread
    sim.start_simulation()
    sim.set_target_position([0, -1, 1, -1, -3.14, 0])
    time.sleep(1)
    try:
        for i in range(100):
            pcd_copy = deepcopy(pcd) # Make a copy of pcd
            # Randomly sample spawn location of stick
            radius = np.random.uniform(0.3, 0.3)
            theta = np.random.uniform(0, 2*np.pi)
            pcd_copy.translate(np.array([radius*np.cos(theta), radius*np.sin(theta), 0]))
            # Construct world_config_dict
            voxel_grid = pointcloud_to_voxelgrid(pcd_copy, voxel_size)
            mesh = voxelgrid_to_mesh(voxel_grid)
            with tempfile.NamedTemporaryFile(suffix=".obj", delete=False) as tmp_file:
                mesh_path = tmp_file.name
                o3d.io.write_triangle_mesh(mesh_path, mesh)
            world_config_dict = mesh_file_to_world_config_dict(mesh_path)

            # Update sim and planner with world_config
            sim.update_world(world_config_dict)
            planner.update_world(world_config_dict)
            
            # Compute target pose
            # Randomly select a point in the point cloud
            points = np.asarray(pcd_copy.points)
            random_index = random.randint(0, len(points) - 1)
            random_point = points[random_index]
            # Get current end-effector pose
            pos, quat_xyzw = sim.get_pose()
            rotation_matrix = R.from_quat(quat_xyzw).as_matrix()
            # End-effector axis
            ee_axis = rotation_matrix[:,2]
            # Target position offset along end-effector axis
            target_position = random_point - ee_axis*0.1
            # Target orientation same as initial
            quat_wxyz = np.concatenate(([quat_xyzw[3]], quat_xyzw[:3]))
            target_pose = target_position.tolist() + quat_wxyz.tolist()
            
            js = sim.get_joint_state()
            result, success = planner.plan(js['position'], target_pose)
            if success:
                joint_waypoints = result.get_interpolated_plan().position
                joint_waypoints = joint_waypoints.detach().cpu().numpy()
                for waypoint in joint_waypoints:
                    t_start = time.time()
                    # Change target positions periodically
                    sim.set_target_position(waypoint)
                    time.sleep(0.02)
                    t_elapsed = time.time()-t_start  
                    if t_elapsed < result.interpolation_dt:
                        time.sleep(result.interpolation_dt - t_elapsed)

    except KeyboardInterrupt:
        print("Stopping simulation...")
    finally:
        sim.close()