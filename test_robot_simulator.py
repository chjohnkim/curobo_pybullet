from RobotSimulator import RobotSimulator
from MotionPlanner import MotionPlanner
from curobo.geom.types import WorldConfig, Sphere, Cuboid
import time 
import numpy as np
"""
obstacle_config = {
    "sphere": {
        "sphere_1": {
            "radius": 0.2, 
            "pose": [0.0, 0.6, 0.4, 1, 0, 0, 0.0],  # x, y, z, qw, qx, qy, qz
        },
        "sphere_2": {
            "radius": 0.2, 
            "pose": [0.0, 0.6, -0.4, 1, 0, 0, 0.0],  # x, y, z, qw, qx, qy, qz
        },

    },
}
"""

world_config_dict = {
    "mesh": {
        "tree_1": {
            "file_path": "/home/johnkim/projects/curobo_pybullet/palm_tree/palm_tree.obj",
            "pose": np.asarray([1.0, 0.0, 0, 1.0, 0.0, 0.0, 0.0]),
            "scale": 0.001,
        },
    },    
    "cuboid": {
        "cuboid_1": {
            "dims": [2.0, 2.0, 0.2],  # x, y, z
            "pose": np.asarray([0.0, 0.0, -0.1, 1, 0, 0, 0.0]),  # x, y, z, qw, qx, qy, qz
        },
    },
}
'''
curobo_obstacles = []
for key, value in obstacle_config["cuboid"].items():
    curobo_obstacles.append(Cuboid(name=key, dims=value["dims"], pose=value["pose"]))
world_model = WorldConfig(
   #mesh=[obstacle_2],
   cuboid=curobo_obstacles,
   #capsule=[obstacle_3],
   #cylinder=[obstacle_4],
   #sphere=curobo_obstacles,
)
'''
#collision_supported_world = WorldConfig.create_collision_support_world(world_model)

target_poses = [
    [-0.4, 0.0, 0.3, 0.0, 1.0, 0.0, 0.0],
    [ 0.4, 0.0, 0.3, 0.0, 1.0, 0.0, 0.0],
    [-0.4, 0.0, 0.3, 0.0, 1.0, 0.0, 0.0],
    [ 0.4, 0.0, 0.3, 0.0, 1.0, 0.0, 0.0],
    [-0.4, 0.0, 0.3, 0.0, 1.0, 0.0, 0.0],
    [ 0.4, 0.0, 0.3, 0.0, 1.0, 0.0, 0.0],
    [-0.4, 0.0, 0.3, 0.0, 1.0, 0.0, 0.0],
    [ 0.4, 0.0, 0.3, 0.0, 1.0, 0.0, 0.0],
    [-0.4, 0.0, 0.3, 0.0, 1.0, 0.0, 0.0],
    [ 0.4, 0.0, 0.3, 0.0, 1.0, 0.0, 0.0],
    [-0.4, 0.0, 0.3, 0.0, 1.0, 0.0, 0.0],
    [ 0.4, 0.0, 0.3, 0.0, 1.0, 0.0, 0.0],
    [-0.4, 0.0, 0.3, 0.0, 1.0, 0.0, 0.0],
    [ 0.4, 0.0, 0.3, 0.0, 1.0, 0.0, 0.0],
    [-0.4, 0.0, 0.3, 0.0, 1.0, 0.0, 0.0],
    [ 0.4, 0.0, 0.3, 0.0, 1.0, 0.0, 0.0],
]
if __name__=='__main__':
    urdf_path = "./ur_description/ur5e.urdf"
    sim = RobotSimulator(urdf_path)
    # planner = MotionPlanner()

    # Initialize the simulation in a background thread
    sim.start_simulation()
    sim.set_target_position([0, -1, 1, -1, -1.57, 0])
    time.sleep(1)

    try:
        for i in range(100):
            js = sim.get_joint_state()
            sim.update_world(world_config_dict)
            # planner.update_world(world_config_dict)
            # result, success = planner.plan(js['position'], target_pose)
            # if success:
            #     joint_waypoints = result.get_interpolated_plan().position
            #     joint_waypoints = joint_waypoints.detach().cpu().numpy()
            #     for waypoint in joint_waypoints:
            #         t_start = time.time()
            #         # Change target positions periodically
            #         sim.set_target_position(waypoint)
            #         t_elapsed = time.time()-t_start  
            #         if t_elapsed < result.interpolation_dt:
            #             time.sleep(result.interpolation_dt - t_elapsed)
            for key, value in world_config_dict["mesh"].items():
                radius = np.random.uniform(0.4, 0.6)
                theta = np.random.uniform(0, 2*np.pi)
                world_config_dict["mesh"][key]["pose"][:2] = np.array([radius*np.cos(theta), radius*np.sin(theta)])
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Stopping simulation...")
    finally:
        sim.close()

    """
    try:
        while True:
            # Change target positions periodically
            sim.set_target_position([0, -1, 1, -1, -1.57, 0])
            time.sleep(3)
            sim.set_target_position([0.5, -0.5, 0.5, -0.5, -1.57, 0.5])
            time.sleep(3)

            # Load an object (example: a cube)
            cube_id = sim.load_object("cube.urdf", position=[0.5, 0.5, 1])

            time.sleep(3)  # Keep the cube for a while

            # Unload the object
            sim.unload_object(cube_id)

            time.sleep(3)  # Wait before changing positions again

    except KeyboardInterrupt:
        print("Stopping simulation...")
    finally:
        sim.close()
    """