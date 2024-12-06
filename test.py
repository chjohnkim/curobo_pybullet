from RobotSimulator import RobotSimulator
from MotionPlanner import MotionPlanner
import time 

if __name__=='__main__':
    urdf_path = "./ur_description/ur5e.urdf"
    sim = RobotSimulator(urdf_path)
    planner = MotionPlanner()
    # Start the simulation in a background thread
    sim.start_simulation()
    js = sim.get_joint_state()
    
    result, success = planner.plan(js['position'], [-0.4, 0.0, 0.4, 1.0, 0.0, 0.0, 0.0])
    dt = result.interpolation_dt
    joint_waypoints = result.get_interpolated_plan().position
    joint_waypoints = joint_waypoints.detach().cpu().numpy()

    try:
        for waypoint in joint_waypoints:
            # Change target positions periodically
            sim.set_target_position(waypoint)
            time.sleep(dt)

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