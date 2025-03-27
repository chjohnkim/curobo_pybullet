import numpy as np
import click
import os
import threading
from scipy.spatial.transform import Rotation as R
from ur5_controller import URControl as Robot
from keyboard_handler import KeyboardHandler
from RobotSimulator import RobotSimulator
import time

@click.command()
@click.option('-h', '--hostname', type=str, default='192.168.1.10')
@click.option('-r', '--control_rate_hz', type=int, default=10)
def main(hostname, control_rate_hz):
    control_period = 1.0/control_rate_hz
    urdf_path = "./ur_description/ur5e.urdf"
    sim = RobotSimulator(urdf_path)
    # Initialize the simulation in a background thread
    sim.start_simulation()

    with KeyboardHandler() as keyboard_listener, Robot(hostname) as robot:
        print("Press 'c' to capture a snapshot or 'q' to quit.")
        while True:
            loop_start_time = time.time()
            sim.set_target_position(list(robot.get_q()))

            if keyboard_listener.action == 'capture':
                # Get robot pose: x, y, z, qx, qy, qz
                tcp_pose = robot.get_tcp_pose()
                translation = tcp_pose[:3]
                rot_vec = tcp_pose[3:]
                rot = R.from_rotvec(rot_vec)
                quaternion = rot.as_quat()
                pose = np.concatenate([translation, quaternion]).reshape(1, -1)
                print("Captured a snapshot!")  # Replace with saving/processing logic
                keyboard_listener.action = None  # Reset action
            
            elif keyboard_listener.action == 'freedrive':
                robot.activate_teach_mode()
                print(keyboard_listener.action)
                keyboard_listener.action = None  # Reset action
            
            elif keyboard_listener.action == 'stop_freedrive':
                robot.deactivate_teach_mode()
                print(keyboard_listener.action)
                keyboard_listener.action = None  # Reset action

            elif keyboard_listener.action == 'quit':
                print("Exiting...")
                break
            # Other loop operations, e.g., robot control, monitoring, etc.
            # Enforce control rate
            elapsed = time.time() - loop_start_time
            time.sleep(max(0.0, control_period - elapsed))
    sim.close()

if __name__ == "__main__":
    main()