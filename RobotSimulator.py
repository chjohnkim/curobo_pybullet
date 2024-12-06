import pybullet as p
import pybullet_data
import time
import threading

class RobotSimulator:
    def __init__(self, urdf_path):
        # Connect to PyBullet GUI
        _physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        # Load environment 
        plane_id = p.loadURDF("plane.urdf")
        
        # Load robot from URDF
        self.robot_id = p.loadURDF(urdf_path, useFixedBase=1, basePosition=[0, 0, 1])
        self._get_active_joints()
        self.target_position = [0]*len(self.active_joints)
        
        # Flag to control the simulation loop
        self.running = False

    def _get_active_joints(self):
        num_joints = p.getNumJoints(self.robot_id)
        print(f"Total number of joints: {num_joints}")

        self.active_joints = []
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            if joint_info[2] == p.JOINT_REVOLUTE:
                self.active_joints.append(i)
        print(f"Active joints: {self.active_joints}")

    def simulate(self):
        self.running = True
        while self.running:
            for i, joint_index in enumerate(self.active_joints):
                p.setJointMotorControl2(self.robot_id, joint_index, p.POSITION_CONTROL, targetPosition=self.target_position[i])
            p.stepSimulation()
            time.sleep(1./240.)

    def start_simulation(self):
        self.sim_thread = threading.Thread(target=self.simulate)
        self.sim_thread.start()

    def stop_simulation(self):
        self.running = False
        if hasattr(self, 'sim_thread'):
            self.sim_thread.join()

    def set_target_position(self, new_position):
        if len(new_position) == len(self.active_joints):
            self.target_position = new_position
        else:
            print("Error: Invalid target position length")

    def load_object(self, urdf_path, position=[0, 0, 1], orientation=[0, 0, 0], fix_base=1):
        """Load an object into the simulation."""
        object_id = p.loadURDF(urdf_path, 
                               basePosition=position, 
                               baseOrientation=p.getQuaternionFromEuler(orientation), 
                               useFixedBase=fix_base)
        print(f"Loaded object ID: {object_id}")
        return object_id

    def unload_object(self, object_id):
        """Remove an object from the simulation."""
        p.removeBody(object_id)
        print(f"Unloaded object ID: {object_id}")

    def close(self):
        self.stop_simulation()
        p.disconnect()

    def get_joint_state(self):
        """Returns the state of the robot."""
        joint_states = {
            'position': [],
            'velocity': [],
            'reaction_forces': [],
            'torque': [],
        }
        for i in self.active_joints:
            state = p.getJointState(self.robot_id, i)
            joint_states['position'].append(state[0])
            joint_states['velocity'].append(state[1])
            joint_states['reaction_forces'].append(state[2])
            joint_states['torque'].append(state[3])
        return joint_states

if __name__=='__main__':
    urdf_path = "./ur_description/ur5e.urdf"
    sim = RobotSimulator(urdf_path)

    # Start the simulation in a background thread
    sim.start_simulation()

    # Example of changing target positions and loading/unloading objects
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