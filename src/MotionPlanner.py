# Third Party
import torch
import time

# CuRobo
from curobo.types.math import Pose
from curobo.types.robot import JointState
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from curobo.geom.types import WorldConfig
from src.utils import color_print

class MotionPlanner:
    def __init__(self, robot_yml="ur5e.yml"):
        world_config_placeholder = {
            "cuboid": {
                "placeholder": {
                    "dims": [5.0, 5.0, 0.2],  # x, y, z
                    "pose": [0.0, 0.0, -0.1, 1, 0, 0, 0.0],  # x, y, z, qw, qx, qy, qz
                },
            },
        }

        # Load Robot Config
        t = time.time()
        motion_gen_config = MotionGenConfig.load_from_robot_config(
            robot_yml,
            world_config_placeholder,
            collision_cache={"obb": 10, "mesh": 10},
            interpolation_dt=0.01,
        )
        color_print(f'Loaded motion gen config: {time.time()-t:.3f}s', 
                    fg='yellow', style='bold')

        self.motion_gen = MotionGen(motion_gen_config)
        # Warm up    
        t = time.time()
        self.motion_gen.warmup()
        color_print(f'Warmup: {time.time()-t:.3f}s\nPlanner loaded.', 
                    fg='yellow', style='bold')


        #retract_cfg = motion_gen.get_retract_config()
        
        #state = motion_gen.rollout_fn.compute_kinematics(
        #    JointState.from_position(retract_cfg.view(1, -1))
        #)

    def plan(self, start_joint_state, target_pose):
        """
        start_joint_state [List]: Base -> Wrist
        target_pose [List]: x, y, z, qw, qx, qy, qz
        """
        t = time.time()
        goal_pose = Pose.from_list(target_pose)  
        start_joint_state = torch.tensor(start_joint_state, dtype=torch.float32).reshape(1, -1).cuda()
        start_state = JointState.from_position(
            start_joint_state,
            joint_names=[
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
        )
        result = self.motion_gen.plan_single(start_state, goal_pose, MotionGenPlanConfig(max_attempts=60))        
        #traj = result.get_interpolated_plan()  # result.interpolation_dt has the dt between timesteps
        success = result.success.detach().cpu().item()
        color_print(f'Planning Success: {success} | Planning time: {time.time()-t:.3f}s | Status: {result.status}', 
                    fg='green' if success else 'red', style='bold')
        return result, success

    def update_world(self, world_config_dict):
        t = time.time()
        self.motion_gen.clear_world_cache()
        # TODO: Implement type checking here
        world_config = WorldConfig.from_dict(world_config_dict)
        self.motion_gen.update_world(world_config)
        color_print(f'Updated world config: {time.time()-t:.3f}s', 
                    fg='yellow', style='bold')

if __name__ == "__main__":
    world_config = {
        #"mesh": {
        #    "base_scene": {
        #        "pose": [10.5, 0.080, 1.6, 0.043, -0.471, 0.284, 0.834],
        #        "file_path": "scene/nvblox/srl_ur10_bins.obj",
        #    },
        #},
        "cuboid": {
            "table": {
                "dims": [1.0, 1.0, 0.2],  # x, y, z
                "pose": [0.0, 1.0, -0.1, 1, 0, 0, 0.0],  # x, y, z, qw, qx, qy, qz
            },
        },
    }
    mp = MotionPlanner()
    #mp.update_world(world_config)
    start_joint_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    target_pose = [-0.4, 0.0, 0.4, 1.0, 0.0, 0.0, 0.0]
    mp.plan(start_joint_state, target_pose)