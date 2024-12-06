# Third Party
import torch
import time

# CuRobo
from curobo.types.math import Pose
from curobo.types.robot import JointState
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig

class MotionPlanner:
    def __init__(self):
        world_config = {
            #"mesh": {
            #    "base_scene": {
            #        "pose": [10.5, 0.080, 1.6, 0.043, -0.471, 0.284, 0.834],
            #        "file_path": "scene/nvblox/srl_ur10_bins.obj",
            #    },
            #},
            "cuboid": {
                "table": {
                    "dims": [5.0, 5.0, 0.2],  # x, y, z
                    "pose": [0.0, 0.0, -0.1, 1, 0, 0, 0.0],  # x, y, z, qw, qx, qy, qz
                },
            },
        }
        # Load Robot Config
        t = time.time()
        motion_gen_config = MotionGenConfig.load_from_robot_config(
            "ur5e.yml",
            world_config,
            interpolation_dt=0.01,
        )
        print('load_from_robot_config:', time.time()-t)

        self.motion_gen = MotionGen(motion_gen_config)

        # Warm up    
        t = time.time()
        self.motion_gen.warmup()
        print('warmup:', time.time()-t)
        print('Planner loaded')
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
        result = self.motion_gen.plan_single(start_state, goal_pose, MotionGenPlanConfig(max_attempts=1))        
        #traj = result.get_interpolated_plan()  # result.interpolation_dt has the dt between timesteps
        success = result.success.detach().cpu().item()
        print(f'Status: {success} | Planning time: {time.time()-t}')
        return result, success

if __name__ == "__main__":
    mp = MotionPlanner()
    start_joint_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    target_pose = [-0.4, 0.0, 0.4, 1.0, 0.0, 0.0, 0.0]
    mp.plan(start_joint_state, target_pose)