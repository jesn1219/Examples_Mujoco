import numpy as np
from custom_gym.envs.mujoco import rotations, robot_env, utils
from custom_gym.envs.mujoco import mujoco_env


def goal_distance(goal_a, goal_b):
    assert goal_a.shape == goal_b.shape
    return np.linalg.norm(goal_a - goal_b, axis=-1)


class CustomEnv(robot_env.RobotEnv):
    """Superclass for all Fetch environments."""
    # model_path, initial_qpos, n_actions, n_substeps: RobotEnv

    def __init__(
        self,
        model_path,
        initial_qpos,
        n_substeps,
        gripper_extra_height,
        contact_force_range,
        block_gripper,
        target_range,
        exclude_current_positions_from_observation,
        distance_threshold,
        reward_type,
        #has_object
        #target_in_the_air
        #target_o
    ):

        self.gripper_extra_height = gripper_extra_height
        self.block_gripper = block_gripper
        self.target_range = target_range
        self.contact_force_range = contact_force_range
        self.distance_threshold = distance_threshold
        self.reward_type = reward_type
        self.exclude_current_positions_from_observation = (
            exclude_current_positions_from_observation
        )


        super(CustomEnv, self).__init__(
            model_path=model_path,
            n_substeps=n_substeps,
            n_actions=6, # gripper(4) + fetch pos(2)
            initial_qpos=initial_qpos,
        )

    def compute_reward(self, achieved_goal, goal, info):
        # Compute distance between goal and the achieved goal.
        d = goal_distance(achieved_goal, goal)
        if self.reward_type == "sparse":
            return -(d > self.distance_threshold).astype(np.float32)
        else:
            return -d

    def _step_callback(self):
        # Fetch 이동
        self.sim.data.set_joint_qpos("robot0:slide0", self.sim.data.ctrl[2])
        self.sim.data.set_joint_qpos("robot0:slide1", self.sim.data.ctrl[3])
        # 개미 이동    
        self.sim.data.set_joint_qpos("hip_4", self.sim.data.get_joint_qpos("hip_4") + np.random.uniform(low=0.1, high=0.2, size=1))
        self.sim.data.set_joint_qpos("ankle_4", self.sim.data.get_joint_qpos("ankle_4") + np.random.uniform(low=0.1, high=0.2, size=1))
        self.sim.data.set_joint_qpos("hip_1", self.sim.data.get_joint_qpos("hip_1") + np.random.uniform(low=0.1, high=0.2, size=1))
        self.sim.data.set_joint_qpos("ankle_1", self.sim.data.get_joint_qpos("ankle_1") + np.random.uniform(low=0.1, high=0.2, size=1))
        self.sim.data.set_joint_qpos("hip_2", self.sim.data.get_joint_qpos("hip_2") + np.random.uniform(low=0.1, high=0.2, size=1))
        self.sim.data.set_joint_qpos("ankle_2", self.sim.data.get_joint_qpos("ankle_2") + np.random.uniform(low=0.1, high=0.2, size=1))
        self.sim.data.set_joint_qpos("hip_3", self.sim.data.get_joint_qpos("hip_3") + np.random.uniform(low=0.1, high=0.2, size=1))
        self.sim.data.set_joint_qpos("ankle_3", self.sim.data.get_joint_qpos("ankle_3") + np.random.uniform(low=0.1, high=0.2, size=1))
        self.sim.forward()

    def _set_action(self, action):
        assert action.shape == (6,) # 현재는 랜덤하게 입력 받는 중  
        action = (
            action.copy()
        )  # ensure that we don't change the action outside of this scope
        
        pos_ctrl, gripper_ctrl, fetch_pos = action[:3], action[3], action[4:6]
        pos_ctrl *= 0.05  # limit maximum change in position
        fetch_pos *= 0.08
        rot_ctrl = [
            1.0,
            0.0,
            1.0,
            0.0,
        ]  # fixed rotation of the end effector, expressed as a quaternion
        gripper_ctrl = np.array([gripper_ctrl, gripper_ctrl]) # gripper가 양쪽이므로

        assert gripper_ctrl.shape == (2,)
        if self.block_gripper:
            gripper_ctrl = np.zeros_like(gripper_ctrl)

        action = np.concatenate([pos_ctrl, rot_ctrl, gripper_ctrl, fetch_pos])

        # Apply action to simulation.        self.sim.data.ctrl[:] = ctrl
        utils.ctrl_set_action(self.sim, action)
        utils.mocap_set_action(self.sim, action) # pos_ctrl, rot_ctrl 

    def _get_obs(self):
        # Fetch obs
        ant_pos = self.sim.data.get_site_xpos("ant0:torso")
        dt = self.sim.nsubsteps * self.sim.model.opt.timestep
        ant_velp = self.sim.data.get_site_xvelp("ant0:torso") * dt
        robot_qpos, robot_qvel = utils.robot_get_obs(self.sim)
        
        gripper_state = robot_qpos[-2:]
        gripper_vel = (
            robot_qvel[-2:] * dt
        )  # change to a scalar if the gripper is made symmetric

        achieved_goal = ant_pos.copy()
        
        # Ant obs
        position = self.sim.data.qpos.flat.copy()
        velocity = self.sim.data.qvel.flat.copy()
        contact_force = self.contact_forces.flat.copy()

        if self.exclude_current_positions_from_observation:
            position = position[2:]

        obs = np.concatenate(
            [
                ant_pos,
                gripper_state,
                ant_velp,
                gripper_vel,
                position,
                velocity,
                contact_force
            ]
        )

        return {
            "observation": obs.copy(),
            "achieved_goal": achieved_goal.copy(),
            "desired_goal": self.goal.copy(),
        }

    '''
    def compute_reward(self, achieved_goal, goal, info):
    '''

    def _viewer_setup(self):
        body_id = self.sim.model.body_name2id("robot0:gripper_link")
        lookat = self.sim.data.body_xpos[body_id]
        for idx, value in enumerate(lookat):
            self.viewer.cam.lookat[idx] = value
        self.viewer.cam.distance = 2.5
        self.viewer.cam.azimuth = 132.0
        self.viewer.cam.elevation = -14.0

    def _sample_goal(self):
        goal = self.initial_ant_pos[:3] + self.np_random.uniform(
                -self.target_range, self.target_range, size=3
            )
        return goal.copy()

    def _render_callback(self):
        # Visualize target.
        sites_offset = (self.sim.data.site_xpos - self.sim.model.site_pos).copy()
        site_id = self.sim.model.body_name2id("target0")
        self.sim.model.site_pos[site_id] = self.goal - sites_offset[0]
        self.sim.forward()

    def _reset_sim(self):
        self.sim.set_state(self.initial_state)
        self.sim.forward()
        return True

    def _is_success(self, achieved_goal, desired_goal):
        d = goal_distance(achieved_goal, desired_goal)
        return (d < self.distance_threshold).astype(np.float32)


    def _env_setup(self, initial_qpos):
        for name, value in initial_qpos.items():
            self.sim.data.set_joint_qpos(name, value)
        utils.reset_mocap_welds(self.sim)
        self.sim.forward()

        # Move end effector into position.
        gripper_target = np.array(
            [-0.498, 0.005, -0.431 + self.gripper_extra_height]
        ) + self.sim.data.get_site_xpos("ant0:torso")
        gripper_rotation = np.array([1.0, 0.0, 1.0, 0.0])
        robot_xpos = self.sim.data.get_body_xpos("robot0:base_link")
        self.sim.data.set_mocap_pos("robot0:mocap", gripper_target)
        self.sim.data.set_mocap_quat("robot0:mocap", gripper_rotation)
        for _ in range(10):
            self.sim.step()

        # Extract information for sampling goals.
        self.initial_ant_pos = self.sim.data.get_site_xpos("ant0:torso").copy()

    def render(self, mode="human", width=500, height=500):
        return super(CustomEnv, self).render(mode, width, height)

    
    @property
    def contact_forces(self):
        raw_contact_forces = self.sim.data.cfrc_ext
        min_value, max_value = self.contact_force_range
        contact_forces = np.clip(raw_contact_forces, min_value, max_value)
        return contact_forces