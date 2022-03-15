import os
from gym import utils
from custom_gym.envs.mujoco import custom_env


# Ensure we get the path separator correct on windows
MODEL_XML_PATH = os.path.join("fetch", "fetch_ant.xml")


class FetchAntEnv(custom_env.CustomEnv, utils.EzPickle):
    def __init__(self, reward_type="sparse"):
        initial_qpos = {
            "robot0:slide0": 0.405,
            "robot0:slide1": 0.48,
            "robot0:slide2": 0.0,
        }
        custom_env.CustomEnv.__init__(
            self,
            MODEL_XML_PATH,
            initial_qpos=initial_qpos,
            n_substeps=20,
            gripper_extra_height=0.2,
            contact_force_range= (-1.0, 1.0),
            block_gripper=True,
            target_range=0.15,
            exclude_current_positions_from_observation=True,
            distance_threshold=0.05,
            reward_type=reward_type,
        )

        utils.EzPickle.__init__(self, reward_type=reward_type,)


