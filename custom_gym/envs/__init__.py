from custom_gym.envs.registration import (
    registry,
    register,
    make,
    spec,
    load_env_plugins as _load_env_plugins,
)

# Hook to load plugins from entry points
_load_env_plugins()


register(
    id="Ant-v2",
    entry_point="custom_gym.envs.mujoco:AntEnv",
    max_episode_steps=1000,
    reward_threshold=6000.0,
)

register(
    id="Ant-v3",
    entry_point="custom_gym.envs.mujoco.ant_v3:AntEnv",
    max_episode_steps=1000,
    reward_threshold=6000.0,
)

## Gym-robotics 없이 그냥 Gym으로 통합 ##
register(
    id=f"Robot-v1",
    entry_point="custom_gym.envs.mujoco.robot_env:RobotEnv",
    max_episode_steps=1000,
    reward_threshold=6000.0,
)

# Fetch
register(
    id=f"Fetch-v1",
    entry_point="custom_gym.envs.mujoco.fetch.fetch:FetchEnv",
    max_episode_steps=1000,
    reward_threshold=6000.0,
)

register(
    id=f"FetchSlide-v1",
    entry_point="custom_gym.envs.mujoco.fetch.slide:FetchSlideEnv",
    max_episode_steps=1000,
    reward_threshold=6000.0,
)

register(
    id=f"FetchPickAndPlace-v1",
    entry_point="custom_gym.envs.mujoco.fetch.pick_and_place:FetchPickAndPlaceEnv",
    max_episode_steps=1000,
    reward_threshold=6000.0,
)

register(
    id=f"FetchReach-v1",
    entry_point="custom_gym.envs.mujoco.fetch.reach:FetchReachEnv",
    max_episode_steps=1000,
    reward_threshold=6000.0,
)

register(
    id=f"FetchPush-v1",
    entry_point="custom_gym.envs.mujoco.fetch.push:FetchPushEnv",
    max_episode_steps=1000,
    reward_threshold=6000.0,
)

register(
    id=f"FetchAnt-v1",
    entry_point="custom_gym.envs.mujoco.fetch.ant_fetch:FetchAntEnv",
    max_episode_steps=1000,
    reward_threshold=6000.0,
)