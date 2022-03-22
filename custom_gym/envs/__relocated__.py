from typing import Dict, Tuple

# The following is a map of environments which have been relocated
# to a different namespace. This map is important when reporting
# new versions of an environment outside of Gym.
# This map should be removed eventually once users
# are sufficiently aware of the environment relocations.
# The value of the mapping is (namespace, package,).
internal_env_relocation_map: Dict[str, Tuple[str, str]] = {
    #"FetchSlide": (None, "gym-robotics"),
    #"FetchPickAndPlace": (None, "gym-robotics"),
    #"FetchReach": (None, "gym-robotics"),
    #"FetchPush": (None, "gym-robotics"),
    #"FetchSlideDense": (None, "gym-robotics"),
    #"FetchPickAndPlaceDense": (None, "gym-robotics"),
    #"FetchReachDense": (None, "gym-robotics"),
    #"FetchPushDense": (None, "gym-robotics"),
}
    
