from legged_gym import LEGGED_GYM_ROOT_DIR, LEGGED_GYM_ENVS_DIR

from legged_gym.envs.go1.go1_config import GO1FlatCfg, GO1FlatCfgPPO
from legged_gym.envs.go2.go2_config import GO2FlatCfg, GO2FlatCfgPPO
from legged_gym.envs.go2.go2_config import GO2RoughCfg, GO2RoughCfgPPO
from legged_gym.envs.aliengo.aliengo_config import ALIENGOFlatCfg, ALIENGOFlatCfgPPO
from legged_gym.envs.h1.h1_config import H1RoughCfg, H1RoughCfgPPO
from legged_gym.envs.h1.h1_env import H1Robot
from legged_gym.envs.h1_2.h1_2_config import H1_2RoughCfg, H1_2RoughCfgPPO
from legged_gym.envs.h1_2.h1_2_env import H1_2Robot
from legged_gym.envs.g1.g1_config import G1RoughCfg, G1RoughCfgPPO
from legged_gym.envs.g1.g1_env import G1Robot
from legged_gym.envs.aliengo.aliengo_env import AlIENGORobot
from .base.legged_robot import LeggedRobot

from legged_gym.utils.task_registry import task_registry

task_registry.register("aliengo", AlIENGORobot, ALIENGOFlatCfg(), ALIENGOFlatCfgPPO())
task_registry.register("go1", LeggedRobot, GO1FlatCfg(), GO1FlatCfgPPO())
task_registry.register("go2", LeggedRobot, GO2FlatCfg(), GO2FlatCfgPPO())
task_registry.register("rough_go2", LeggedRobot, GO2RoughCfg(), GO2RoughCfgPPO())
task_registry.register("h1", H1Robot, H1RoughCfg(), H1RoughCfgPPO())
task_registry.register("h1_2", H1_2Robot, H1_2RoughCfg(), H1_2RoughCfgPPO())
task_registry.register("g1", G1Robot, G1RoughCfg(), G1RoughCfgPPO())
