from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class ALIENGOFlatCfg( LeggedRobotCfg ):
    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.6] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            'FL_hip_joint': 0.0,   # [rad]
            'FR_hip_joint': -0.0,  # [rad]
            'RL_hip_joint': 0.0,   # [rad]
            'RR_hip_joint': -0.0,   # [rad]

            'FL_thigh_joint': 1.0,     # [rad]
            'FR_thigh_joint': 1.0,     # [rad]
            'RL_thigh_joint': 1.5,   # [rad]
            'RR_thigh_joint': 1.5,   # [rad]

            'FL_calf_joint': -1.35,   # [rad]
            'FR_calf_joint': -1.35,  # [rad]
            'RL_calf_joint': -1.2,    # [rad]
            'RR_calf_joint': -1.2,    # [rad]
        }

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        control_type = 'P'
        stiffness = {'calf': 40.,'hip': 40, 'thigh': 40}  # [N*m/rad]
        damping = {'calf': 1.0,'hip': 1.0, 'thigh': 1.0}     # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.25
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4

    class asset( LeggedRobotCfg.asset ):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/aliengo/urdf/aliengo.urdf'
        name = "aliengo"
        foot_name = "foot"
        penalize_contacts_on = ["thigh", "calf"]
        terminate_after_contacts_on = ["trunk"]
        self_collisions = 1 # 1 to disable, 0 to enable...bitwise filter
  
    class rewards( LeggedRobotCfg.rewards ):
        soft_dof_pos_limit = 0.9
        base_height_target = 0.43
        class scales( LeggedRobotCfg.rewards.scales ):
            torques = -0.00001
            dof_pos_limits = -10.0
            base_height = -10.0
            orientation = -1.0

class ALIENGOFlatCfgPPO( LeggedRobotCfgPPO ):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01
    class runner( LeggedRobotCfgPPO.runner ):
        max_iterations = 100000 # number of policy updates
        run_name = ''
        experiment_name = 'aliengo'

  