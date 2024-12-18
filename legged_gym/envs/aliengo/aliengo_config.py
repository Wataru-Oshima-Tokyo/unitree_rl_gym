from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class ALIENGOFlatCfg( LeggedRobotCfg ):
    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.45] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            'FL_hip_joint': 0.1,   # [rad]
            'RL_hip_joint': 0.1,   # [rad]
            'FR_hip_joint': -0.1 ,  # [rad]
            'RR_hip_joint': -0.1,   # [rad]

            'FL_thigh_joint': 0.8,     # [rad]
            'RL_thigh_joint': 1.,   # [rad]
            'FR_thigh_joint': 0.8,     # [rad]
            'RR_thigh_joint': 1.,   # [rad]

            'FL_calf_joint': -1.5,   # [rad]
            'RL_calf_joint': -1.5,    # [rad]
            'FR_calf_joint': -1.5,  # [rad]
            'RR_calf_joint': -1.5,    # [rad]
        }

    class domain_rand(LeggedRobotCfg.domain_rand):
        randomize_friction = True
        friction_range = [0.1, 1.25]
        randomize_base_mass = True
        added_mass_range = [-1., 5.]
        push_robots = True
        push_interval_s = 5
        max_push_vel_xy = 1.5

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        control_type = 'P'
        stiffness = {'joint': 20.}  # [N*m/rad]
        damping = {'joint': 0.5}     # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.25
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4

    class asset( LeggedRobotCfg.asset ):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/aliengo/urdf/aliengo.urdf'
        name = "aliengo"
        foot_name = "foot"
        penalize_contacts_on = ["thigh", "calf"]
        terminate_after_contacts_on = ["base"]
        self_collisions = 1 # 1 to disable, 0 to enable...bitwise filter
  
    class rewards( LeggedRobotCfg.rewards ):
        soft_dof_pos_limit = 0.9
        base_height_target = 0.43
        only_positive_rewards = True # if true negative total rewards are clipped at zero (avoids early termination problems)
        class scales( LeggedRobotCfg.rewards.scales ):
            torques = -0.0002
            dof_pos_limits = -10.0
            alive = 0.15
            # contact_no_vel = -0.2
            # hip_pos = -1.0
            # feet_swing_height = -20.0
            # collision = 0.0
            # torques = -0.00001
            # dof_pos_limits = -10.0
            # base_height = -10.0
            # orientation = -1.0
        # class scales( LeggedRobotCfg.rewards.scales ):
        #     # tracking_lin_vel = 1.0
        #     # tracking_ang_vel = 0.5
        #     # lin_vel_z = -2.0
        #     # ang_vel_xy = -0.05
        #     # orientation = -1.0
        #     # base_height = -10.0
        #     # dof_acc = -2.5e-7
        #     # dof_vel = -1e-3
        #     # feet_air_time = 0.0
        #     collision = 0.0
        #     # action_rate = -0.01
        #     # dof_pos_limits = -5.0
        #     # alive = 0.15
        #     # hip_pos = -1.0
        #     # contact_no_vel = -0.2
        #     # feet_swing_height = -20.0
        #     # contact = 0.18

class ALIENGOFlatCfgPPO( LeggedRobotCfgPPO ):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01
    class runner( LeggedRobotCfgPPO.runner ):
        max_iterations = 1500 # number of policy updates
        run_name = ''
        experiment_name = 'aliengo'



class ALIENGORoughCfg( LeggedRobotCfg ):
    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.6] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            'FL_hip_joint': 0.1,   # [rad]
            'RL_hip_joint': 0.1,   # [rad]
            'FR_hip_joint': -0.1 ,  # [rad]
            'RR_hip_joint': -0.1,   # [rad]

            'FL_thigh_joint': 0.8,     # [rad]
            'RL_thigh_joint': 1.,   # [rad]
            'FR_thigh_joint': 0.8,     # [rad]
            'RR_thigh_joint': 1.,   # [rad]

            'FL_calf_joint': -1.5,   # [rad]
            'RL_calf_joint': -1.5,    # [rad]
            'FR_calf_joint': -1.5,  # [rad]
            'RR_calf_joint': -1.5,    # [rad]
        }

    class domain_rand(LeggedRobotCfg.domain_rand):
        randomize_friction = True
        friction_range = [0.1, 1.25]
        randomize_base_mass = True
        added_mass_range = [-1., 5.]
        push_robots = True
        push_interval_s = 5
        max_push_vel_xy = 1.5

    class terrain( LeggedRobotCfg.terrain ):
        mesh_type = 'trimesh' # "heightfield" # none, plane, heightfield or trimesh
        horizontal_scale = 0.1 # [m]
        vertical_scale = 0.005 # [m]
        border_size = 25 # [m]
        curriculum = False
        static_friction = 1.0
        dynamic_friction = 1.0
        restitution = 0.
        # rough terrain only:
        measure_heights = True
        measured_points_x = [-0.8, -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0., 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8] # 1mx1.6m rectangle (without center line)
        measured_points_y = [-0.5, -0.4, -0.3, -0.2, -0.1, 0., 0.1, 0.2, 0.3, 0.4, 0.5]
        selected = False # select a unique terrain type and pass all arguments
        terrain_kwargs = None # Dict of arguments for selected terrain
        max_init_terrain_level = 5 # starting curriculum state
        terrain_length = 8.
        terrain_width = 8.
        num_rows = 10 # number of terrain rows (levels)
        num_cols = 30 # number of terrain cols (types)
        # terrain types: [smooth slope, rough slope, stairs up, stairs down, discrete]
        # terrain_proportions = [0.1, 0.1, 0.35, 0.25, 0.2]
        terrain_proportions = [0.1, 0.2, 0.3, 0.45, 0.6, 0.8, 1.0]
        # trimesh only:
        slope_treshold = 0.75 # slopes above this threshold will be corrected to vertical surfaces

    class viewer( LeggedRobotCfg.viewer ):
        ref_env = 0
        follow = True
        pos = [70, 0, 6]  # [m]
        lookat = [71., 5, 3.]  # [m]

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        control_type = 'P'
        stiffness = {'joint': 20.}  # [N*m/rad]
        damping = {'joint': 0.5}     # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.25
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4

    class asset( LeggedRobotCfg.asset ):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/aliengo/urdf/aliengo.urdf'
        name = "aliengo"
        foot_name = "foot"
        penalize_contacts_on = ["thigh", "calf", "base"]
        terminate_after_contacts_on = ["base"]
        termination_duration = 3 #seconds
        self_collisions = 0 # 1 to disable, 0 to enable...bitwise filter
  
    class rewards( LeggedRobotCfg.rewards ):
        soft_dof_pos_limit = 0.9
        base_height_target = 0.43
        only_positive_rewards = True # if true negative total rewards are clipped at zero (avoids early termination problems)
        class scales( LeggedRobotCfg.rewards.scales ):
            tracking_lin_vel = 2.0
            tracking_ang_vel = 1.0
            torques = -0.0002
            dof_pos_limits = -10.0
            # alive = 0.15
            contact_no_vel = -0.2
            # collision = 0.0
            # torques = -0.00001
            # dof_pos_limits = -10.0
            # base_height = -10.0
            # orientation = -1.0
        # class scales( LeggedRobotCfg.rewards.scales ):
        #     # tracking_lin_vel = 1.0
        #     # tracking_ang_vel = 0.5
        #     # lin_vel_z = -2.0
        #     # ang_vel_xy = -0.05
        #     # orientation = -1.0
        #     # base_height = -10.0
        #     # dof_acc = -2.5e-7
        #     # dof_vel = -1e-3
        #     # feet_air_time = 0.0
        #     collision = 0.0
        #     # action_rate = -0.01
        #     # dof_pos_limits = -5.0
        #     # alive = 0.15
        #     # hip_pos = -1.0
        #     # contact_no_vel = -0.2
        #     # feet_swing_height = -20.0
        #     # contact = 0.18

class ALEINGORoughCfgPPO( LeggedRobotCfgPPO ):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01
    class runner( LeggedRobotCfgPPO.runner ):
        max_iterations = 10000 # number of policy updates
        run_name = ''
        experiment_name = 'rough_aliengo'