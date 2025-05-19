from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class tinymalRoughCfg( LeggedRobotCfg ):
    class env:
        num_envs = 2048
        num_observations = 45
        num_privileged_obs = None # if not None a priviledge_obs_buf will be returned by step() (critic obs for assymetric training). None is returned otherwise 
        num_actions = 12
        env_spacing = 3.  # not used with heightfields/trimeshes 
        send_timeouts = True # send time out information to the algorithm
        episode_length_s = 20 # episode length in seconds
        test = False


    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.15] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            "LF_HAA": -0.2,
            "LH_HAA": 0.2,
            "RF_HAA": 0.2,
            "RH_HAA": -0.2,

            "LF_HFE": 0.9,
            "LH_HFE": -0.9,
            "RF_HFE": 0.9,
            "RH_HFE": -0.9,

            "LF_KFE": -1.55,
            "LH_KFE": 1.55,
            "RF_KFE": -1.55,
            "RH_KFE": 1.55,
        }
    class commands(LeggedRobotCfg.commands):
        class ranges:
            lin_vel_x = [-0.8, 0.8] # min max [m/s]
            lin_vel_y = [-0.8, 0.8]   # min max [m/s]
            ang_vel_yaw = [-1.2, 1.2]    # min max [rad/s]
            heading = [-3.14, 3.14]

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        control_type = 'P'
        stiffness = {'HAA': 3.3, 'HFE': 3.3, 'KFE': 3.3}  # [N*m/rad]
        damping = {'HAA': 0.25, 'HFE': 0.25, 'KFE': 0.25}     # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.25
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4

    class asset( LeggedRobotCfg.asset ):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/tinymal/urdf/tinymal.urdf'
        name = "tinymal"
        foot_name = "FOOT"
        penalize_contacts_on = ["thigh", "calf"]
        terminate_after_contacts_on = ["trunk","calf"]
        self_collisions = 0 # 1 to disable, 0 to enable...bitwise filter
        flip_visual_attachments = False #tinymal
    class domain_rand:
        randomize_friction = True
        friction_range = [0.5, 1.25]
        randomize_base_mass = False
        added_mass_range = [-0.2, 0.5]
        push_robots = True
        push_interval_s = 15
        max_push_vel_xy = 1.


    class rewards( LeggedRobotCfg.rewards ):
        soft_dof_pos_limit = 0.9
        base_height_target = 0.18
        touch_thr= 5 #N

        class scales( LeggedRobotCfg.rewards.scales ):
            termination = -0.0
            #tracking_lin_vel = 2.0
            # #tracking_ang_vel = 0.4
            # lin_vel_z = -2.0
            # #ang_vel_xy = -0.05
            #orientation = 0.1
            # torques = -0.00001
            # dof_vel = -0.
            # dof_acc = -2.5e-7
            base_height = -0.2 

            # feet_air_time =  1.0
            # collision = -1.
            # feet_stumble = -0.0 
            # action_rate = -0.01
            # stand_still = -0.
            # dof_pos_limits = -10.0
            #user add reward
            hip_pos = -0.5
            feet_contact_number=-0.05
            orientation_eular = 0.15
            feet_contact_forces = -0.01
            foot_slip = -0.05
            vel_mismatch_exp = 0.35
class tinymalRoughCfgPPO( LeggedRobotCfgPPO ):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01
    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        max_iterations = 16000 
        save_interval = 500
        experiment_name = 'tinymal'
        resume = False
        resume_path = "/home/lemon/isaacgym/unitree_rl_gym/logs/rough_go2/May12_22-07-37_/model_1500.pt" # updated from load_run and chkpt
