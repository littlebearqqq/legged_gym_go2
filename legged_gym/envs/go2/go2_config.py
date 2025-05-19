from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class GO2RoughCfg( LeggedRobotCfg ):
    class env:
        num_envs = 4096
        num_observations = 45
        num_privileged_obs = None # if not None a priviledge_obs_buf will be returned by step() (critic obs for assymetric training). None is returned otherwise 
        num_actions = 12
        env_spacing = 3.  # not used with heightfields/trimeshes 
        send_timeouts = True # send time out information to the algorithm
        episode_length_s = 20 # episode length in seconds
        test = False


    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.42] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0

            'FL_hip_joint': 0.1,   # [rad]
            'FL_thigh_joint': 0.8,     # [rad]
            'FL_calf_joint': -1.5,   # [rad]


            'FR_hip_joint': -0.1 ,  # [rad]
            'FR_thigh_joint': 0.8,     # [rad]
            'FR_calf_joint': -1.5,  # [rad]



            'RL_hip_joint': 0.1,   # [rad]
            'RL_thigh_joint': 1.,   # [rad]
            'RL_calf_joint': -1.5,    # [rad]


            'RR_hip_joint': -0.1,   # [rad]
            'RR_thigh_joint': 1.,   # [rad]
            'RR_calf_joint': -1.5,    # [rad]

        
          
        }

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
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/go2/urdf/go2.urdf'
        name = "go2"
        foot_name = "foot"
        penalize_contacts_on = ["thigh", "calf"]
        terminate_after_contacts_on = ["base","calf"]
        self_collisions = 1 # 1 to disable, 0 to enable...bitwise filter
    class domain_rand:
        randomize_friction = True
        friction_range = [0.5, 1.25]
        randomize_base_mass = False
        added_mass_range = [-0.5, 0.5]
        push_robots = True
        push_interval_s = 15
        max_push_vel_xy = 1.


    class rewards( LeggedRobotCfg.rewards ):
        soft_dof_pos_limit = 0.9
        base_height_target = 0.35
        touch_thr= 8 #N
        class scales( LeggedRobotCfg.rewards.scales ):
            termination = -0.0
            tracking_lin_vel = 2.0
            #tracking_ang_vel = 0.4
            lin_vel_z = -2.0
            #ang_vel_xy = -0.05
            orientation = -0.
            torques = -0.00001
            dof_vel = -0.
            dof_acc = -2.5e-7
            base_height = -0.3 
            feet_air_time =  1.0 #这个最好保持为1 如果修改大了 腿可能会颤抖
            collision = -1.
            feet_stumble = -0.0 
            action_rate = -0.01
            stand_still = -0.
            dof_pos_limits = -10.0

            hip_pos = -0.5
            feet_contact_number=-0.2
            orientation_eular = 0.32
            feet_contact_forces = -0.01
            foot_slip = -0.05
class GO2RoughCfgPPO( LeggedRobotCfgPPO ):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01
    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        max_iterations = 16000 
        experiment_name = 'rough_go2'
        save_interval = 500 # check for potential saves every this many iterations
        resume = True
        resume_path = "/home/lemon/isaacgym/unitree_rl_gym/logs/rough_go2/May12_22-07-37_/model_1500.pt" # updated from load_run and chkpt
