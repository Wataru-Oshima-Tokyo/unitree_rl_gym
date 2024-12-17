import time

import mujoco.viewer
import mujoco
import numpy as np
from legged_gym import LEGGED_GYM_ROOT_DIR
import torch
import yaml


def get_gravity_orientation(quaternion):
    qw = quaternion[0]
    qx = quaternion[1]
    qy = quaternion[2]
    qz = quaternion[3]

    gravity_orientation = np.zeros(3)
    gravity_orientation[0] = 2 * (-qz * qx + qw * qy)
    gravity_orientation[1] = -2 * (qz * qy + qw * qx)
    gravity_orientation[2] = 1 - 2 * (qw * qw + qz * qz)

    return gravity_orientation


def pd_control(target_q, q, kp, target_dq, dq, kd):
    """Calculates torques from position commands"""
    return (target_q - q) * kp + (target_dq - dq) * kd


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("config_file", type=str, help="config file name in the config folder")
    args = parser.parse_args()
    config_file = args.config_file

    # Load configuration file
    with open(f"{LEGGED_GYM_ROOT_DIR}/deploy/deploy_mujoco/configs/{config_file}", "r") as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
        policy_path = config["policy_path"].replace("{LEGGED_GYM_ROOT_DIR}", LEGGED_GYM_ROOT_DIR)
        xml_path = config["xml_path"].replace("{LEGGED_GYM_ROOT_DIR}", LEGGED_GYM_ROOT_DIR)
        simulation_duration = config["simulation_duration"]
        simulation_dt = config["simulation_dt"]
        control_decimation = config["control_decimation"]
        kps = np.array(config["kps"], dtype=np.float32)
        kds = np.array(config["kds"], dtype=np.float32)
        default_angles = np.array(config["default_angles"], dtype=np.float32)
        lin_vel_scale = config["lin_vel_scale"]
        ang_vel_scale = config["ang_vel_scale"]
        dof_pos_scale = config["dof_pos_scale"]
        dof_vel_scale = config["dof_vel_scale"]
        action_scale = config["action_scale"]
        cmd_scale = np.array(config["cmd_scale"], dtype=np.float32)
        num_actions = config["num_actions"]
        num_obs = config["num_obs"]
        cmd = np.array(config["cmd_init"], dtype=np.float32)

        # Print loaded configuration
        print("\nLoaded Configuration:")
        print(yaml.dump(config, default_flow_style=False))

    action = np.zeros(num_actions, dtype=np.float32)
    target_dof_pos = default_angles.copy()
    obs = np.zeros(num_obs, dtype=np.float32)
    counter = 0

    # Load MuJoCo model
    m = mujoco.MjModel.from_xml_path(xml_path)
    d = mujoco.MjData(m)
    m.opt.timestep = simulation_dt

    # Load policy
    policy = torch.jit.load(policy_path)
    print("\nLoaded Policy:")
    print(policy)

    # Retrieve joint names
    xml_joint_names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(m.njnt)]
    actuated_joint_names = [xml_joint_names[m.dof_jntid[i]] for i in range(m.nu)]

    # Print all joint names
    xml_joint_names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(m.njnt)]
    print("All Joint Names in the XML:", xml_joint_names)

    # Print actuated joint names
    actuated_joint_names = [xml_joint_names[m.dof_jntid[i]] for i in range(m.nu)]
    print("Actuated Joint Names (Order matches d.qpos[7:]):", actuated_joint_names)
    print("Number of Actuated DOFs:", m.nu)
    print("Actuated DOFs:", actuated_joint_names)


    # Print all joint names
    print("All Joint Names in XML:", xml_joint_names)

    # Print actuated joint names (mapped from DOFs)
    actuated_joint_names = [xml_joint_names[jnt_id] if jnt_id >= 0 else None for jnt_id in m.dof_jntid]
    print("Actuated Joint Names:", actuated_joint_names)

    # Cross-check unused joints
    unused_joints = set(xml_joint_names) - set(actuated_joint_names)
    print("Unused Joints:", unused_joints)

    with mujoco.viewer.launch_passive(m, d) as viewer:
        start = time.time()
        last_cmd_update = start
        while viewer.is_running() and time.time() - start < simulation_duration:
            step_start = time.time()

            # Update command every 3 seconds
            current_time = time.time()
            if current_time - last_cmd_update > 3.0:
                cmd = np.random.uniform(-0.3, 0.3, size=3).astype(np.float32)  # Random linear and angular velocities
                # cmd = np.array([0.0, 0.5, 0.0]) # linear and angular velocities
                last_cmd_update = current_time
                print(f"New random cmd: {cmd}")
            tau = pd_control(target_dof_pos, d.qpos[7:], kps, np.zeros_like(kds), d.qvel[6:], kds)
            d.ctrl[:] = tau
            mujoco.mj_step(m, d)
            counter += 1

            if counter % control_decimation == 0:
                # Create observations

                qj = d.qpos[7:]
                dqj = d.qvel[6:]
                quat = d.qpos[3:7]
                linear = d.qvel[0:3]
                omega = d.qvel[3:6]
                qj = (qj - default_angles) * dof_pos_scale
                dqj = dqj * dof_vel_scale
                gravity_orientation = get_gravity_orientation(quat)
                linear = linear * lin_vel_scale
                omega = omega * ang_vel_scale

                period = 0.8
                count = counter * simulation_dt
                phase = count % period / period
                sin_phase = np.sin(2 * np.pi * phase)
                cos_phase = np.cos(2 * np.pi * phase)

                obs[:3] = linear
                obs[3:6] = omega
                obs[6:9] = gravity_orientation
                obs[9:12] = cmd * cmd_scale
                obs[12 : 12 + num_actions] = qj
                obs[12 + num_actions : 12 + 2 * num_actions] = dqj
                obs[12 + 2 * num_actions : 12 + 3 * num_actions] = action
                # obs[9 + 3 * num_actions : 9 + 3 * num_actions + 2] = np.array([sin_phase, cos_phase])
                obs_tensor = torch.from_numpy(obs).unsqueeze(0)
                # policy inference
                action = policy(obs_tensor).detach().numpy().squeeze()
                # transform action to target_dof_pos
                target_dof_pos = action * action_scale + default_angles
                if counter % (control_decimation*100) == 0:
                    print(f"d.ctrl {d.ctrl}")
                    print(f"\nStep {counter}:")
                    print("Applied Torques (tau):", tau)
                    print("Observations (scaled):", obs)
                    print("Action from Policy:", action)
                    print("Target DOF Positions:", target_dof_pos)

            viewer.sync()
            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
