
# from legged_gym import LEGGED_GYM_ROOT_DIR, envs
from legged_gym.envs.base.legged_robot import LeggedRobot

from isaacgym.torch_utils import *
from isaacgym import gymtorch, gymapi, gymutil
import torch
from legged_gym.utils.terrain import Terrain
from isaacgym.terrain_utils import *

class Go2Robot(LeggedRobot):

    def reset_idx(self, env_ids):
        """ Reset some environments.
            Calls self._reset_dofs(env_ids), self._reset_root_states(env_ids), and self._resample_commands(env_ids)
            [Optional] calls self._update_terrain_curriculum(env_ids), self.update_command_curriculum(env_ids) and
            Logs episode info
            Resets some buffers

        Args:
            env_ids (list[int]): List of environment ids which must be reset
        """
        if len(env_ids) == 0:
            return
        
        # reset robot states
        self._reset_dofs(env_ids)
        self._reset_root_states(env_ids)

        self._resample_commands(env_ids)

        # reset buffers
        self.last_actions[env_ids] = 0.
        self.last_dof_vel[env_ids] = 0.
        self.feet_air_time[env_ids] = 0.
        self.episode_length_buf[env_ids] = 0
        self.reset_buf[env_ids] = 1
        # fill extras
        self.extras["episode"] = {}
        for key in self.episode_sums.keys():
            self.extras["episode"]['rew_' + key] = torch.mean(self.episode_sums[key][env_ids]) / self.max_episode_length_s
            self.episode_sums[key][env_ids] = 0.
        if self.cfg.commands.curriculum:
            self.extras["episode"]["max_command_x"] = self.command_ranges["lin_vel_x"][1]
        # send timeout info to the algorithm
        if self.cfg.env.send_timeouts:
            self.extras["time_outs"] = self.time_out_buf
    
    def create_sim(self):
        """ Creates simulation, terrain and evironments
        """
        self.up_axis_idx = 2 # 2 for z, 1 for y -> adapt gravity accordingly
        self.sim = self.gym.create_sim(self.sim_device_id, self.graphics_device_id, self.physics_engine, self.sim_params)
        print(f"\033[34mTerrain type is {self.cfg.terrain.mesh_type}\033[0m")
        # 1) Instantiate your Terrain class
        # (only if mesh_type is 'heightfield' or 'trimesh')
        self.terrain = Terrain(self.cfg.terrain, self.num_envs)
        # self._create_ground_plane()
        if self.cfg.terrain.mesh_type in ["heightfield", "trimesh"]:
            # Pass in your terrain config and the number of envs (robots)
            # self.terrain = Terrain(self.cfg.terrain, self.num_envs)

            # 2) Register the heightfield or trimesh with Isaac Gym
            if self.cfg.terrain.mesh_type == "heightfield":
                # self.add_terrain("slope")
                # self.add_terrain("stair", 3.95, True)
                self.add_random_uniform_terrain()
                # self._create_trimesh()
            else:
                self._create_trimesh()
        else:
            # Fallback: just create a flat plane if mesh_type is "plane" or "none"
            # self._create_ground_plane()
            pass
        # After terrain creation, set the camera to center

        self._create_envs()


    
    def _reset_root_states(self, env_ids):
        """ Resets ROOT states position and velocities of selected environmments
            Sets base position based on the curriculum
            Selects randomized base velocities within -0.5:0.5 [m/s, rad/s]
        Args:
            env_ids (List[int]): Environemnt ids
        """
        # base position
        if self.custom_origins:
            self.root_states[env_ids] = self.base_init_state
            for index in env_ids:
                x,y,z = self.random_robot_position()
                self.env_origins[index, 0] = x
                self.env_origins[index, 1] = y
                self.env_origins[index, 2] = z
            self.root_states[env_ids, :3] += self.env_origins[env_ids]
            if not self.headless and self.cfg.viewer.ref_env in env_ids and self.cfg.viewer.follow:
                # Suppose env_origins[0] is something like [x0, y0, z0] in a torch.Tensor
                origin = self.env_origins[self.cfg.viewer.ref_env].cpu().numpy()   # -> array([x0, y0, z0])
                # Add some Z offset so the camera sits above the environment
                cam_x_offset = -3.0     # Move camera behind the robot in X
                cam_y_offset = 2.0      # Move camera slightly to the left in Y
                cam_z_offset = 6.0      # Move camera above the robot in Z

                camera_pos = [
                    origin[0] + cam_x_offset,
                    origin[1] + cam_y_offset,
                    origin[2] + cam_z_offset
                ]

                # Look at the robot’s position but slightly angled downward
                camera_target = [
                    origin[0],
                    origin[1],
                    origin[2] + 3.0
                ]

                # self.set_camera(camera_pos, camera_target)
                self.set_camera(camera_pos, camera_target)
        else:
            self.root_states[env_ids] = self.base_init_state
            self.root_states[env_ids, :3] += self.env_origins[env_ids]
        # base velocities
        self.root_states[env_ids, 7:13] = torch_rand_float(-0.5, 0.5, (len(env_ids), 6), device=self.device) # [7:10]: lin vel, [10:13]: ang vel
        env_ids_int32 = env_ids.to(dtype=torch.int32)
        self.gym.set_actor_root_state_tensor_indexed(self.sim,
                                                     gymtorch.unwrap_tensor(self.root_states),
                                                     gymtorch.unwrap_tensor(env_ids_int32), len(env_ids_int32))

    #----------------------------------------
    def _create_ground_plane(self):
        """ Adds a ground plane to the simulation, sets friction and restitution based on the cfg.
        """
        plane_params = gymapi.PlaneParams()
        plane_params.normal = gymapi.Vec3(0.0, 0.0, 1.0)
        plane_params.static_friction = self.cfg.terrain.static_friction
        plane_params.dynamic_friction = self.cfg.terrain.dynamic_friction
        plane_params.restitution = self.cfg.terrain.restitution
        self.gym.add_ground(self.sim, plane_params)


    def _create_trimesh(self):
        vertices = self.terrain.vertices.astype(np.float32)
        triangles = self.terrain.triangles.astype(np.uint32)
        
        min_x = np.min(vertices[:, 0])
        max_x = np.max(vertices[:, 0])
        min_y = np.min(vertices[:, 1])
        max_y = np.max(vertices[:, 1])
        min_z = np.min(vertices[:, 2])
        max_z = np.max(vertices[:, 2])
        
        # Shift the entire mesh so bottom-left corner is at (0,0,0)
        # vertices[:, 0] = 
        # vertices[:, 1] = (max_y -min_y)//2
        # vertices[:, 2] -= min_z

        # Now rasterize for PhysX
        vertices_1d = vertices.ravel()
        triangles_1d = triangles.ravel()

        tm_params = gymapi.TriangleMeshParams()
        tm_params.nb_vertices  = len(vertices_1d) // 3
        tm_params.nb_triangles = len(triangles_1d) // 3

        # No transform offset needed if we've shifted the vertices
        # tm_params.transform.p.x -=  (max_x -min_x)//4
        # tm_params.transform.p.y -=  (max_y -min_y)//4 
        # tm_params.transform.p.z = 0.0

        self.gym.add_triangle_mesh(self.sim, vertices_1d, triangles_1d, tm_params)

    # def _get_env_origins(self):
    #     """ Sets environment origins. On rough terrain the origins are defined by the terrain platforms.
    #         Otherwise create a grid.
    #     """
    #     self.custom_origins = False
    #     self.env_origins = torch.zeros(self.num_envs, 3, device=self.device, requires_grad=False)
    #     # # # create a grid of robots
    #     num_cols = np.floor(np.sqrt(self.num_envs))
    #     num_rows = np.ceil(self.num_envs / Go2Robotnum_cols)
    #     xx, yy = torch.meshgrid(torch.arange(num_rows), torch.arange(num_cols))
    #     spacing = self.cfg.env.env_spacing
    #     self.env_origins[:, 0] = spacing * xx.flatten()[:self.num_envs]
    #     self.env_origins[:, 1] = spacing * yy.flatten()[:self.num_envs]
    #     self.env_origins[:, 2] = 0.


    def _get_env_origins(self):
        self.custom_origins = True
        self.env_origins = torch.zeros(self.num_envs, 3, device=self.device, requires_grad=False)
        for i in range(self.num_envs):
            x,y,z = self.random_robot_position()
            # Convert to torch
            self.env_origins[i, 0] = x
            self.env_origins[i, 1] = y
            self.env_origins[i, 2] = z

    def random_robot_position(self):
        # 1. Sample random row, col
        row = np.random.randint(self.terrain.border, self.terrain.tot_rows - self.terrain.border)
        col = np.random.randint(self.terrain.border, self.terrain.tot_cols - self.terrain.border)
        # 2. Convert (row, col) -> (x, y) in world coords
        # Each cell is horizontal_scale in size
        x = row * self.terrain.cfg.horizontal_scale 
        y = col * self.terrain.cfg.horizontal_scale
        
        # 3. Get terrain height in meters
        z = self.terrain.height_field_raw[row, col] * self.terrain.cfg.vertical_scale
        
        # 4. Add a small offset so the robot spawns above the ground
        # z += 0.1  # for example
        
        return x, y, z
