from OCC.Core.Bnd import Bnd_Box

import numpy as np
import gymnasium as gym
from gymnasium import spaces
from typing import Tuple, List, Optional, Set, Dict
import random

from src.algorithm.reinforcement.agent import SensorRoutingAgent
from src.datastruct.voxel.voxel_grids import VoxelGrids3D, VoxelNode
from src.datastruct.voxel.random_gen import BndObstacleGen, NodePairGen


class PanelRandomRoutingEnv(gym.Env):
    metadata = {"render_modes": [None]}
    def __init__(self,
                voxel_gen: BndObstacleGen, 
                n_boxes: int = 3,
                len_bounary: tuple[int, int] = (5, 3),
                turning_boundary: tuple[int, int] = (2, 1),  
                map_size: int = 30, 
                max_step: int = 100):
        super().__init__()
        self.n_boxes: int = n_boxes 
        self.voxel_gen: BndObstacleGen = voxel_gen
        self.len_bounary: tuple[int, int] = len_bounary 
        self.turning_boundary: tuple[int, int] = turning_boundary      

        self.cur_step: int = 0
        self.max_step: int = max_step
        self.map_size: int = map_size
        
        self.action_space = spaces.Discrete(n = 26)
        self.observation_space = spaces.Box(low=0, high=1.0, shape=(56*1,), dtype=np.float32)
        
        self.agent: Optional[SensorRoutingAgent] = None   
        self.grids: Optional[VoxelGrids3D] = None
        self.cur_node: Optional[VoxelNode] = None
    
    
    def step(self, action):
        reward: float = -1.0
        terminated: bool = False
        self.cur_step += 1
        action: tuple[int, int, int] = self.agent.action(action)

        nxt_i, nxt_j, nxt_k = self.cur_node.i + action[0], self.cur_node.j + action[1], self.cur_node.k + action[2] 
        nxt_node: VoxelNode = self.grids[nxt_i, nxt_j, nxt_k]
        
        
        old_distance = self.cur_node.center_pnt.Distance(self.grids.goal_node.center_pnt)   
        new_distance = nxt_node.center_pnt.Distance(self.grids.goal_node.center_pnt)
        if new_distance >= old_distance:
            reward -= 1.0
        # else:
        #     reward += 1.0   
        
        nxt_node.parent = self.cur_node
        self.cur_node = nxt_node
        if self.cur_node.is_goal_node: 
            print("Routing is Done!!")
            terminated = True
            reward += 10.0
            
        if self.cur_step >= self.max_step:
            print("Max Step")
            reward -= 10.0
            terminated = True
    
        return self.agent.get_observation(self.cur_node), reward, terminated, False, {}

    def reset(self, seed=None, options=None):
        self.cur_step = 0
        
        n_boxes = random.randint(1, self.max_boxes) 
        self.grids = self.voxel_gen.generate(map_size=self.map_size, n_boxes=n_boxes)
        node_gen = NodePairGen(self.grids)
        src_node, dst_node = node_gen.generate(1, self.len_bounary, self.turning_boundary)[0]

        self.agent = SensorRoutingAgent(self.grids, n_frames=1)
        self.cur_node: VoxelNode = self.grids.start_node   
        self.cur_step = 0
        
        return self.agent.get_observation(self.cur_node), {}
    
    def valid_action_mask(self):
        n_actions = self.action_space.n
        possible_actions = np.arange(n_actions)

        x, y, z = self.cur_node.i, self.cur_node.j, self.cur_node.k
        invalid_actions = []
        for i in range(n_actions):
            nx, ny, nz = self.agent.action_table[i]
            nx, ny, nz = nx + x, ny + y, nz + z
            if 0 > nx or nx >= self.grids.map_size or\
                0 > ny or ny >= self.grids.map_size or \
                0 > nz or nz >= self.grids.map_size:
                    invalid_actions.append(i)
                    continue
            if self.grids[nx, ny, nz].is_obstacle:
                invalid_actions.append(i)
        masked_action = [action not in invalid_actions for action in possible_actions]
        return masked_action
    
    def render(self):
        return NotImplemented()

    def close(self):
        return NotImplemented()



