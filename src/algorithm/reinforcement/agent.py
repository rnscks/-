from abc import ABC
import numpy as np
from typing import Tuple, List, Optional, Set, Dict
from itertools import product

from src.datastruct.voxel.voxel_grids import VoxelGrids3D, VoxelNode
from src.algorithm.reinforcement.observation import SensorObservation


class VoxelAgent(ABC):
    def __init__(self, grids: VoxelGrids3D = None) -> None:
        super().__init__()   
        self.grids: Optional[VoxelGrids3D] = grids 
        self.action_table = {}
        for i, j, k in product([-1, 0, 1], repeat=3):
            if i == j == k == 0:
                continue
            self.action_table[len(self.action_table)] = (i, j, k)   

    def _get_action(self, action: np.int64) -> Tuple[int, int, int]:
        return self.action_table[action]
    
    def _has_line_of_sight(self, src_node: VoxelNode, dst_node: VoxelNode) -> bool:
        x1, y1, z1 = src_node.i, src_node.j, src_node.k
        x2, y2, z2 = dst_node.i, dst_node.j, dst_node.k

        if (x2 > x1):
            xs = 1
            dx = x2 - x1
        else:
            xs = -1
            dx = x1 - x2

        if (y2 > y1):
            ys = 1
            dy = y2 - y1
        else:
            ys = -1
            dy = y1 - y2

        if (z2 > z1):
            zs = 1
            dz = z2 - z1
        else:
            zs = -1
            dz = z1 - z2

        if (dx >= dy and dx >= dz):
            p1 = 2 * dy - dx
            p2 = 2 * dz - dx
            while (x1 != x2):
                x1 += xs
                if (p1 >= 0):
                    y1 += ys
                    p1 -= 2 * dx
                if (p2 >= 0):
                    z1 += zs
                    p2 -= 2 * dx
                p1 += 2 * dy
                p2 += 2 * dz
                if self.grids[x1, y1, z1].is_obstacle: 
                    return False

        elif (dy >= dx and dy >= dz):
            p1 = 2 * dx - dy
            p2 = 2 * dz - dy
            while (y1 != y2):
                y1 += ys
                if (p1 >= 0):
                    x1 += xs
                    p1 -= 2 * dy
                if (p2 >= 0):
                    z1 += zs
                    p2 -= 2 * dy
                p1 += 2 * dx
                p2 += 2 * dz
                if self.grids[x1, y1, z1].is_obstacle:  
                    return False
        else:
            p1 = 2 * dy - dz
            p2 = 2 * dx - dz
            while (z1 != z2):
                z1 += zs
                if (p1 >= 0):
                    y1 += ys
                    p1 -= 2 * dz
                if (p2 >= 0):
                    x1 += xs
                    p2 -= 2 * dz
                p1 += 2 * dy
                p2 += 2 * dx
                if self.grids[x1, y1, z1].is_obstacle:  
                    return False
        return True
    
class SensorRoutingAgent(VoxelAgent):
    def __init__(self, grids: VoxelGrids3D, n_frames: int = 4, obs_dims: int = 56) -> None:
        super().__init__(grids=grids)
        self.sensor_observation = SensorObservation(grids)
        self.frames = []
        self.n_frames = n_frames    
        self.obs_dims: int= obs_dims


    def action(self, action: np.int64) -> Tuple[int, int, int]:
        return self._get_action(action)

    def get_observation(self, currnet_node: VoxelNode) -> np.ndarray:
        sensor_observation = self.sensor_observation.get_obsesrvation(currnet_node)
        observation = sensor_observation
        # bouding_observation = self.bounding_obersevation.get_obsesrvation() 
        # observation = np.concatenate((sensor_observation, bouding_observation)) 
        
        self.frames.append(observation)
        if len(self.frames) > self.n_frames:
            self.frames.pop(0)
        rl_observation = np.array(self.frames) 
        padded_observation = np.zeros(self.obs_dims*self.n_frames)
        padded_observation[:len(rl_observation.flatten())] = rl_observation.flatten() 
        return padded_observation 