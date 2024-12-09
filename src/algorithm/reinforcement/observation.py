import numpy as np
from typing import Tuple, List, Optional, Set, Dict
from itertools import product

from src.datastruct.voxel.voxel_grids import VoxelGrids3D, VoxelNode

class SensorObservation:
    def __init__(self, grids: VoxelGrids3D) -> None:
        self.grids: VoxelGrids3D = grids 
        self.dir_table = {(1,1,1)}
        for i, j, k in product([-1, 0, 1], repeat=3):
            if i == j == k == 0:
                continue
            self.dir_table[(i, j, k)] = len(self.dir_table)
        
        self.max_map_size: int = 50
        self.max_sensing_range: int = 10


    def _is_valid_node(self, src_node: VoxelNode, dir_i: int, dir_j: int, dir_k: int) -> bool:     
        map_size: int = self.grids.map_size
        nxt_i, nxt_j, nxt_k = src_node.i + dir_i, src_node.j + dir_j, src_node.k + dir_k
        
        if  0 <= nxt_i < map_size and \
            0 <= nxt_j < map_size and \
            0 <= nxt_k < map_size:
            if self.grids[nxt_i, nxt_j, nxt_k].is_obstacle:
                return False
            else:
                return True
        return False
    
    def _sensing_forward(self, src_node: VoxelNode, dir_i: int, dir_j: int, dir_k: int) -> Tuple[float, float]: 
        node_state: float = 1.0
        cur_node: VoxelNode = src_node
        n_steps = 0
        
        while self._is_valid_node(cur_node, dir_i, dir_j, dir_k) and n_steps < self.max_sensing_range:   
            nxt_i, nxt_j, nxt_k = cur_node.i + dir_i, cur_node.j + dir_j, cur_node.k + dir_k    
            cur_node = self.grids.nodes_map[nxt_i][nxt_j][nxt_k]    
            n_steps += 1
            if cur_node.is_obstacle:
                node_state = 0.0
                break
            
            if cur_node.is_goal_node:   
                node_state = 0.5
                break
        
        relative_distance: float = n_steps / self.max_sensing_range
        return relative_distance, node_state

    def get_obsesrvation(self, cur_node: VoxelNode) -> np.ndarray:
        observation: np.ndarray = np.zeros(56)
        goal_node = self.grids.goal_node
        for idx, dir in enumerate(self.dir_table):
            observation[idx], observation[idx+len(self.dir_table)] = self._sensing_forward(cur_node, *dir)
        start_to_dst  = self.grids.start_node.center_pnt.Distance(self.grids.goal_node.center_pnt)  
        cur_to_dst = self.grids.start_node.center_pnt.Distance(cur_node.center_pnt)
        to_dst_feature = cur_to_dst / start_to_dst  
        
        observation[52] = np.clip(to_dst_feature, 0, 1)
        magnitude = goal_node.center_pnt.Distance(cur_node.center_pnt)
        if magnitude == 0:
            return observation
        dx = (goal_node.center_pnt.X() - cur_node.center_pnt.X())/magnitude
        dy = (goal_node.center_pnt.Y() - cur_node.center_pnt.Y())/magnitude
        dz = (goal_node.center_pnt.Z() - cur_node.center_pnt.Z())/magnitude 

        observation[53], observation[54], observation[55] = dx, dy, dz
        return observation