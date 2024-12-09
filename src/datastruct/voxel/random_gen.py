from OCC.Core.gp import gp_Pnt
from OCC.Core.Bnd import Bnd_Box    

from typing import List, Tuple, Set, Optional
import random

from src.datastruct.voxel.voxel_grids import VoxelGrids3D, VoxelNode
from src.algorithm.pathfinding.pathfinding import AStar
    
    
class BndObstacleGen:
    def __init__(self,
                available_area: Bnd_Box) -> None:
        self.available_area: Bnd_Box = available_area
    
    def _sample_random_corner(self, max_value: float, min_value: float, dist_boundary: Tuple[float, float]) -> Tuple[float, float]:
        while True:
            max_point = random.uniform(min_value, max_value)   
            min_point = random.uniform(min_value, max_value)   
            min_dist, max_dist = dist_boundary
            if abs(max_point - min_point) > min_dist and abs(max_point - min_point) < max_dist:
                return max(max_point, min_point), min(max_point, min_point) 

    def _generate_random_bnd_box(self) -> Bnd_Box:

        # 최대 및 최소 x, y를 결정
        maxx: float = self.available_area.CornerMax().X()  
        minx: float = self.available_area.CornerMin().X()  
        maxy: float = self.available_area.CornerMax().Y()  
        miny: float = self.available_area.CornerMin().Y()  
        
        corner_maxx, corner_minx = self._sample_random_corner(maxx, minx, (150, 250))
        corner_maxy, corner_miny = self._sample_random_corner(maxy, miny, (60, 80))    
        corner_maxz = self.available_area.CornerMax().Z()
        corner_minz = self.available_area.CornerMin().Z()
        
        corner_max = gp_Pnt(corner_maxx, corner_maxy, corner_maxz)
        corner_min = gp_Pnt(corner_minx, corner_miny, corner_minz)
        
        # 바운드 박스를 정해진 바운더리 내에서 생성
        # 바운드 박스는 최대 꼭지점과 최소 꼭지점을 가지고 있음
        bnd_box: Bnd_Box = Bnd_Box()    
        bnd_box.SetGap(0.0) 
        bnd_box.Update(*corner_min.Coord(), *corner_max.Coord())    
        return bnd_box
    
    def _has_over_rap(self, bnd_box: Bnd_Box, others: List[Bnd_Box]) -> bool: 
        for other in others:
            if not bnd_box.IsOut(other):
                return True
            if not other.IsOut(bnd_box):
                return True 
        return False
    
    def _create_voxel_grids(self, available_area: Bnd_Box, resolution: int) -> VoxelGrids3D:
        corner_max = available_area.CornerMax()   
        corner_min = available_area.CornerMin()  
        
        x_gap = corner_max.X() - corner_min.X() 
        y_gap = corner_max.Y() - corner_min.Y() 
        z_gap = corner_max.Z() - corner_min.Z() 
        
        gap = max(x_gap, y_gap, z_gap)
        voxel_corner_max = gp_Pnt(corner_max.X(), corner_max.Y(), corner_max.Z())
        voxel_corner_min = gp_Pnt(corner_max.X() - gap, 
                                corner_max.Y() - gap, 
                                corner_max.Z() - gap)   
        
        voxel_grids = VoxelGrids3D(corner_max=voxel_corner_max, corner_min=voxel_corner_min, map_size=resolution)
        for node in voxel_grids:
            if available_area.IsOut(node.position):
                node.is_obstacle = True    
        return voxel_grids

    def generate(self, map_size: int, n_boxes: int) -> VoxelGrids3D:        
        bnd_boxes: List[Bnd_Box] = []
        while len(bnd_boxes) < n_boxes:
            bnd_box = self._generate_random_bnd_box()   
            if self._has_over_rap(bnd_box, bnd_boxes): 
                continue
            bnd_boxes.append(bnd_box)
            
        voxel_grids = self._create_voxel_grids(
            available_area=self.available_area,
            resolution=map_size)
        
        for bnd_box in bnd_boxes:
            maxx, maxy, maxz = bnd_box.CornerMax().Coord()
            minx, miny, minz = bnd_box.CornerMin().Coord()
            for node in voxel_grids:
                if (maxx > node.position.X() > minx) and \
                    (maxy > node.position.Y() > miny):
                    node.is_obstacle = True 
        return voxel_grids   
    
class NodePairGen:
    def __init__(self, grids: VoxelGrids3D) -> None:
        self.grids: VoxelGrids3D = grids
        
        
    def _count_turning(self, path_nodes: List[VoxelNode]) -> int:
        n_turning = 0
        for idx, node in enumerate(path_nodes[1:-1]):
            i, j, k = node.i, node.j, node.k
            prev_i, prev_j, prev_k = path_nodes[idx - 1].i, path_nodes[idx - 1].j, path_nodes[idx - 1].k
            next_i, next_j, next_k = path_nodes[idx + 1].i, path_nodes[idx + 1].j, path_nodes[idx + 1].k  
            nxt_dir_i, nxt_dir_j, nxt_dir_k = next_i - i, next_j - j, next_k - k    
            prv_dir_i, prv_dir_j, prv_dir_k = i - prev_i, j - prev_j, k - prev_k    
            if nxt_dir_i != prv_dir_i or nxt_dir_j != prv_dir_j or nxt_dir_k != prv_dir_k:
                n_turning += 1
        return n_turning
    
    def _is_valid_path(self, grids: VoxelGrids3D, len_boundary: Tuple[int, int], turning_boundary: Tuple[int, int]) -> bool:
        min_len, max_len = len_boundary
        min_turning, max_turning = turning_boundary 
        
        ast = AStar(grids)
        if ast.search():
            path_nodes: List[VoxelNode] = ast.get_path_nodes()   
            n_nodes = len(path_nodes)
            if n_nodes > max_len or n_nodes < min_len:
                return False
            if self._count_turning(path_nodes) > max_turning or self._count_turning(path_nodes) < min_turning:     
                return False

            return True
        
        return False

    def generate(self, n_node_pairs: int, len_boundary: Tuple[int, int], turning_boundary: Tuple[int, int]) -> List[Tuple[VoxelNode, VoxelNode]]:
        grids: VoxelGrids3D = self.grids
        
        free_node: List[VoxelNode] = [node for node in grids if not node.is_obstacle]    
        terminal_pairs: List[Tuple[VoxelNode, VoxelNode]] = []    
        while n_node_pairs > len(terminal_pairs):
            src_node = random.choice(free_node)
            dst_node = random.choice(free_node)
            grids.set_start_node(src_node)
            grids.set_goal_node(dst_node)
            
            if not self._is_valid_path(grids, len_boundary, turning_boundary):
                continue
            if (src_node, dst_node) in terminal_pairs or src_node == dst_node:
                continue
            if src_node != dst_node:
                terminal_pairs.append((src_node, dst_node))
        
        return terminal_pairs
