from typing import List, Optional, Tuple, Set
from itertools import product
from abc import ABC, abstractmethod 
import heapq

from src.datastruct.voxel.voxel_grids import VoxelNode, VoxelGrids3D

class PathFollower:
    def __init__(self, grids: VoxelGrids3D) -> None:
        self.grids: VoxelGrids3D = grids 
        pass
    
    
    def get_path_nodes(self) -> List[VoxelNode]:
        path_nodes: List[VoxelNode] = []
        cur_node: VoxelNode = self.grids.goal_node
        self.grids.start_node.parent = None
        while cur_node:
            path_nodes.append(cur_node)
            if isinstance(cur_node.parent, VoxelNode):
                cur_node = cur_node.parent
            else:
                break
        
        return path_nodes[::-1]
    
    def get_smooth_path_nodes(self) -> List[VoxelNode]:
        path_nodes: List[VoxelNode] = []
        cur_node: VoxelNode = self.grids.goal_node
        self.grids.start_node.parent = None
        while cur_node:
            path_nodes.append(cur_node)
            if isinstance(cur_node.parent, VoxelNode):
                while cur_node.parent.parent != None:
                    if self._has_line_of_sight(cur_node, cur_node.parent.parent, self.grids):  
                        cur_node.parent = cur_node.parent.parent
                    else:
                        break
                cur_node = cur_node.parent
            else:
                break


        return path_nodes[::-1]
    
    
    def _has_line_of_sight(self, 
                        src_node: VoxelNode, 
                        dst_node: VoxelNode, 
                        grids: VoxelGrids3D) -> bool:
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
                if grids[x1, y1, z1].is_obstacle: 
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
                if grids[x1, y1, z1].is_obstacle:  
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
                if grids[x1, y1, z1].is_obstacle:  
                    return False
        return True

class PathFinding(ABC):
    def __init__(self, grids: VoxelGrids3D) -> None:
        super().__init__()
        self.grids: VoxelGrids3D = grids 
        self.grids.reset()
    
    @abstractmethod
    def search(self) -> bool:
        pass    
    
    def reset(self) -> None:
        self.grids.reset()  
        return

    def set_start_node(self, node: VoxelNode) -> None:
        self.grids.set_start_node(node)
        return  
    
    def set_goal_node(self, node: VoxelNode) -> None:
        self.grids.set_goal_node(node)
        return
    
    def get_path_nodes(self) -> List[VoxelNode]:
        return PathFollower(self.grids).get_path_nodes()    
    
    def get_smooth_path_nodes(self) -> List[VoxelNode]:
        return PathFollower(self.grids).get_smooth_path_nodes() 

class AStar(PathFinding):
    def __init__(self,
                grids: VoxelGrids3D) -> None:
        super().__init__(grids=grids)  
        
        
    def search(self) -> bool:
        open_list: List[VoxelNode] = []  
        closed_list: set[VoxelNode] = set() 
        
        start_node: VoxelNode = self.grids.start_node    
        heapq.heappush(open_list, start_node)   
        while open_list:
            cur_node: VoxelNode  = heapq.heappop(open_list)    
            closed_list.add(cur_node)    

            if cur_node.is_goal_node:   
                return True 
            
            neighbors: List[VoxelNode] = self._get_neighbors(cur_node)   
            for neighbor in neighbors:  
                if neighbor in closed_list:
                    continue
                
                ng = cur_node.g + cur_node.center_pnt.Distance(neighbor.center_pnt)
                h = neighbor.center_pnt.Distance(self.grids.goal_node.center_pnt)   
                if neighbor.parent == None or neighbor.g > ng:
                    neighbor.g = ng
                    neighbor.f = h + ng
                    neighbor.parent = cur_node
                    heapq.heappush(open_list, neighbor) 
            
        return False    
    
    def _get_neighbors(self, node: VoxelNode) -> List[VoxelNode]:
        grids: VoxelGrids3D = self.grids      
        neighbors: List[VoxelNode] = []  
        
        for dx, dy, dz in product([-1, 0, 1], repeat=3):
            if dx == dy == dz == 0:
                continue
            nxt_i, nxt_j, nxt_k = node.i + dx, node.j + dy, node.k + dz
            if not self._is_valid(nxt_i, nxt_j, nxt_k): 
                continue
            neighbors.append(grids[nxt_i, nxt_j, nxt_k])
        return neighbors
    
    def _is_valid(self, nxt_i: int, nxt_j: int, nxt_k: int) -> bool:
        if not (0 <= nxt_i < self.grids.map_size):
            return False    
        if not (0 <= nxt_j < self.grids.map_size):
            return False
        if not (0 <= nxt_k < self.grids.map_size):
            return False    
        if self.grids[nxt_i, nxt_j, nxt_k].is_obstacle:
            return False
        
        return True
    
class ThetaStar(PathFinding):
    def __init__(self,
                grids: VoxelGrids3D) -> None:
        super().__init__(grids=grids)
        self.grids: VoxelGrids3D = grids
        self.grids.reset()
        
        
    def search(self) -> bool:
        open_list: List[VoxelNode] = []  
        closed_list: set[VoxelNode] = set()
        
        start_node: VoxelNode = self.grids.start_node    
        heapq.heappush(open_list, start_node)   
        while open_list:
            cur_node: VoxelNode = heapq.heappop(open_list)   
            closed_list.add(cur_node)    

            if cur_node.is_goal_node:   
                return True 
            
            neighbors: List[VoxelNode] = self._get_neighbors(cur_node)   
            for successor in neighbors:
                if successor in closed_list:
                    continue
                
                h = successor.center_pnt.Distance(self.grids.goal_node.center_pnt)   
                if cur_node.parent != None and PathFollower(self.grids)._has_line_of_sight(cur_node.parent, successor, self.grids):
                    ng: float = cur_node.parent.g + cur_node.parent.center_pnt.Distance(successor.center_pnt) 
                    if successor.parent == None or ng < successor.g:
                        successor.g = ng
                        successor.f = h + ng
                        successor.parent = cur_node.parent
                        heapq.heappush(open_list, successor)    
                else:
                    ng = cur_node.g + cur_node.center_pnt.Distance(successor.center_pnt)
                    if successor.parent == None or successor.g > ng:
                        successor.g = ng
                        successor.f = h + ng
                        successor.parent = cur_node
                        heapq.heappush(open_list, successor)    
        return False    
    
    def _get_neighbors(self, node: VoxelNode) -> List[VoxelNode]:
        grids: VoxelGrids3D = self.grids      
        neighbors: List[VoxelNode] = []  
        
        for dx, dy, dz in product([-1, 0, 1], repeat=3):
            if dx == dy == dz == 0:
                continue
            nxt_i, nxt_j, nxt_k = node.i + dx, node.j + dy, node.k + dz
            if not self._is_valid(nxt_i, nxt_j, nxt_k): 
                continue
            neighbors.append(grids[nxt_i, nxt_j, nxt_k])
        return neighbors
    
    def _is_valid(self, nxt_i: int, nxt_j: int, nxt_k: int) -> bool:
        if not (0 <= nxt_i < self.grids.map_size):
            return False    
        if not (0 <= nxt_j < self.grids.map_size):
            return False
        if not (0 <= nxt_k < self.grids.map_size):
            return False    
        if self.grids[nxt_i, nxt_j, nxt_k].is_obstacle:
            return False
        
        return True
