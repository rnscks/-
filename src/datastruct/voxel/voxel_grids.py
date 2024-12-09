from OCC.Core.gp import gp_Pnt
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox

from typing import Tuple, Optional, List, Set

from src.display.entity import Entity

class VoxelNode(Entity):
    def __init__(self, i: int = 0, j: int = 0, k: int = 0) -> None:
        super().__init__()
        self.i: int = i
        self.j: int = j
        self.k: int = k
        
        self.f: float = 0.0
        self.g: float = 0.0
        
        self.parent: Optional[VoxelNode] = None
        self.is_start_node: bool = False   
        self.is_goal_node: bool = False
        self.is_obstacle: bool = False
        self.position: gp_Pnt = gp_Pnt(0, 0, 0)
    
    
    def copy_from(self, other: "VoxelNode") -> None:
        self.i, self.j, self.k = other.i, other.j, other.k
        self.f, self.g = other.f, other.g
        self.parent = other.parent
        self.is_start_node = other.is_start_node
        self.is_goal_node = other.is_goal_node
        self.is_obstacle = other.is_obstacle
        return
    
    def reset(self) -> None:
        self.parent = None
        self.f, self.g = 0.0, 0.0
        return

    def __eq__(self, other: object) -> bool:
        if isinstance(other, VoxelNode):
            return self.i == other.i and self.j == other.j and self.k == other.k
        else:
            return False        
        
    def __hash__(self) -> int:  
        return hash((self.i, self.j, self.k))
    
    def __str__(self) -> str:
        return f"Voxel Node ({self.i},{self.j},{self.k}), g = {self.g}, f = {self.f}"
    
    def __lt__(self, other: "VoxelNode") -> bool:
        return self.f < other.f

    def init_brep_solid(self, corner_max: gp_Pnt, corner_min: gp_Pnt) -> None:
        box_solid = BRepPrimAPI_MakeBox(corner_max, corner_min).Shape()
        self.set_brep_solid(box_solid)  
        self.position = gp_Pnt((corner_max.X() + corner_min.X()) / 2,   
                               (corner_max.Y() + corner_min.Y()) / 2, 
                               (corner_max.Z() + corner_min.Z()) / 2)   
        self.center_pnt = self.position
        return

# TEST CODE기 작성되어 있으므로, 수정 시 TEST CODE를 함께 수정해야 함
class VoxelGrids3D:
    def __init__(self, 
                corner_max: gp_Pnt, 
                corner_min: gp_Pnt, 
                map_size: int) -> None:
        super().__init__()
        gap: float = corner_max.X() - corner_min.X()
        self.start_node: Optional[VoxelNode] = None
        self.goal_node: Optional[VoxelNode] = None
        self.map_size: int = map_size   
        self.nodes_map: List[List[List[VoxelNode]]]  = \
            [[[VoxelNode(i, j, k) for k in range(map_size)] for j in range(map_size)] for i in range(map_size)] 
            
        node_gap: float = gap / map_size
        for i in range(map_size):
            for j in range(map_size):
                for k in range(map_size):
                    node_corner_min = gp_Pnt(corner_min.X() + i * node_gap, 
                                             corner_min.Y() +  j * node_gap, 
                                             corner_min.Z() +  k * node_gap)
                    node_corner_max = gp_Pnt(corner_min.X() + (i + 1) * node_gap, 
                                             corner_min.Y() + (j + 1) * node_gap, 
                                             corner_min.Z() + (k + 1) * node_gap)
                    self.nodes_map[i][j][k].init_brep_solid(node_corner_min, node_corner_max)  
    
    
    def reset(self) -> None:    
        for node in self:
            node.reset()
        return
    
    def set_start_node(self, node: VoxelNode) -> None:
        if isinstance(self.start_node, VoxelNode):
            self.start_node.is_start_node = False
        self.start_node = node
        node.is_start_node = True
        node.is_obstacle = False
        return
    
    def set_goal_node(self, node: VoxelNode) -> None:
        if isinstance(self.goal_node, VoxelNode):
            self.goal_node.is_goal_node = False
        self.goal_node = node
        node.is_goal_node = True
        node.is_obstacle = False    
        return  
    
    def __iter__(self):
        for i in range(self.map_size):
            for j in range(self.map_size):
                for k in range(self.map_size):
                    yield self.nodes_map[i][j][k]    
                    
    def __getitem__(self, index: Tuple[int, int, int]) -> VoxelNode:
        return self.nodes_map[index[0]][index[1]][index[2]]
        
    def __len__(self):
        return len(self.nodes_map)