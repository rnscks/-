from OCC.Core.gp import gp_Pnt
from typing import List, Tuple, Set
from abc import ABC, abstractmethod

from algorithm.pathfinding.pathfinding import AStar, PathFollower
from src.datastruct.voxel.voxel_grids import VoxelGrids3D, VoxelNode
from src.cable_routing.routing_component.terminal import Terminal
from src.cable_routing.routing_component.cable import Cable


class Router(ABC):
    def __init__(self,
                grids: VoxelGrids3D) -> None:
        self.grids: VoxelGrids3D = grids
        
    @abstractmethod
    def route(self,
                terminal_pairs: Tuple[Terminal, Terminal], 
                diameter: float = 2,
                thickness: float = 1) -> Cable:
        pass

class PathFindingRouter(Router):    
    def __init__(self, grids: VoxelGrids3D) -> None:
        super().__init__(grids)
    
    
    def route(self,
            terminal_pair: Tuple[Terminal, Terminal],
            diameter: float = 2,  
            thickness: float = 1) -> Cable:    
        self.grids.reset()
        src_terminal, dst_terminal = terminal_pair
        self.grids.set_goal_node(dst_terminal.terminal_node)
        self.grids.set_start_node(src_terminal.terminal_node)    
        astar = AStar(self.grids)
        
        if astar.search():
            path_nodes: List[VoxelNode] =  astar.get_smooth_path_nodes()
            path_pnts: List[gp_Pnt] = []
            path_pnts.append(src_terminal.terminal_pnt)
            path_pnts.append(src_terminal.front_pnt)
            
            for node in path_nodes:
                path_pnts.append(node.center_pnt)
            
            path_pnts.append(dst_terminal.front_pnt)
            path_pnts.append(dst_terminal.terminal_pnt)
            cable = Cable()
            cable.init_brep_solid(
                control_pnts=path_pnts,
                diameter=diameter,
                thickness=thickness)
            return cable
        else:
            return ValueError("Path not found")

class RLRouter(Router):
    def __init__(self, grids: VoxelGrids3D, rl_model: str) -> None:
        super().__init__(grids)
        pass