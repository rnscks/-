from OCC.Core.gp import gp_Pnt
from typing import List, Tuple, Set
from abc import ABC, abstractmethod

from src.algorithm.pathfinding import PathFinding
from src.datastruct.voxel_grids import VoxelNode
from src.cable_routing.routing_component.terminal import Terminal
from src.cable_routing.routing_component.cable import Cable, SimpleCable


class Router(ABC):
    def __init__(self) -> None:
        pass        
    
    
    @abstractmethod
    def route(self,
                terminal_pairs: Tuple[Terminal, Terminal], 
                diameter: float = 2,
                thickness: float = 1) -> Cable:
        pass

class CableModelingwithSlope(Router):    
    def __init__(self) -> None:
        super().__init__()
    
    
    def route(self,
            pathfinder: PathFinding,
            terminal_pairs: Tuple[Terminal, Terminal],  
            diameter: float = 2,  
            thickness: float = 1) -> Cable:    
        src_terminal, dst_terminal = terminal_pairs 
        pathfinder.reset()
        pathfinder.set_goal_node(dst_terminal.terminal_node)  
        pathfinder.set_start_node(src_terminal.terminal_node) 
        
        if pathfinder.search():
            path_nodes: List[VoxelNode] =  pathfinder.get_smooth_path_nodes()
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

class BasicCableModeling(Router):    
    def __init__(self) -> None:
        super().__init__()
    
    
    def route(self,
            pathfinder: PathFinding,
            terminal_pairs: Tuple[Terminal, Terminal],  
            diameter: float = 2,  
            thickness: float = 1) -> Cable:    
        src_terminal, dst_terminal = terminal_pairs 
        pathfinder.reset()
        pathfinder.set_goal_node(dst_terminal.terminal_node)  
        pathfinder.set_start_node(src_terminal.terminal_node) 
        
        if pathfinder.search():
            path_nodes: List[VoxelNode] =  pathfinder.get_smooth_path_nodes()
            path_pnts: List[gp_Pnt] = []
            path_pnts.append(src_terminal.terminal_pnt)
            path_pnts.append(src_terminal.front_pnt)
            
            for node in path_nodes:
                path_pnts.append(node.center_pnt)
            
            path_pnts.append(dst_terminal.front_pnt)
            path_pnts.append(dst_terminal.terminal_pnt)
            cable = SimpleCable()
            cable.init_brep_solid(
                control_pnts=path_pnts,
                diameter=diameter,
                thickness=thickness)
            return cable
        else:
            return ValueError("Path not found")