from OCC.Core.TopoDS import TopoDS_Shape
from OCC.Core.Bnd import Bnd_Box

from src.cable_routing.routing_component.panel import Panel
from abc import ABC, abstractmethod 
from typing import List

class PreProcessor(ABC):
    def __init__(self) -> None:
        pass
    
    @abstractmethod
    def process(self, **kwargs) -> None:
        pass
    

class Voxelization(PreProcessor):
    def __init__(self) -> None:
        super().__init__()
    
    def process(self, available_area: Bnd_Box,terminal_block: List[TopoDS_Shape]) -> Panel:
        panel = Panel()
        panel.set_voxel_grids(available_area)
        
        return panel

class VoxelizationwithBounding(PreProcessor):
    def __init__(self) -> None:
        super().__init__()
    
    def process(self, available_area: Bnd_Box,terminal_block: List[TopoDS_Shape]) -> Panel:
        panel = Panel()
        panel.set_voxel_grids(available_area)
        panel.set_bnd_obs(terminal_block)
        return panel