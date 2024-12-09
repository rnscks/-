from OCC.Core.gp import gp_Pnt
from OCC.Core.Bnd import Bnd_Box
from OCC.Core.BRepBndLib import brepbndlib
from OCC.Core.TopoDS import TopoDS_Shape

from typing import List, Tuple, Set, Optional

from src.datastruct.voxel.voxel_grids import VoxelGrids3D

class Panel:
    def __init__(self) -> None:        
        self.grids: Optional[VoxelGrids3D] = None
        return
    
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
        voxel_grids: VoxelGrids3D = VoxelGrids3D(corner_max=voxel_corner_max, corner_min=voxel_corner_min, map_size=resolution) 
        for node in voxel_grids:
            if available_area.IsOut(node.position):
                node.is_obstacle = True
        return voxel_grids
    
    def _create_bnd_box(self, part_model: TopoDS_Shape, padding: float) -> Bnd_Box:
        bnd_box = Bnd_Box()
        brepbndlib.Add(part_model, bnd_box)
        bnd_box.Enlarge(padding)
        return bnd_box    
    
    def set_part_models(self, step_models: List[TopoDS_Shape]) -> None:
        bnd_boxes: List[Bnd_Box] = [self._create_bnd_box(step_model, padding=0.0) for step_model in step_models] 
        
        for bnd_box in bnd_boxes:
            maxx, maxy, maxz = bnd_box.CornerMax().Coord()
            minx, miny, minz = bnd_box.CornerMin().Coord()
            for node in self.grids:
                if (maxx > node.center_pnt.X() > minx) and \
                    (maxy > node.center_pnt.Y() > miny):
                    node.is_obstacle = True
        return

    def set_bnd_obs(self, bnd_boxes: List[Bnd_Box]) -> None:
        for bnd_box in bnd_boxes:
            maxx, maxy, maxz = bnd_box.CornerMax().Coord()
            minx, miny, minz = bnd_box.CornerMin().Coord()  
            for node in self.grids:
                if (maxx > node.center_pnt.X() > minx) and \
                    (maxy > node.center_pnt.Y() > miny):
                    node.is_obstacle = True 
        return  

    def init_voxel_grids(self, available_area: Bnd_Box, resolution: int) -> None:
        self.grids = self._create_voxel_grids(available_area, resolution)
        return  
    
    def set_voxel_grids(self, grids: VoxelGrids3D) -> None:
        self.grids = grids
        return  