from OCC.Core.gp import gp_Pnt
from OCC.Core.Bnd import Bnd_Box

from src.datastruct.voxel_grids import VoxelNode, VoxelGrids3D
from src.datastruct.random_gen import BndObstacleGen, NodePairGen
from src.cable_routing.routing_component.cable import Cable, SimpleCable
from src.display.scene import Scene

def create_available_area(corner_max: gp_Pnt = gp_Pnt(25.0, 25.0, 130.0), 
                        corner_min: gp_Pnt = gp_Pnt(-270.0, -370.0, 5.0)) -> Bnd_Box:   
    available_area = Bnd_Box()
    available_area.SetGap(0.0)  
    available_area.Update(*corner_min.Coord(), *corner_max.Coord())   
    return available_area



if __name__ == "__main__":
    cable = SimpleCable()
    cable.init_brep_solid(
        gp_pnts=[gp_Pnt(0, 0, 0), gp_Pnt(0, 0, 10), gp_Pnt(0, 10, 10)],
        diameter=2,
        thickness=1)
    cable.display()