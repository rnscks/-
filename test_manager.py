from OCC.Core.gp import gp_Pnt  
from OCC.Core.Bnd import Bnd_Box

from src.datastruct.voxel_grids import VoxelGrids3D
from src.datastruct.random_gen import BndObstacleGen, NodePairGen
from src.cable_routing.routing_component.cable import Cable, SimpleCable
from src.cable_routing.modeling import BasicCableModeling
from src.algorithm.pathfinding import ThetaStar, AStar

def create_available_area(corner_max: gp_Pnt = gp_Pnt(25.0, 25.0, 130.0), 
                        corner_min: gp_Pnt = gp_Pnt(-270.0, -370.0, 5.0)) -> Bnd_Box:   
    available_area = Bnd_Box()
    available_area.SetGap(0.0)  
    available_area.Update(*corner_min.Coord(), *corner_max.Coord())   
    return available_area

class TestDisplayA:
    def __init__(self):
        voxel_gen = BndObstacleGen(create_available_area())
        self.voxel_grids: VoxelGrids3D = voxel_gen.generate(10, n_boxes=1)
        self.astar = AStar(self.voxel_grids)
        self.router = BasicCableModeling()
        
    def test(self) -> None: 
        self.router.route(
            self.astar
        ).display()


if __name__ == "__main__":
    TestDisplayA().test()   