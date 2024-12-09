from OCC.Core.gp import gp_Pnt
from OCC.Core.Bnd import Bnd_Box

from src.datastruct.voxel.voxel_grids import VoxelNode, VoxelGrids3D
from src.datastruct.voxel.random_gen import BndObstacleGen, NodePairGen
from src.display.scene import Scene

def create_available_area(corner_max: gp_Pnt = gp_Pnt(25.0, 25.0, 130.0), 
                                    corner_min: gp_Pnt = gp_Pnt(-270.0, -370.0, 5.0)) -> Bnd_Box:   
    available_area = Bnd_Box()
    available_area.SetGap(0.0)  
    available_area.Update(*corner_min.Coord(), *corner_max.Coord())   
    return available_area



if __name__ == "__main__":
    available_are: Bnd_Box = create_available_area()
    obstacle_gen: BndObstacleGen = BndObstacleGen(available_are)
    voxel_grid: VoxelGrids3D = obstacle_gen.generate(map_size=10, n_boxes=2)
    
    scene = Scene('voxel_grids')
    
    for node in voxel_grid:
        if not node.is_obstacle:
            scene.add_entity(node)
            node.color = 'green'
            node.transparency = 0.5
            
    scene.display()