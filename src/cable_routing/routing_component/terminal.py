from OCC.Core.gp import gp_Pnt

from typing import List, Tuple, Set, Optional
import pandas as pd

from src.datastruct.voxel_grids import VoxelNode, VoxelGrids3D


class Terminal:
    def __init__(self,
                terminal_pnt: gp_Pnt,
                front_pnt: gp_Pnt,
                terminal_dir: Tuple[int, int, int],
                terminal_node: VoxelNode,
                number: int) -> None:
        self.terminal_pnt: gp_Pnt = terminal_pnt
        self.front_pnt: gp_Pnt = front_pnt
        self.terminal_dir: Tuple[int, int, int] = terminal_dir
        self.number: int = number
        self.terminal_node: VoxelNode = terminal_node
        
    def __eq__(self, value: 'Terminal') -> bool:
        return self.number == value.number

class TerminalReader:
    def __init__(self) -> None:
        pass
    
    def read_terminal_pairs(self,
                grids: VoxelGrids3D,   
                file_name: str,
                io_table:List[Tuple[int, int]]) -> List[Tuple[Terminal, Terminal]]:
        df = pd.read_excel(file_name)
        terminal_pairs: List[Tuple[Terminal, Terminal]] = []   
        io_table: List[Tuple[int, int]] = io_table
        
        socket_pnts: List[gp_Pnt] = [gp_Pnt(x, y, z) for x, y, z in df[['X', 'Y', 'Z']].values]   
        socket_dirs: List[Tuple[int, int, int]] = [(dirx, diry, dirz) for dirx, diry, dirz in df[['DIRX', 'DIRY', 'DIRZ']].values]  
        in_pnts: List[gp_Pnt] = []
        in_gap: float = 10
        
        for pnt, dir in zip(socket_pnts, socket_dirs):  
            in_pnts.append(gp_Pnt(pnt.X() + dir[0] * in_gap, pnt.Y() + dir[1] * in_gap, pnt.Z() + dir[2] * in_gap))    
        
        for input, output in io_table: 
            in_terminal_node: VoxelNode = self._search_node(grids=grids, pnt=socket_pnts[input])
            in_terminal_node = self._translate_node(grids, in_terminal_node, *socket_dirs[input])
            in_terminal = Terminal(
                terminal_pnt=socket_pnts[input],
                terminal_dir=socket_dirs[input],
                front_pnt=in_pnts[input],  
                number=input,
                terminal_node=in_terminal_node)
            
            out_terminal_node: VoxelNode = self._search_node(grids=grids, pnt=socket_pnts[output]) 
            out_terminal_node = self._translate_node(grids, out_terminal_node, *socket_dirs[output])  
            out_terminal = Terminal(
                terminal_pnt=socket_pnts[output],
                terminal_dir=socket_dirs[output],
                front_pnt=in_pnts[output],
                number=output,
                terminal_node=out_terminal_node)
            
            terminal_pairs.append((in_terminal, out_terminal))  
        return terminal_pairs
        
    def read_terminals(self,
                grids: VoxelGrids3D,   
                file_name: str) -> List[Terminal]:
        df = pd.read_excel(file_name)
        
        socket_pnts: List[gp_Pnt] = [gp_Pnt(x, y, z) for x, y, z in df[['X', 'Y', 'Z']].values]   
        socket_dirs: List[Tuple[int, int, int]] = [(dirx, diry, dirz) for dirx, diry, dirz in df[['DIRX', 'DIRY', 'DIRZ']].values]  
        terminals: List[Terminal] = []  
        for idx, socket_pnt, socket_dir in zip(range(len(socket_pnts)), socket_pnts, socket_dirs):    
            terminal_node: VoxelNode = self._search_node(grids=grids, pnt=socket_pnt)   
            terminal_node = self._translate_node(grids, terminal_node, *socket_dir) 
            terminal = Terminal(
                terminal_pnt=socket_pnt,
                terminal_dir=socket_dir,
                terminal_node=terminal_node,
                number=idx)
            terminals.append(terminal)
        return terminals
    
    def _search_node(self,
                    grids: VoxelGrids3D,
                    pnt: gp_Pnt) -> VoxelNode:  
        node: Optional[VoxelNode] = grids[0, 0, 0]
        for other in grids:
            if other.center_pnt.Distance(pnt) < node.center_pnt.Distance(pnt):
                node = other
        return node
    
    def _translate_node(self, 
                        grids: VoxelGrids3D, 
                        node: VoxelNode,
                        dir: Tuple[int, int, int]) -> VoxelNode:
        i,j,k = node.i, node.j, node.k
        dirx, diry, dirz = dir  
        
        while True:
            i, j, k = i + dirx, j + diry, k + dirz
            node = grids[i, j, k]
            if not node.is_obstacle:
                return node