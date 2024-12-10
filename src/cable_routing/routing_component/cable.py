from OCC.Core.gp import gp_Pnt, gp_Vec, gp_Dir, gp_Ax2, gp_Circ
from OCC.Core.TColgp import TColgp_HArray1OfPnt, TColgp_HArray1OfVec, TColgp_Array1OfPnt
from OCC.Core.TColStd import TColStd_HArray1OfBoolean
from OCC.Core.TopoDS import TopoDS_Shell, TopoDS_Edge, TopoDS_Wire, TopoDS_Shape
from OCC.Core.GeomAPI import GeomAPI_Interpolate, GeomAPI_PointsToBSpline
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire, BRepBuilderAPI_Sewing
from OCC.Core.BRepOffsetAPI import BRepOffsetAPI_MakePipe, BRepOffsetAPI_ThruSections
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Cut    
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeFace
from OCC.Core.GeomFill import GeomFill_Pipe
from OCC.Core.Geom import Geom_Circle


from itertools import zip_longest
from enum import Enum
from typing import List, Optional, Tuple, Set

from src.display.entity import Entity

class BREPMODELING(Enum):   
    TOLENRANCE = 1e-2   

class Cable(Entity):
    def __init__(self):
        super().__init__()
        self.color = 'blue'
        pass
        
    def init_brep_solid(self, 
                        gp_pnts: List[gp_Pnt], 
                        diameter: float = 2.0, 
                        thickness: float = 0.2) -> None:    
        self.pnts: List[gp_Pnt] = gp_pnts
        self.vecs: List[gp_Vec] = []
        
        for pnt in self.pnts[1:-1]:
            nxt = self.pnts.index(pnt) + 1
            self.vecs.append(gp_Vec(pnt, self.pnts[nxt]))
        
        self.vecs.append(gp_Vec(self.pnts[-2], self.pnts[-1]))
        self.vecs.insert(0, gp_Vec(self.pnts[0], self.pnts[1])) 
        
        tcol_pnt = TColgp_HArray1OfPnt(1, len(self.pnts))
        tcol_vec = TColgp_HArray1OfVec(1, len(self.vecs))
        std_bools = TColStd_HArray1OfBoolean(1, len(self.pnts))
        
        for i, pnt, vec in zip_longest(range(1, len(gp_pnts) + 1), self.pnts, self.vecs):
            tcol_pnt.SetValue(i, pnt)
            tcol_vec.SetValue(i, vec)
            std_bools.SetValue(i, True)

        cable_model: TopoDS_Shape = self._create_cable_solid(
            tcol_pnt=tcol_pnt,
            tcol_vec=tcol_vec,
            std_bools=std_bools,
            diameter=diameter,
            thickness=thickness)
        self.brep_solid = cable_model
        return
    
    def _create_cable_solid(self,
                        tcol_pnt: TColgp_HArray1OfPnt,
                        tcol_vec: TColgp_HArray1OfVec,
                        std_bools: TColStd_HArray1OfBoolean,
                        diameter: float,
                        thickness: float) -> Optional[TopoDS_Shape]:
        
        central_curve_wire: TopoDS_Wire = self._create_central_curve_wire(
            tcol_pnt=tcol_pnt,
            tcol_vec=tcol_vec,
            std_bools=std_bools)
        
        outer_cable_shell = self._create_curve_shell(
            central_wire=central_curve_wire,
            center_pnt=tcol_pnt.Value(1),
            normal_dir=gp_Dir(tcol_vec.Value(1)),
            diameter=diameter + thickness)  
        if outer_cable_shell == None:
            print("CableModeling: Outer Cable Shell is None.")
            return None 
        

        
        return outer_cable_shell

    def _create_central_curve_wire(self,
                            tcol_pnt: TColgp_HArray1OfPnt,
                            tcol_vec: TColgp_HArray1OfVec,
                            std_bools: TColStd_HArray1OfBoolean) -> Optional[TopoDS_Shell]:
        if tcol_pnt.Length() < 2:   
            raise ValueError("LineBuilder: pnt_list should have at least 2 points.")
        
        tolenrance = BREPMODELING.TOLENRANCE.value
        central_curve_edge: Optional[TopoDS_Edge] = None
        try:
            interpolate = GeomAPI_Interpolate(tcol_pnt, False, tolenrance)
            interpolate.Load(tcol_vec, std_bools)
            interpolate.Perform()
            if (interpolate.IsDone()):
                interpolated_curve = interpolate.Curve()
                central_curve_edge = BRepBuilderAPI_MakeEdge(interpolated_curve).Edge()
        except RuntimeError:
            print("CableModeling: RuntimeError[Central Curve Edge]")
            return None
        
        if central_curve_edge is None:  
            return None 
        return BRepBuilderAPI_MakeWire(central_curve_edge).Wire()   

    def _create_curve_shell(self, 
                        central_wire: TopoDS_Wire,
                        center_pnt: gp_Pnt, 
                        normal_dir: gp_Dir,
                        diameter: float = 2.0) -> Optional[TopoDS_Shell]:
        if central_wire is None:
            raise ValueError("BRepModeling: Central curve wire is None.")
        
        initial_section_axis = gp_Ax2(center_pnt, normal_dir)        
        initial_section = gp_Circ(initial_section_axis, diameter / 2)    
        circle_edge: TopoDS_Edge = BRepBuilderAPI_MakeEdge(initial_section).Edge()    
        circle_wire: TopoDS_Wire = BRepBuilderAPI_MakeWire(circle_edge).Wire()   

        try:
            return BRepOffsetAPI_MakePipe(central_wire, circle_wire).Shape()
        except RuntimeError:
            print("BRepModeling: RuntimeError[Pipe Shape]")
            return None
        
    def _create_circle_shell(self,
                        center_pnt: gp_Pnt,    
                        normal_dir: gp_Dir, 
                        diameter: float) -> TopoDS_Shape:    
        circle_axis = gp_Ax2(center_pnt, normal_dir)  
        ternal_circle = gp_Circ(circle_axis, diameter / 2)  
        ternal_edge = BRepBuilderAPI_MakeEdge(ternal_circle).Edge()
        ternal_wire = BRepBuilderAPI_MakeWire(ternal_edge).Wire()
        ternal_face = BRepBuilderAPI_MakeFace(ternal_wire, True).Face()
        sewing_builder = BRepBuilderAPI_Sewing()

        sewing_builder.Add(ternal_face)   
        sewing_builder.Perform()
        
        return sewing_builder.SewedShape()   
    
    def _create_hole_shell(self,
                        center_pnt: gp_Pnt,    
                        normal_dir: gp_Dir, 
                        diameter: float,
                        thickness: float) -> TopoDS_Shape:  
        inner_shell = self._create_circle_shell(
            center_pnt=center_pnt,
            normal_dir=normal_dir,
            diameter=diameter)
        
        outer_shell = self._create_circle_shell(    
            center_pnt=center_pnt,    
            normal_dir=normal_dir,
            diameter=diameter + thickness)
        hole_shell = BRepAlgoAPI_Cut(outer_shell, inner_shell).Shape()
        
        return hole_shell

class SimpleCable(Entity):
    def __init__(self):
        super().__init__()
        self.color = 'blue'
        pass
        
    def init_brep_solid(self, 
                        gp_pnts: List[gp_Pnt], 
                        diameter: float = 2.0, 
                        thickness: float = 0.2) -> None:    
        self.pnts: List[gp_Pnt] = gp_pnts
        self.vecs: List[gp_Vec] = []
        
        for pnt in self.pnts[1:-1]:
            nxt = self.pnts.index(pnt) + 1
            self.vecs.append(gp_Vec(pnt, self.pnts[nxt]))
        
        self.vecs.append(gp_Vec(self.pnts[-2], self.pnts[-1]))
        self.vecs.insert(0, gp_Vec(self.pnts[0], self.pnts[1])) 
        
        tcol_pnt = TColgp_HArray1OfPnt(1, len(self.pnts))
        tcol_vec = TColgp_HArray1OfVec(1, len(self.vecs))
        std_bools = TColStd_HArray1OfBoolean(1, len(self.pnts))
        
        for i, pnt, vec in zip_longest(range(1, len(gp_pnts) + 1), self.pnts, self.vecs):
            tcol_pnt.SetValue(i, pnt)
            tcol_vec.SetValue(i, vec)
            std_bools.SetValue(i, True)

        cable_model: TopoDS_Shape = self._create_cable_solid(
            tcol_pnt=tcol_pnt,
            tcol_vec=tcol_vec,
            std_bools=std_bools,
            diameter=diameter,
            thickness=thickness)
        self.brep_solid = cable_model
        return
    
    def _create_cable_solid(self,
                        tcol_pnt: TColgp_HArray1OfPnt,
                        tcol_vec: TColgp_HArray1OfVec,
                        std_bools: TColStd_HArray1OfBoolean,
                        diameter: float,
                        thickness: float) -> Optional[TopoDS_Shape]:
        
        central_curve_wire: TopoDS_Wire = self._create_central_curve_wire(
            tcol_pnt=tcol_pnt,
            tcol_vec=tcol_vec,
            std_bools=std_bools)
        

        
        outer_cable_shell = self._create_curve_shell(
            central_wire=central_curve_wire,
            center_pnt=tcol_pnt.Value(1),
            normal_dir=gp_Dir(tcol_vec.Value(1)),
            diameter=diameter + thickness)  
        if outer_cable_shell == None:
            print("CableModeling: Outer Cable Shell is None.")
            return None 
        
        return outer_cable_shell

    def _create_central_curve_wire(self,
                            tcol_pnt: TColgp_HArray1OfPnt,
                            tcol_vec: TColgp_HArray1OfVec,
                            std_bools: TColStd_HArray1OfBoolean) -> Optional[TopoDS_Wire]:
        if tcol_pnt.Length() < 2:   
            raise ValueError("LineBuilder: pnt_list should have at least 2 points.")
        
        # TColgp_HArray1OfPnt를 TColgp_Array1OfPnt로 변환
        array_points = TColgp_Array1OfPnt(1, tcol_pnt.Length())
        for i in range(1, tcol_pnt.Length() + 1):
            array_points.SetValue(i, tcol_pnt.Value(i))
        
        try:
            # GeomAPI_PointsToBSpline을 사용하여 B-Spline 곡선 생성
            curve_builder = GeomAPI_PointsToBSpline(array_points)
            if curve_builder.IsDone():
                curve = curve_builder.Curve()
                central_curve_edge = BRepBuilderAPI_MakeEdge(curve).Edge()
                return BRepBuilderAPI_MakeWire(central_curve_edge).Wire()
        except RuntimeError:
            print("CableModeling: RuntimeError[Central Curve Edge]")
            return None
        
        return None

    def _create_curve_shell(self, 
                        central_wire: TopoDS_Wire,
                        center_pnt: gp_Pnt, 
                        normal_dir: gp_Dir,
                        diameter: float = 2.0) -> Optional[TopoDS_Shell]:
        if central_wire is None:
            raise ValueError("BRepModeling: Central curve wire is None.")
        
        initial_section_axis = gp_Ax2(center_pnt, normal_dir)        
        initial_section = gp_Circ(initial_section_axis, diameter / 2)    
        circle_edge: TopoDS_Edge = BRepBuilderAPI_MakeEdge(initial_section).Edge()    
        circle_wire: TopoDS_Wire = BRepBuilderAPI_MakeWire(circle_edge).Wire()   

        try:
            return BRepOffsetAPI_MakePipe(central_wire, circle_wire).Shape()
        except RuntimeError:
            print("BRepModeling: RuntimeError[Pipe Shape]")
            return None
        
    def _create_circle_shell(self,
                        center_pnt: gp_Pnt,    
                        normal_dir: gp_Dir, 
                        diameter: float) -> TopoDS_Shape:    
        circle_axis = gp_Ax2(center_pnt, normal_dir)  
        ternal_circle = gp_Circ(circle_axis, diameter / 2)  
        ternal_edge = BRepBuilderAPI_MakeEdge(ternal_circle).Edge()
        ternal_wire = BRepBuilderAPI_MakeWire(ternal_edge).Wire()
        ternal_face = BRepBuilderAPI_MakeFace(ternal_wire, True).Face()
        sewing_builder = BRepBuilderAPI_Sewing()

        sewing_builder.Add(ternal_face)   
        sewing_builder.Perform()
        
        return sewing_builder.SewedShape()   
    
    def _create_hole_shell(self,
                        center_pnt: gp_Pnt,    
                        normal_dir: gp_Dir, 
                        diameter: float,
                        thickness: float) -> TopoDS_Shape:  
        inner_shell = self._create_circle_shell(
            center_pnt=center_pnt,
            normal_dir=normal_dir,
            diameter=diameter)
        
        outer_shell = self._create_circle_shell(    
            center_pnt=center_pnt,    
            normal_dir=normal_dir,
            diameter=diameter + thickness)
        hole_shell = BRepAlgoAPI_Cut(outer_shell, inner_shell).Shape()
        
        return hole_shell