from OCC.Display.SimpleGui import init_display    
from OCC.Core.Quantity import Quantity_NOC_WHITE, Quantity_Color

from typing import List

from src.display.entity import Entity    


class Scene:
    def __init__(self, name: str) -> None:
        self.entities: List[Entity] = []
        self.name: str = name   
    
    def add_entity(self, entity: Entity) -> None:
        self.entities.append(entity)
        return
    
    def remove_entity(self, entity: Entity) -> None:
        self.entities.remove(entity)
        return
    
    def display(self) -> None:
        display, start_display, add_menu, add_function_to_menu = init_display()
        
        display.View.SetBgGradientColors(
            Quantity_Color(Quantity_NOC_WHITE),
            Quantity_Color(Quantity_NOC_WHITE),
            2,
            True)

        for entity in self.entities:
            display.DisplayShape(entity.brep_solid, update=True, color=entity.color, transparency=entity.transparency)  
            if entity.msg != "":
                display.DisplayMessage(entity.center_pnt, entity.msg)
        start_display() 
        return  
    
    def __iter__(self):
        return iter(self.entities)