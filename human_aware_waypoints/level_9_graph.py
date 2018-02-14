import networkx as nx
import matplotlib.pyplot as plt

""" this is super rudimentary and should be updated asap
    by someone after the map is updated """

graph = nx.Graph()
robotics_lab    = (-0.5,  0.0, "robotics_lab")
matteo_inside   = ( 7.3,  5.7, "matteo_inside")
matteo_outside  = ( 3.5,  5.7, "matteo_outside")
tony_outside    = ( 3.4, -2.0, "tony_outside")
mehmet_outside  = ( 3.5, 11.8, "mehmet_outside")
east_door_east  = ( 3.2,  3.5, "east_door_east")
east_door_west  = ( 3.2,  6.0, "east_door_west")

floor_bump_east = ( 3.3, 14.2, "floor_bump_east")
floor_bump_west = ( 3.3, 16.4, "floor_bump_west")

staff_room_outside_east = ( 3.0,  6.0, "staff_room_outside_east")
staff_room_inside_east  = ( 1.0,  6.0, "staff_room_inside_east")
staff_room_outside_west = ( 3.0, 14.4, "staff_room_outside_west")
staff_room_inside_west  = ( 1.7, 14.2, "staff_room_inside_west")
staff_room_kitchen_east = (-2.8, 14.5, "staff_room_kitchen_east")
staff_room_kitchen_west = (-3.7, 16.3, "staff_room_kitchen_west")
kitchen_outside         = ( 3.3, 15.7, "kitchen_outside")
kitchen_inside          = ( 1.0, 16.2, "kitchen_inside")

graph.add_node (robotics_lab)
graph.add_node (matteo_inside)
graph.add_node (matteo_outside)
graph.add_node (tony_outside)
graph.add_node (mehmet_outside)
graph.add_node (east_door_east)
graph.add_node (east_door_west)
graph.add_node (floor_bump_east)
graph.add_node (floor_bump_west)

graph.add_node (staff_room_outside_east)
graph.add_node (staff_room_inside_east)
graph.add_node (staff_room_outside_west)
graph.add_node (staff_room_inside_west)
graph.add_node (staff_room_kitchen_east)
graph.add_node (staff_room_kitchen_west)
graph.add_node (kitchen_outside)
graph.add_node (kitchen_inside)


graph.add_edge(robotics_lab,   east_door_east,   weight = 1)
graph.add_edge(robotics_lab,   tony_outside,     weight = 1)
graph.add_edge(east_door_east, tony_outside,     weight = 1)

graph.add_edge(east_door_east, east_door_west,   weight = 1)

graph.add_edge(east_door_west, matteo_outside,   weight = 1)
graph.add_edge(east_door_west, mehmet_outside,   weight = 1)
graph.add_edge(matteo_outside, mehmet_outside,   weight = 1)
graph.add_edge(matteo_outside, matteo_inside,    weight = 1)

graph.add_edge(floor_bump_east, floor_bump_west, weight = 1)
graph.add_edge(floor_bump_east, east_door_west,  weight = 1)
graph.add_edge(floor_bump_east, matteo_outside,  weight = 1)
graph.add_edge(floor_bump_east, mehmet_outside,  weight = 1)

graph.add_edge(staff_room_outside_east, staff_room_inside_east, weight = 1)
graph.add_edge(staff_room_outside_east, matteo_outside,  weight = 1)
graph.add_edge(staff_room_outside_east, mehmet_outside,  weight = 1)
graph.add_edge(staff_room_outside_east, floor_bump_east, weight = 1)
graph.add_edge(staff_room_outside_east, east_door_west,  weight = 1)
graph.add_edge(staff_room_inside_west, staff_room_inside_east, weight = 1)

graph.add_edge(staff_room_outside_west, staff_room_inside_west, weight = 1)
graph.add_edge(staff_room_outside_west, floor_bump_east,  weight = 1)
graph.add_edge(staff_room_outside_west, mehmet_outside,  weight = 1)
graph.add_edge(staff_room_outside_west, floor_bump_east, weight = 1)
graph.add_edge(staff_room_outside_west, east_door_west,  weight = 1)

graph.add_edge(staff_room_inside_west, staff_room_kitchen_east, weight = 1)
graph.add_edge(staff_room_inside_east, staff_room_kitchen_east,  weight = 1)
graph.add_edge(staff_room_kitchen_east, staff_room_kitchen_west,  weight = 1)
graph.add_edge(staff_room_kitchen_west, kitchen_inside, weight = 1)
graph.add_edge(kitchen_outside, kitchen_inside,  weight = 1)
graph.add_edge(kitchen_outside, floor_bump_east,  weight = 1)
graph.add_edge(kitchen_outside, floor_bump_west,  weight = 1)
