# Modules
import networkx as nx
import matplotlib.pyplot as plt

# Networkx object
simulator_graph = nx.Graph()

# Waypoints
point1           = (-2.4, -0.124, "point1")
point2           = (1.24, -1.41, "point2")
point3           = (3.68, -6.6, "point3")
point4           = (-3.26, -8.92, "point4")

# Graph's vertices
simulator_graph.add_node(point1)
simulator_graph.add_node(point2)
simulator_graph.add_node(point3)
simulator_graph.add_node(point4)

# Graph's weights
simulator_graph.add_edge(point1, point2, weight = 1)
simulator_graph.add_edge(point2, point3, weight = 1)
simulator_graph.add_edge(point3, point4, weight = 1)
