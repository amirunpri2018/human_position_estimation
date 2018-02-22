# Modules
import networkx as nx
import matplotlib.pyplot as plt

# Networkx object
graph = nx.Graph()

# Waypoints
point1           = (-2.4, -0.124, "point1")
point2           = (1.24, -1.41, "point2")
point3           = (3.68, -6.6, "point3")
point4           = (-3.26, -8.92, "point4")

# Graph's vertices
graph.add_node(point1)
graph.add_node(point2)
graph.add_node(point3)
graph.add_node(point4)

# Graph's weights
graph.add_edge(point1, point2, weight = 1)
graph.add_edge(point2, point3, weight = 1)
graph.add_edge(point3, point4, weight = 1)
