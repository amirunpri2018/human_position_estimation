# Modules
import networkx as nx
import matplotlib.pyplot as plt

# Networkx object
logik_graph = nx.Graph()

# Waypoints
exit             = (7.45, 4.66, "exit")
lab_inside       = (0.42, -0.6, "lab_inside")
lab_outside      = (2.49, -0.43, "lab_outside")
cluster_exit     = (4.0, -3.4, "cluster_exit")
cluster_entrance = (3.23, 2.38, "cluster_entrance")

# Graph's vertices
logik_graph.add_node (exit)
logik_graph.add_node (lab_inside)
logik_graph.add_node (lab_outside)
logik_graph.add_node (cluster_exit)
logik_graph.add_node (cluster_entrance)

# Graph's weights
logik_graph.add_edge(lab_outside, exit, weight = 1)
logik_graph.add_edge(lab_inside, lab_outside, weight = 1)
logik_graph.add_edge(lab_outside, cluster_entrance, weight = 1)
logik_graph.add_edge(cluster_entrance, cluster_exit, weight = 1)
