import networkx as nx
import matplotlib.pyplot as plt

""" this is super rudimentary and should be updated asap
    by someone after the map is updated """

graph = nx.Graph()
lab_inside    = ( 0.42,  -0.6, "lab_inside")
lab_outside   = ( 2.49, -0.43, "lab_outside")
exit          = ( 7.45,  4.66, "exit")
cluster_entrance    = ( 3.23,  2.38, "cluster_entrance")
cluster_exit  = ( 4.0,   -3.4, "cluster_exit")

graph.add_node (lab_inside)
graph.add_node (lab_outside)
graph.add_node (exit)
graph.add_node (cluster_entrance)
graph.add_node (cluster_exit)

graph.add_edge(lab_inside,   lab_outside,        weight = 1)

graph.add_edge(lab_outside,   cluster_entrance,     weight = 1)
graph.add_edge(lab_outside, exit,     weight = 1)

graph.add_edge(cluster_entrance, cluster_exit,     weight = 1)
