"""
    Plot the custom topological
    graph created.
"""

# Modules
import networkx as nx
from upstairs import graph
import matplotlib.pyplot as plt

# positions = {}
# node_labels = {}
#
# for node in graph.nodes():
#     positions[node] = (-node[1], node[0])
#     node_labels[node] = node[2]

nx.draw(graph, with_labels = True)

plt.show()
