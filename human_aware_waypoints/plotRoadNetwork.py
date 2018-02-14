# graph libraries
import networkx as nx
from upstairs import graph
import matplotlib.pyplot as plt

""" this is useful when you're drawing out maps, it visualises where the
    nodes are to help make sure you don't miss any """


# this program just plots out the road network with labels

# positions = {}
# node_labels = {}
#
# for node in graph.nodes():
#     positions[node] = (-node[1], node[0])
#     node_labels[node] = node[2]

nx.draw(graph, with_labels = True)#, pos = positions, labels = node_labels)

plt.show()
