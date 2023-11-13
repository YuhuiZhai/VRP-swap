import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
class Node:
    def __init__(self, id, pos:np.ndarray, weight:float):
        # pos is the position of this node
        self.id = id
        self.pos = pos
        self.weight = weight
        self.status = 0
    
    def get_id(self):
        return self.id
    
    def get_pos(self):
        return self.pos
    
    def get_weight(self):
        return self.weight
    
    def get_status(self):
        return self.status
    
    def set_status(self, new_status):
        self.status = new_status

class Edge:
    def __init__(self, id, node1:Node, node2:Node):
        self.id = id
        self.node1 = node1
        self.node2 = node2
        self.l = np.linalg.norm(node1.get_pos()-node2.get_pos(), 2)
    
    def get_id(self):
        return self.id
    
    def get_length(self):
        return self.l

    def get_nodes(self):
        return self.node1, self.node2

# Assume a complete graph 
# nodes is a (N, d) np array, first two dimensions represent node poistions, third dimension represents weights
class Graph:
    def __init__(self, nodes:np.ndarray):
        # mapping from id to nodes / mapping from nodes to id
        self.id_V = {i:Node(id=i, pos=nodes[i, :2], weight=nodes[i, 2]) for i in range(nodes.shape[0])}
        self.V_id = {self.id_V[id]:id for id in self.id_V.keys()}
        self.V = set(self.id_V.values())
        
        # mapping from id to edges / mapping from edges to id
        self.id_E = {(i*len(self.V)+j):Edge(id=(i*len(self.V)+j), node1=self.id_V[i], node2=self.id_V[j]) 
                     for i in range(len(self.V)) for j in range(len(self.V)) if i != j}
        self.E_id = {self.id_E[edge_id]:edge_id for edge_id in self.id_E.keys()}
        self.E = set(self.id_E.values())
        
        self.nodes_E_single = {edge.get_nodes():edge for edge in self.E_id.keys()}
        self.nodes_E_double = self.nodes_E_single | {(edge.get_nodes()[1], edge.get_nodes()[0]):edge for edge in self.E_id.keys()}
        
        
    # depot is assumed to have id = 0
    def get_depot(self) -> Node:
        return self.id_V[0]
    
    def get_node_by_id(self, id) -> Node:
        return self.id_V[id]
    
    def get_id_by_node(self, node:Node):
        return self.V_id[node]
    
    def get_edge_by_id(self, id) -> Edge:
        return self.id_E[id]
    
    def get_id_by_edge(self, edge:Edge):
        return self.E_id[edge]
    
    def get_edge_by_id(self, node1:Node, node2:Node) -> Edge:
        return self.nodes_E_double[(node1, node2)]
    
    def get_nodes(self) -> set:
        return self.V

    def get_edges(self) -> set:
        return self.E
    
    def get_nodes_without_depot(self) -> set:
        return self.V - { self.get_depot() } 
    
    def draw(self):
        nxgraph = nx.Graph()
        node_name = [i for i in self.id_V.keys()]
        node_pos = [tuple(i.get_pos().tolist()) for i in self.id_V.values()]
        edge_name = [(nodes[0].get_id(), nodes[1].get_id()) for nodes in self.nodes_E_single.keys()]
        nxgraph.add_nodes_from(node_name)
        nxgraph.add_edges_from(edge_name)
        plt.cla()
        nx.draw(nxgraph, node_pos)
        plt.xlabel("x")
        plt.ylabel("y")
        plt.title("Complete graph")
        plt.show()
        
