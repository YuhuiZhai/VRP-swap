#!/usr/bin/env python
# coding: utf-8

# In[1]:
import numpy as np
import networkx as nx
from geometry import *
from vehicle import *

class VRP_generator:
    def __init__(self, speed, vehicle_capacity):
        self.speed = speed
        self.vehicle_capacity = vehicle_capacity
        return 
    
    def set_graph(self, graph:Graph):
        self.graph = graph
    
    def get_speed(self):
        return self.speed
    
    def get_vehicle_capacity(self):
        return self.vehicle_capacity
    
    def draw_VRP(self, fleet:set, vrp_method=""):
        node = [0]
        edge = []
        pos = [tuple(self.graph.get_depot().get_pos().tolist())]
        for vehicle in fleet:
            assert type(vehicle) == Vehicle
            path = vehicle.get_path()
            for idx in range(len(path)-1):
                node1 = path[idx]
                node2 = path[idx+1]
                assert type(node1) == Node
                assert type(node2) == Node
                node_idx = len(node)
                if idx == 0:
                    edge.append((0, node_idx))
                elif idx == len(path)-2:
                    node.append(node_idx)
                    pos.append(tuple(path[idx].get_pos().tolist()))
                    edge.append((node_idx, 0))
                else:
                    node.append(node_idx)
                    pos.append(tuple(path[idx].get_pos().tolist()))
                    edge.append((node_idx, node_idx+1))
        g = nx.Graph()
        g.add_nodes_from(node)
        g.add_edges_from(edge)
        plt.cla()
        nx.draw(G=g, pos=pos, with_labels=True, node_size=100)
        plt.title(f"VRP routes by {vrp_method}")
        plt.show()
        
    def assert_VRP(self, supply_nodes:set, fleet:set):
        assert type(fleet) == set
        assigned_nodes = {self.graph.get_depot()}
        
        for vehicle in fleet:
            # test the type of vehicle
            assert type(vehicle) == Vehicle
            
            # assert the first node and last node should be the same as depot
            path = vehicle.get_path()     
            assert (path[0] == path[-1])
            assert (path[0] == self.graph.get_depot())
            
            # test every vehicle is assigned with at least three nodes
            assert len(path) >= 3
            
            # test every node is added only once
            path = set(path)    
            prev_count = len(assigned_nodes)
            assigned_nodes = assigned_nodes | path
            after_count = len(assigned_nodes)
            assert after_count == prev_count + len(path) - 1
            
            # test the capacity limit
            total_weight = sum([node.get_weight() for node in path])
            assert total_weight <= self.get_vehicle_capacity()
            
        # test all nodes are assigned
        diff = supply_nodes - assigned_nodes
        assert len(diff) == 0
        
        print("Pass all tests")
    
    def sample_test_case(self, test_vrp_method="NNS"):
        N = 10
        phi = 5
        speed = 5
        vehicle_capacity = 30
        nodes = np.zeros((N, 3))
        nodes[:, :2] = phi*np.random.random((N, 2))
        nodes[:, 2] = np.random.randint(1, 10)
        # this is depot
        nodes[0, :] = np.array([0, 0, 0])
        g = Graph(nodes)
        g.draw()
        self.set_graph(g)
        supply_nodes = g.get_nodes_without_depot()
        
        if test_vrp_method == "NNS":
            fleet = self.VRP_NNS(time=0, supply_nodes=supply_nodes)
        elif test_vrp_method == "Saving":
            fleet = self.VRP_Saving(time=0, supply_nodes=supply_nodes)
        elif test_vrp_method == "NI":
            fleet = self.VRP_NI(time=0, supply_nodes=supply_nodes)
        elif test_vrp_method == "FI":
            fleet = self.VRP_FI(time=0, supply_nodes=supply_nodes)
        elif test_vrp_method == "Sweep":
            fleet = self.VRP_Sweep(time=0, supply_nodes=supply_nodes)
        
        self.assert_VRP(supply_nodes, fleet)
        self.draw_VRP(fleet, test_vrp_method)

    # Sample
    # finding a VRP of a subgraph by using nearest neighbor search
    def VRP_NNS(self, time, supply_nodes:set):

        """
        Given vehicle capacity and the required supply points, generate a heuristic VRP route. 
        Note that required set of supply points is a subset of whole supply points.
        
        Args:
            vehicle_capacity (_type_): max capacity of supplies 
            sub_V (_type_): a partial set of class Node in this graph  
        
        Returns:
            fleet: set of vehicles with assigned delivery routes
        """
        
        # Preprocessing
        fleet = set()
        depot = self.graph.get_depot()
        vehicle_capacity = self.get_vehicle_capacity()
        speed = self.get_speed()
        
        # get the sorted list of node to the depot 
        supply_nodes_sorted = list(supply_nodes)
        supply_nodes_sorted.sort(key=lambda x : np.linalg.norm(x.get_pos() - depot.get_pos(), 2))
        
        idx = 0
        while idx != len(supply_nodes_sorted):
            # every iteration add closest node to a vehicle's route until the capacity is maximized
            total_weight = 0
            path = [depot]
            while idx != len(supply_nodes_sorted) and                     (total_weight + supply_nodes_sorted[idx].get_weight() <= vehicle_capacity):
                path.append(supply_nodes_sorted[idx])
                total_weight += supply_nodes_sorted[idx].get_weight()
                idx += 1
            path.append(depot)
            vehicle = Vehicle(speed=speed, vehicle_capacity=vehicle_capacity, 
                              initial_time=time, path=path)
            fleet.add(vehicle)
        return fleet
    
    # finding a VRP of a subgraph by using saving heuristic
    def VRP_Saving(self, time, supply_nodes:set):
        """
        Using the savings heuristic to solve the VRP.

        Args:
            vehicle_capacity (float): The capacity of each vehicle.
            supply_nodes (set): A set of nodes that need to be visited.

        Returns:
            fleet (set): Set of vehicles with assigned delivery routes.
        """    
        # Preprocessing
        depot = self.graph.get_depot()
        vehicle_capacity = self.get_vehicle_capacity()
        speed = self.get_speed()
        unvisited = set(supply_nodes)
        savings_list = []
        routes = []
        
        # Calculate the savings for each pair of supply nodes
        for node_i in supply_nodes:
            for node_j in supply_nodes:
                if node_i != node_j:
                    saving = (np.linalg.norm(node_i.get_pos() - depot.get_pos(), 2) + 
                          np.linalg.norm(node_j.get_pos() - depot.get_pos(), 2) -
                          np.linalg.norm(node_i.get_pos() - node_j.get_pos(), 2))
                    savings_list.append((saving, node_i, node_j))
        
        # Sort savings in decreasing order
        savings_list.sort(key=lambda x: x[0], reverse=True)
        
        for node in supply_nodes:
            routes.append([node])
        
        for saving, node_i, node_j in savings_list:
            route_i = None
            route_j = None
            for route in routes:
                if node_i in route:
                    route_i = route
                if node_j in route:
                    route_j = route
            # Check if nodes are not in the same route and merging won't exceed capacity
            if route_i != route_j and sum(node.get_weight() for node in route_i + route_j) <= vehicle_capacity:
                route_i.extend(route_j)
                routes.remove(route_j)

            # Convert routes to vehicles
            fleet = set()
            for route in routes:
                vehicle_route = [depot] + route + [depot]
                vehicle = Vehicle(speed=speed, vehicle_capacity=vehicle_capacity, 
                                  initial_time=time, path=vehicle_route)
                fleet.add(vehicle)

        return fleet
    
    
    def VRP_NI(self, time, supply_nodes:set):
        depot = self.graph.get_depot()
        vehicle_capacity = self.get_vehicle_capacity()
        speed = self.get_speed()
        unvisited = set(supply_nodes)
        routes = []
        fleet = set()  

        while unvisited:
            route = [depot, depot]  
            total_weight = 0
            while True:
                min_distance = float('inf')
                nearest_node = None
                best_insertion_position = None

                for idx, route_node in enumerate(route[:-1]):
                    for supply_node in unvisited:
                        if total_weight + supply_node.get_weight() > vehicle_capacity:
                            continue
                        distance_to_supply = np.linalg.norm(route_node.get_pos() - supply_node.get_pos(),2)
                        distance_from_supply = np.linalg.norm(supply_node.get_pos() - route[idx+1].get_pos(),2)

                        cost_insertion = distance_to_supply + distance_from_supply
                        if cost_insertion < min_distance:
                            min_distance = cost_insertion
                            nearest_node = supply_node
                            best_insertion_position = idx + 1
                if not nearest_node:
                    break

                route.insert(best_insertion_position, nearest_node)
                total_weight += nearest_node.get_weight()
                unvisited.remove(nearest_node)

            if len(route) > 2: 
                routes.append(route)


        for route in routes:
            vehicle = Vehicle(speed=speed, vehicle_capacity=vehicle_capacity, 
                              initial_time=time, path=route)
            fleet.add(vehicle)
        return fleet


    
    # finding a VRP of a subgraph by using farthest insertion heuristic 
    def VRP_FI(self, time, supply_nodes:set):
        """
        Implement your code here
        """
        depot = self.graph.get_depot()
        vehicle_capacity = self.get_vehicle_capacity()
        speed = self.get_speed()
        unvisited = set(supply_nodes)
        routes = []
        fleet = set()

        while unvisited:
            route = [depot]
            total_weight = 0

            while unvisited:
                max_distance = -1
                farthest_node = None
                insertion_position = None

                for route_node in route:
                    for supply_node in unvisited:
                        distance = np.linalg.norm(np.array(route_node.get_pos()) - np.array(supply_node.get_pos()),2)
                        if distance > max_distance:
                            max_distance = distance
                            farthest_node = supply_node
                            insertion_position = route.index(route_node)

                if total_weight + farthest_node.get_weight() <= vehicle_capacity:
                    route.insert(insertion_position + 1, farthest_node)
                    total_weight += farthest_node.get_weight()
                    unvisited.remove(farthest_node)
                else:
                    break

            route.append(depot)
            routes.append(route)
            route = [depot]


        for route in routes:
            vehicle = Vehicle(speed=speed, vehicle_capacity=vehicle_capacity, 
                              initial_time=time, path=route)
            fleet.add(vehicle)

        return fleet
    
    # finding a VRP of a subgraph by using sweep heuristic 
    def VRP_Sweep(self, time, supply_nodes:set):
        """
        Implement your code here
        """   
        depot = self.graph.get_depot()
        vehicle_capacity = self.get_vehicle_capacity()
        speed = self.get_speed()
        fleet = set()
        
        angles = {}
        for node in supply_nodes:
            dx = node.get_pos()[0] - depot.get_pos()[0]
            dy = node.get_pos()[1] - depot.get_pos()[1]
            angle = np.arctan2(dy, dx)
            angles[node] = angle

        sorted_nodes = sorted(supply_nodes, key=lambda node: angles[node])

        routes = []
        current_route = [depot]
        current_weight = 0

        for node in sorted_nodes:
            if current_weight + node.get_weight() <= vehicle_capacity:
                current_route.append(node)
                current_weight += node.get_weight()
            else:
                current_route.append(depot)  # Return to depot
                routes.append(current_route)
                current_route = [depot, node]  # Start new route
                current_weight = node.get_weight()

        if current_route:
            current_route.append(depot)  # Final return to depot for the last route
            routes.append(current_route)

        for route in routes:
            vehicle = Vehicle(speed=speed, vehicle_capacity=vehicle_capacity, 
                              initial_time=time, path=route)
            fleet.add(vehicle)

        return fleet