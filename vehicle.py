from geometry import *
from intervaltree import Interval, IntervalTree

class Route:
    def __init__(self, speed, initial_time, path):
        """_summary_

        Args:
            path (_type_): list of Nodes
            speed (_type_): speed
            initial_time (_type_): starting time
        """
        self.path = path
        self.speed = speed
        self.schedule = [0]
        for i in range(len(self.path) - 1):
            self.schedule.append(self.schedule[-1] + 
                    np.linalg.norm(self.path[i+1].get_pos() - self.path[i].get_pos(), 2)/self.speed)
        self.schedule = np.array(self.schedule) + initial_time
        self.node_time = {path[time_idx]:self.schedule[time_idx] for time_idx in range(len(self.schedule))}
        self.time_interval_tree = IntervalTree([Interval(self.schedule[i], self.schedule[i+1], 
                                    (self.path[i], self.path[i+1])) for i in range(len(self.schedule)-1)])
     
    def get_schedule(self):
        return self.schedule
    
    def get_total_distance(self):
        return (self.schedule[len(self.schedule)-1] - self.schedule[0]) * self.speed
    
    def get_seg(self, time):
        """
        Get the last delivery and next delivery point at time t

        Args:
            time (_type_): time 

        Returns:
            node1, node2: last delivery point and next delivery point
        """
        min_time = self.schedule[0]
        max_time = self.schedule[-1]
        last_node = self.path[-1]
        if time < min_time:
            return None, None, None
        elif time > max_time:
            return None, None, last_node
        segs = self.time_interval_tree[time]
        seg = list(segs)[0]
        node1, node2 = seg.data
        pos1, pos2 = node1.get_pos(), node2.get_pos()
        vector = (pos2 - pos1)/np.linalg.norm(pos2-pos1, 2) 
        pos_mid = pos1 + (time - self.node_time[node1])*vector
        return node1, node2, pos_mid
        
class Vehicle:
    def __init__(self, speed, vehicle_capacity, initial_time, path):
        """
        initialize vehicle class

        Args:
            vehicle_capacity (_type_): capacity
            initial_time (_type_): time when this vehicle enters the system
            route (_type_, optional): list of Nodes. Defaults to None.
        """
        self.speed = speed
        self.vehicle_capacity = vehicle_capacity
        self.path = path
        self.initial_time =  initial_time
        self.set_route(speed, initial_time, path)
        
    def set_route(self, speed, initial_time, path):
        self.route = Route(speed, initial_time, path)
        
    def get_route(self):
        return self.route 
    
    def get_initial_time(self):
        return self.initial_time
    
    def get_path(self):
        """
        Get the path = [Node1, Node2, ...] 
        """
        return self.path
    
    def get_status(self, time):
        """
        Get spatial status of vehicle at time t

        Args:
            time (_type_): time

        Returns:
            last_node, next_node, curr_position: last delivery point, next_delivery point, and current positions   
        """
        last_node, next_node, curr_position = self.route.get_seg(time)    
        return last_node, next_node, curr_position
    
    def get_end_time(self):
        return self.route.get_schedule()[-1]