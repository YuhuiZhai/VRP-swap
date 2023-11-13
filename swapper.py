from vehicle import *
class Swapper:
    def __init__(self):
        return 
        
    # Sample
    def check_swap_1(self, vehicle1:Vehicle, vehicle2:Vehicle, time):
        """
        Check whether two vehicles can be swapped in swap 1 case
        
        Args:
            vehicle1 (_type_): first vehicle
            vehicle2 (_type_): second vehicle
            time (_type_): time 
            
        Returns:
            distance_gain: the distance shorten after swapping
        """
        
        # check distance gain   
        last_node1, next_node1, curr_pos1 = vehicle1.get_status(time)
        last_node2, next_node2, curr_pos2 = vehicle2.get_status(time)
        dist1 = np.linalg.norm(next_node1.get_pos() - curr_pos1, 2)
        dist2 = np.linalg.norm(next_node2.get_pos() - curr_pos2, 2)
        dist3 = np.linalg.norm(next_node2.get_pos() - curr_pos1, 2)
        dist4 = np.linalg.norm(next_node1.get_pos() - curr_pos2, 2)
        dist_gain = dist1 + dist2 - (dist3 + dist4)
        return dist_gain
    
    def check_swap_2(self, vehicle1:Vehicle, vehicle2:Vehicle, time):
        """
        Check whether two vehicles can be swapped in swap 2 case
        
        Args:
            vehicle1 (_type_): first vehicle
            vehicle2 (_type_): second vehicle
            
        Returns:
            gain: gain after swapping
        """
        
        
        """
        Check whether first segments of two routes intersect with each other
        
        Implement your code here
        """
        is_intersect = ...
        
        if not is_intersect:
            return -np.inf
        
        
        """
        Check the distance gain after swapping
        
        Implement your code here
        """
        distance_gain = ... 
        return distance_gain
    