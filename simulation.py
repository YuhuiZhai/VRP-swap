import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
from vehicle import *
from geometry import * 
from vrp_generator import *
np.random.seed(1)

class Simulation:
    def __init__(self, area:float, speed, vehicle_capacity:float, time_duration:float, time_resolution:float, customer_arrival_rate, 
                 batch_time, depot:np.ndarray, vrp_method):
        """
        Initialize class
        """
        # space related
        self.R = area
        self.phi = np.sqrt(area)
        self.depot = depot
        
        # supply and demand
        self.speed = speed
        self.vehicle_capacity = vehicle_capacity
        self.demand_rate = customer_arrival_rate
        
        # time related
        self.deltaT = time_resolution
        self.T = time_duration
        self.total_time_steps = int(self.T/self.deltaT)
        self.batch_time = batch_time
        
        # generator related
        self.fleet = set()
        self.customers = set()
        self.VRP_generator = VRP_generator(speed=self.speed, vehicle_capacity=self.vehicle_capacity)
        self.vrp_method = vrp_method
        # recorder related
        self.time_fleet_number = []
        self.time_assigned_distance = []
        return 
    
    def get_area(self):
        return self.R
    
    def get_area_side_length(self):
        return self.phi
    
    def get_depot(self):
        return self.depot
    
    def get_fleet_size(self):
        return self.M
    
    def get_vehicle_capacity(self):
        return self.vehicle_capacity

    def get_time_duration(self):
        return self.T
  
    def get_time_resolution(self):
        return self.deltaT
  
    def get_total_time_steps(self):
        return self.total_time_steps
  
    def get_demand_rate(self):
        return self.demand_rate
     
    def get_batch_time(self):
        return self.batch_time
  
    def get_fleet(self):
        return self.fleet

    def remove_from_fleet(self, vehicles:set):
        self.fleet -= vehicles
   
    def add_to_fleet(self, vehicles:set):
        self.fleet = self.fleet | vehicles
    
    def get_customers(self):
        return self.customers
    
    def remove_from_customers(self, customers:set):
        self.customers -= customers
    
    def add_to_customers(self, customers:set):
        self.customers = self.customers | customers
    
    # generate the number of arrived customers at every time step; the arrival is assumed to follow Poisson distribution
    def generate_customers_arrival(self) -> np.ndarray:
        adjusted_lam = self.get_time_resolution() * self.get_demand_rate()
        customers_arrival = np.random.poisson(lam=adjusted_lam, size=self.get_total_time_steps())
        return customers_arrival
    
    def generate_new_customers(self, customers_number):
        # get current number of customers 
        num = customers_number
        if num == 0:
            return None        
        customers = np.zeros((num, 3))
        customers[:, :2] = self.get_area_side_length() * np.random.random((num, 2))
        customers[:, 2] = np.random.randint(1, 11)
        return customers 
    
    # assign customers to vehciles by VRP heuristic, retunr set of vehicles
    def generate_new_vehicles(self, time, customers, vrp_method:str):
        if type(customers) != np.ndarray:
            return set()
        nodes = np.concatenate((self.get_depot(), customers), axis=0)
        graph = Graph(nodes=nodes)
        self.VRP_generator.set_graph(graph)
        fleet = None
        if vrp_method == "NNS":
            fleet = self.VRP_generator.VRP_NNS(time, graph.get_nodes_without_depot())        
        elif vrp_method == "Saving":
            fleet = self.VRP_generator.VRP_Saving(time, graph.get_nodes_without_depot())     
        elif vrp_method == "NI":
            fleet = self.VRP_generator.VRP_NI(time, graph.get_nodes_without_depot())   
        elif vrp_method == "FI":
            fleet = self.VRP_generator.VRP_FI(time, graph.get_nodes_without_depot())   
        elif vrp_method == "Sweep":
            fleet = self.VRP_generator.VRP_Sweep(time, graph.get_nodes_without_depot())   
        
        assert fleet != None
        return fleet
    
    def update_vehicles_status(self, time):
        removed_vehicles = set()
        for vehicle in self.fleet:
            end_time = vehicle.get_end_time()
            # the vehicle has delivered its last customer
            if time >= end_time:
                removed_vehicles.add(vehicle)
        self.remove_from_fleet(removed_vehicles)
        return 
    
    def total_assigned_distane(self, fleet):
        total_dist = 0
        for vehicle in fleet:
            dist = vehicle.get_route().get_total_distance()
            total_dist += dist
        return total_dist
    
    def go(self):
        # initialize
        deltaT = self.get_time_resolution()
        batch_time = self.get_batch_time()
        batch_count = 0
        customers_arrival = self.generate_customers_arrival()
        
        waiting_customers = None
        # iteration 
        for time_step in tqdm(range(self.get_total_time_steps())):
            time = time_step * deltaT
            
            # generate new customers with positions and demand (np.ndarray)
            customers_number = customers_arrival[time_step]
            new_customers = self.generate_new_customers(customers_number)

            new_vehicles = set()
            # determine whether it is time to do VRP
            if time > batch_count * batch_time:
                new_vehicles = self.generate_new_vehicles(time, waiting_customers, vrp_method=self.vrp_method)
                batch_count += 1
                waiting_customers = None
            else:
                if type(waiting_customers) != np.ndarray and type(new_customers) == np.ndarray:
                    waiting_customers = new_customers
                elif type(waiting_customers) == np.ndarray and type(new_customers) == np.ndarray:
                    waiting_customers = np.concatenate((waiting_customers, new_customers), axis=0)

            self.add_to_fleet(new_vehicles)
            self.update_vehicles_status(time)
            
            self.time_fleet_number.append([time, len(self.fleet)])
            self.time_assigned_distance.append([time, self.total_assigned_distane(new_vehicles)])
        self.time_fleet_number = np.array(self.time_fleet_number)
        self.time_assigned_distance = np.array(self.time_assigned_distance)
        
    def plot(self, parameter="fleet number"):
        recorders = {
            "fleet number":self.time_fleet_number,
            "delivery distance":self.time_assigned_distance
        }
        recorder = recorders[parameter]
        time = recorder[:, 0]
        value = recorder[:, 1]
        plt.plot(time, value, "-")
        plt.title(f"VRP {parameter} with {int(self.batch_time/60)} min matching gap\ndemand rate {int(self.demand_rate*3600)} customer/hr by {self.vrp_method} heuristic")
        plt.xlabel("time (sec)")
        plt.ylabel(f"VRP {parameter}")
        plt.show()
