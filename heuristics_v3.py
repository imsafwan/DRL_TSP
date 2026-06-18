

'''
 v2: include graph network 
 
- Heuriscs approach for performing persistent surveillance

- It is for prioritized mission points visit using proritized penalty function 

- Static recharging

- UGV route is found by solving MSC and TSP 

-  It is tried on a single agent

'''


import multiprocessing
import csv
import ast
import numpy as np
from ortools.sat.python import cp_model
import matplotlib.pyplot as plt
import pandas as pd
import time
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import time
import random
import networkx as nx
import matplotlib.pyplot as plt
from colorama import Fore
from termcolor import cprint
import os

# Folder and file setup
folder_name = 'Tours_output'  # Replace with your folder name



def remove_consecutive_duplicates(a_list):
    if not a_list:  # Check if the list is empty
        return []

    # Initialize a new list with the first element of the original list
    new_list = [a_list[0]]

    # Iterate through the list, compare each element with the previous one
    for i in range(1, len(a_list)):
        if a_list[i] != a_list[i - 1]:
            new_list.append(a_list[i])

    return new_list

def find_key_for_value(my_dict, value_to_find):
    #print(my_dict, value_to_find)
    for key, value in my_dict.items():
        
        if value == value_to_find:
            kkkk
            return key
    return None


def printc(text, color, bold=False):
    colors = {
        'r': Fore.RED,
        'g': Fore.GREEN,
        'y': Fore.YELLOW,
        'b': Fore.BLUE,
        'm': Fore.MAGENTA,
        'c': Fore.CYAN,
        'w': Fore.WHITE,
    }
    bold_code = '\033[1m' if bold else ''
    color_code = colors.get(color, '')
    reset_code = '\033[0m'
    print(f'{bold_code}{color_code}{text}{reset_code}')  



       
class Major_Replan():
    
    def __init__(self, uav_data_points,ugv_data_points,Targets,Locations,starting, metaheuristics):
        
        self.uav_data_points = uav_data_points
        self.ugv_data_points = ugv_data_points        
        self.Targets = Targets
        self.last_visit = {i : 0 for i in self.Targets }
        self.Age_period = {i : 0 for i in self.Targets }
        self.Mission_elapsed_time = 0
        self.Locations = Locations
        self.starting = starting
        self.Fuel_limit =int(287700)
        self.UAV_Mission_time = 0
        self.UAV_node_visit = [starting]
        self.UAV_node_visit_time = [0]
        self.UGV_node_visit = [starting]
        self.UGV_node_visit_time = [0]
        self.Recharging_number = 0
        self.Unvisited_Mission_points = Targets.copy()
        self.solver_time = 60
        self.metaheuristics = metaheuristics
        self.starting_depot = starting 
        self.recharging = 0
        self.ugv_speed = 15
        self.uav_speed = 33
        #self.df = pd.read_csv(os.path.join(folder_name, 'output_start_loc.csv'))
        self.UAV_starting_point = [starting]
        self.UGV_starting_point = [starting]
        self.UGV_route_instances = []
        self.UAV_route_instances = [[],[]]
        self.UAV_fuel = [0,0]
        self.UGV_stop_location = []
        self.UGV_route_plan = []
        self.UAV_route_plan = {}
        self.recharge_map_dict = {ugv_data_points[i]: i for i in range(len(ugv_data_points))}
        
        self.map_dict = {ugv_data_points[i]: len(ugv_data_points) + i for i in range(len(ugv_data_points))}
        self.map_dict.update({uav_data_points[i]: len(ugv_data_points) + len(ugv_data_points) + i for i in range(len(uav_data_points))})
        self.action_heu = []
        
        

        
    
    def cumulative_distance_determination (self):
         '''
        

        Returns
        -------
        G : graph network of the ugv road network

         '''
         df_main = pd.read_csv('ugv_road.csv')
         ugv_data_points = [(df_main['UGV_X'][i], df_main['UGV_Y'][i]) for i in range(len(df_main['UGV_X']))]
         
         G = nx.Graph()
         
         for coord in ugv_data_points:
                 G.add_node(coord)
         # Add edges with weights
         for i in range(0,6):
                  weight = int(np.sqrt((ugv_data_points[i+1][0] - ugv_data_points[i][0])**2 + (ugv_data_points[i+1][1] - ugv_data_points[i][1])**2)*5280)
                  G.add_edge(ugv_data_points[i], ugv_data_points[i+1], weight=weight)
         for i in range(7,18):
                  weight = int(np.sqrt((ugv_data_points[i+1][0] - ugv_data_points[i][0])**2 + (ugv_data_points[i+1][1] - ugv_data_points[i][1])**2)*5280)
                  G.add_edge(ugv_data_points[i], ugv_data_points[i+1], weight=weight)
         for i in range(19,29):
                  weight = int(np.sqrt((ugv_data_points[i+1][0] - ugv_data_points[i][0])**2 + (ugv_data_points[i+1][1] - ugv_data_points[i][1])**2)*5280)
                  G.add_edge(ugv_data_points[i], ugv_data_points[i+1], weight=weight)
         
         return G                 
            
                     

    def cu_dis1 (self,node1,node2):
          G = self.cumulative_distance_determination()
          path = nx.shortest_path(G, source=node1, target=node2, weight='weight')
          distance = int(sum(G[path[i]][path[i + 1]]['weight'] for i in range(len(path) - 1)))
          return distance               

    def path_(self,Stop1,Stop2):
        
         '''
        

        Parameters
        ----------
        Stop1 : Tuple
            Coordinate of mission point.
        Stop2 : Tuple
            Coordinate of mission point.

        Returns
        -------
        path : list of tuples
            path between two coordinates along the ugv road
        Time : list of tuples
            time taken to travel along the path.
        Recharge_time_nearest : list of tuples
           needed for recharge while travelling.
        Recharge_loc_nearest : list of tuples
           needed for recharge while travelling.

         '''
        
         G = self.cumulative_distance_determination()
         if Stop1 == Stop2:
             path = [Stop1,Stop1]
         else:
             path = nx.shortest_path(G, source=Stop1, target = Stop2, weight='weight')
         
         path = [i for i in path if i  in self.ugv_data_points]
         
         Time = []
         Recharge_time_nearest = []
         Recharge_loc_nearest = []
         t = 0
         for i in range(len(path)-1):
             t = t+ self.cu_dis1(path[i],path[i+1])/self.ugv_speed
             Time.append((int(t),600000000))
             Recharge_time_nearest.append(int(t))
             Recharge_loc_nearest.append(path[i+1])
          
         return path,Time,Recharge_time_nearest,Recharge_loc_nearest                   
                          
                      
    def nearest_loc(self,locs,time,recharge_time):
        '''
            The nearest location from where UAV can take off after recharging
            Locs = available locations
            time = time corresponds to the locations
            recharge_time = recharging time 
            
        '''
        try:
          closest_item = min((t for t in time if t > recharge_time), key=lambda x: abs(x - recharge_time))
          Flag = True
        except:
          closest_item = time[-1]
          Flag = False
        print('-----------------------')
        print(locs)
        print(time)
        print(recharge_time)
        print(Flag)
        print(closest_item)
        print('-----------------------')
        next_start_loc = locs[time.index(closest_item)]
        return next_start_loc,Flag

   
        

    def CP_set_cover(self,Targets,Locations,starting): 
        
        '''
        modified minimum set cover algorithm
        Input : Targets = target locations, Locations = path locations , starting = starting point
        Output : Minimum number of recharging locations (with starting location ) with minimum distance traversal along the path
        
        '''
         
        Targets = Targets.copy()
        reduced_Fuel_limit = self.Fuel_limit*1.0
        A = Locations.copy() #Targets.copy()

        def hit(point,targets,Fuel_limit):
            hitted_targets = []
            for j in range(len(targets)):
                a = np.array(point)
                b = np.array(targets[j])
                d = int(np.sqrt(np.sum(np.square(a-b)))*5280)
                
                if (198*d/self.uav_speed) <=(reduced_Fuel_limit/2):
                    hitted_targets.append(j)            
            return hitted_targets
        
        
        starting = starting
        hit_start= hit(starting,Targets,self.Fuel_limit)
        remv=[]
        for i in hit_start:
            remv.append(Targets[i])
        for i in remv:
            Targets.remove(i)
        
        A.remove(starting)
        
        
        
                
        ############## minimum hitting set #################

        model = cp_model.CpModel()
        hitted_dict = {}
        point_vars= {}
        target_set={}
        hit_set={}
        obj_var={}
        objective_set={}
        var = []
        
        
        
        for i in range(len(Targets)):  
           target_set.setdefault('Target {}'.format(i), set())  
           hit_set.setdefault('Target {}'.format(i), set())                          
        for i in range(len(A)):  
           p_var= point_vars.setdefault('Point {}'.format(i), model.NewBoolVar('Point '+str(i)))
           
           hitted_dict.setdefault('{}'.format(i), set(hit(A[i],Targets,self.Fuel_limit)))
           obj_var.setdefault('Point {}'.format(i),(self.cu_dis1(A[i],starting)))
        
        
        for a, b in point_vars.items():
               var.append(b)
               
        for a, b in obj_var.items():

           objective_set.setdefault(a,(b*point_vars[a]))
               
        
        for a, b in hitted_dict.items():
               for k in b:
                   hit_set['Target {}'.format(k)].add(a)


        for a, b in hit_set.items():
               for k in b:
                   target_set['{}'.format(a)].add(list(point_vars.values())[int(k)])
                    
           
        for a,b in target_set.items():
            model.Add(sum(b) >= 1)  
        model.Minimize(sum(objective_set.values()))
        #model.Minimize(sum(point_vars.values()))  
        solver = cp_model.CpSolver()
        status = solver.Solve(model)
        
        if status == cp_model.OPTIMAL:
             selected_points = [p for p in point_vars
                              if solver.Value(point_vars[p])]
             print('No of Rendzvous points: {}\n' .format((len(selected_points))+1))
        else:
             print('Unable to find an optimal solution')
             
        Locs=[]       
        for i in selected_points:
            a,b = i.split(" ")
            Locs.append(A[int(b)])
        
        
        Locs =   [starting] + [starting] + Locs #
        return Locs


                      
                              
    def Allocation_function(self,Targets,Locations,starting):
        '''
        

        Parameters
        ----------
        Targets : List
            Target mission points.
        Locations : list
            path locations.
        starting : tuple , single mission point
           starting point.

        Returns
        -------
        Allocation : dict
            allocation dictionary of stop locations & their covered mission points.

        '''
        Allocation = {}
        Targets = Targets
        Locations = Locations
        starting = starting
        
        
        Locations = Locations
        Refuel_stop_Locs = self.CP_set_cover(Targets, Locations, starting)
        
        Targets = Targets
        
        
        
        # UGV path determination #
        UGV_path = []
        
        if len(Refuel_stop_Locs) == 1: # if only one recharging location
            Refuel_stop_Locs = [Refuel_stop_Locs[0],Refuel_stop_Locs[0]]
        
        i = 1
        while i < len(Refuel_stop_Locs): # creating the path
              UGV_path_i,UGV_time,c,d = self.path_(Refuel_stop_Locs[i-1],Refuel_stop_Locs[i])
              UGV_path = UGV_path + UGV_path_i
              i = i+1
            
            
        
        Locations_copy =  [x for x in Targets if x not in UGV_path]  # Target for UAV , not in UGV path
        
        
        Allc = {}

        for i in Refuel_stop_Locs:
              Allc.setdefault((i),[])

        for i in Locations_copy :
              Allocated = []
              for j in Refuel_stop_Locs:
                  a = np.array(i)
                  b = np.array(j)
                  d = int(np.sqrt(np.sum(np.square(a-b)))*5280)
                  if d*6 <= self.Fuel_limit/2:
                      Allocated.append(j)    # a list that shows uav points and its covering rf stops
              
              Allocated.sort(key=lambda x:int(np.sqrt(np.sum(np.square(np.array(x)-np.array(i))))*5280))
              
              
              Allc[(Allocated[0])].append(i) # add that uav points to the nearest rf stop
        
        
        '''for j in range(len(Refuel_stop_Locs)):
            
            if j ==0:
                 A = Allc[Refuel_stop_Locs[j]]
                 AB = []
                 for item in A :
                    a = np.array(item)
                    b = np.array(Refuel_stop_Locs[j+1])
                    d = int(np.sqrt(np.sum(np.square(a-b)))*5280)
                    if d*6 <= self.Fuel_limit/2:
                        AB.append(item)
                 
                 Covered_locs = [ite for ite in A if ite not in AB]
                 Allocation[starting,Refuel_stop_Locs[j]] = Covered_locs
                 


            elif j == len(Refuel_stop_Locs)-1 :
                    C = Allc[Refuel_stop_Locs[j]]
                    Covered_locs =  C + BC
                    Allocation[Refuel_stop_Locs[j-1],Refuel_stop_Locs[j]] = Covered_locs
                    

            else  : 
                 B = Allc[Refuel_stop_Locs[j]]
                 
                 BC = []
                 for item in B :
                    a = np.array(item)
                    b = np.array(Refuel_stop_Locs[j+1])
                    d = int(np.sqrt(np.sum(np.square(a-b)))*5280)
                    if d*6 <= self.Fuel_limit/2:
                        BC.append(item)
                 
                 jll = [ite for ite in B if ite not in BC.copy()]
                 
                 Covered_locs = AB + [ite for ite in B if ite not in BC]
                 
                 AB = BC
                 Allocation[Refuel_stop_Locs[j-1],Refuel_stop_Locs[j]] = Covered_locs
                 
                 
                   

        cprint('\n Allocation of Mission points obtained = \n {} \n \n'.format( Allocation ), 'cyan', attrs = ['bold'] )'''
        
        Allocation = {}
        for j in range(len(Refuel_stop_Locs)-1):
            
            Covered_locs = []
            
            if j == 0 : 
                    Allocation[starting, Refuel_stop_Locs[j+1]] = Allc[starting] + Allc[Refuel_stop_Locs[j+1]]
            else : 
                    Allocation[Refuel_stop_Locs[j], Refuel_stop_Locs[j+1]] = Allc[Refuel_stop_Locs[j+1]]
        

        cprint('\n Allocation of Mission points obtained = \n {} \n \n'.format( Allocation ), 'cyan', attrs = ['bold'] ) 
        
        
        '''
        # Allocation #
        for j in range(len(Refuel_stop_Locs)-1):
            
            Covered_locs = []
            
            if j == 0 : 

                        for i in range(len(Locations_copy)):
                             a = Locations_copy[i]
                             b = Refuel_stop_Locs[j]
                             a = np.array(a)
                             b = np.array(b)
                             d = int(np.sqrt(np.sum(np.square(a-b)))*5280)
                             
                             if d*6 <= self.Fuel_limit/2:
                                 Covered_locs.append(Locations_copy[i])
                             
                             

                        for item in Covered_locs: 
                            if item in Locations_copy:
                                    Locations_copy.remove(item)

                        
                        
                        for i in range(len(Locations_copy)):
                             a = Locations_copy[i]
                             b = Refuel_stop_Locs[j+1]
                             a = np.array(a)
                             b = np.array(b)
                             d = int(np.sqrt(np.sum(np.square(a-b)))*5280)
                             if d*6 <= self.Fuel_limit/2:
                                 Covered_locs.append(Locations_copy[i])

                        
                        
                        
                        for item in Covered_locs: 
                            if item in Locations_copy:
                                    Locations_copy.remove(item)

                        Allocation[starting,Refuel_stop_Locs[j+1]] = Covered_locs

                        
            else:

                        for i in range(len(Locations_copy)):
                             a = Locations_copy[i]
                             b = Refuel_stop_Locs[j+1]
                             a = np.array(a)
                             b = np.array(b)
                             d = int(np.sqrt(np.sum(np.square(a-b)))*5280)
                             if d*6 <= self.Fuel_limit/2:
                                 Covered_locs.append(Locations_copy[i])
                             

                        for item in Covered_locs: 
                            if item in Locations_copy:
                                    Locations_copy.remove(item)

                        Allocation[Refuel_stop_Locs[j],Refuel_stop_Locs[j+1]] = Covered_locs     
            
        cprint('\n Allocation of Mission points obtained = \n {} \n \n'.format( Allocation ), 'cyan', attrs = ['bold'] ) 
        kkk '''

        return Allocation
    
    
    
    
    
    
    
    


    
    
        
    def UAV_planner(self, starting_time, starting_location, Mission_points, UGV_end_stop, UGV_end_stop_time):
        
    
              '''
              

              Parameters
              ----------
              MP : list of tuples
                  mission points.
              UGV_points : list of tuples
                  ugv path.
              UGV_time : list
                  ugv time.

              Returns
              -------
              TYPE
                  route.

              '''
              
              def create_data_model():
                  
                  """Stores the data for the problem."""
                  
                  data = {}
                  distance = []
                  N_d_r =  6   #No_of_duplicate_nodes_for_recharging
                  
                  # as a form of coordinates #
                  A_s = [starting_location]
                  A_v = Mission_points
                  A_r =  [UGV_end_stop]*N_d_r
                  A_e = [UGV_end_stop]
                  
                  # as a form of nodes #
                  starting_node = [0]
                  mission_point_node = list(range(len(starting_node),len(A_s +A_v)))
                  refuel_stop_nodes =  list(range(len(A_s+A_v), len(A_s+A_v+A_r)))
                  end_node = [len(A_s+ A_v+ A_r)]
                  
                  data['starting node'] = starting_node
                  data['mission_point_node'] = mission_point_node
                  data['refuel_stop_nodes'] = refuel_stop_nodes
                  data['end_node'] = end_node
                  
                  
                  

                  
                          
                  #creation of distance matrix#
                  A1 = A_s + A_v + A_r + A_e # all coordinates
                  
                  for i in range(len(A1)):
                     distance.append([])
                  for i in range(len(A1)):
                      for j in range(len(A1)):
                          a = A1[i]
                          b = A1[j]
                          a = np.array(a)
                          b = np.array(b)
                          #if i in refuel_stop_nodes and j in refuel_stop_nodes :
                              #d = 10000
                          #else: 
                          d = int(np.sqrt(np.sum(np.square(a-b)))*5280)
                          distance[i].append(d)
                  
                  
                  
                  

                  data['distance_matrix'] = distance
                  data['num_vehicles'] = 1
                  data['starts'] = starting_node 
                  data['locations']= A1
                  data['ends'] = [len(A1)-1]*data['num_vehicles']
                  
                  data['refuel_stations'] = list(range(len(A_s+A_v), len(A_s+A_v+A_r)))
                  data['visit_stations'] = list(range(1,len(A_s +A_v)))
                  tws = [(0,60000000)]*data['num_vehicles']
                  tw1 = [(0,600000000)]*len(A_v)
                  tw2 =  [(UGV_end_stop_time, 60000000)]*N_d_r
                  tw_e = [(UGV_end_stop_time, 60000000)]
                  

                  data['time_windows'] = tws+tw1+tw2+tw_e
                  data['fuel_limit'] = self.Fuel_limit
                  data['vehicle_speed']= self.uav_speed
                  data['visit_end_nodes'] = len(A_s+A_v)-1 
                  data['station_end_nodes'] = len(A1)-2  
                  

                  
                  print('Time window', data['time_windows'])
                  print('Starts',data['starts'] , 'Ends', data['ends'])
                  

                  assert len(data['distance_matrix']) == len(data['time_windows'])
                  assert len(data['starts']) == len(data['ends'])
                  assert data['num_vehicles'] == len(data['starts'])
                  
                  print('Refuel stops ', A_r)
                  data['Refuel_locs'] = A_r
                  data['Refuel_time']= tw2
                  data['Mission_locs'] = A_v
                  
                  
                  
                  
                  
                  
                  return data


              def print_solution(data, manager, routing, solution): 
                  
                      data = create_data_model()
                      locations = data['locations']
                      A_r = data['Refuel_locs']
                      tw2=data['Refuel_time']
                      A_v=  data['Mission_locs'] 
                      
                      #print("Objective: {}".format(solution.ObjectiveValue()))  
                      obj =  solution.ObjectiveValue()
                      total_fuel = 0
                      total_time = 0
                      total_distance = 0
                      
                      distance_dimension = routing.GetDimensionOrDie("Distance")
                      fuel_dimension = routing.GetDimensionOrDie("Fuel")
                      time_dimension = routing.GetDimensionOrDie("Time")
                      # Display dropped nodes.
                      dropped_nodes = 'Dropped nodes:'
                      dropped_locs = []  
                      for node in range(routing.Size()):
                         if routing.IsStart(node) or routing.IsEnd(node):
                             continue
                         if solution.Value(routing.ActiveVar(node)) == 0:
                             dropped_nodes += ' {} node {}'.format(manager.IndexToNode(node),locations[manager.IndexToNode(node)])
                             #if locations[manager.IndexToNode(node)] in A_v:
                             dropped_locs.append(locations[manager.IndexToNode(node)])
                            
                      print('Dropped in SP ={}'.format(dropped_locs) )      
                      
                      
                      
                      #UGV_route_plan = {}
                      UGV_recharge_plan = []
                      
                      
                      for vehicle_id in range(data['num_vehicles']):
                          
                          print('vehicle id >>>>>>>>>>>>>>> ', vehicle_id)
                          index = routing.Start(vehicle_id)
                          plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
                          route_distance = 0
                          d = 0
                          
                          ugv_starting_location = starting_location
                         
                          Travelled_node = []
                          Travelled_time_start = []
                          Travelled_time_end = []
                          waiting_time_UAV = []
                          Recharging_time = []            
                          Fuel_UAV = []
                          
                          
                          initial = 0
                          while not routing.IsEnd(index):
                              dist_var = distance_dimension.CumulVar(index)
                              fuel_var = fuel_dimension.CumulVar(index)
                              time_var = time_dimension.CumulVar(index)
                              slack_var = time_dimension.SlackVar(index)
                        
                              Travelled_node.append(locations[manager.IndexToNode(index)])
                              
                                 
                                       
                              Travelled_time_start.append(solution.Min(time_var))
                              Travelled_time_end.append(solution.Max(time_var))
                              waiting_time_UAV.append(solution.Max(slack_var))
                              Fuel_UAV.append(solution.Value(fuel_var))
                              

                                  
                              plan_output += "{0} Node {1} Fuel({2}) Time({3},{4}) TimeSlack ({5}) Distance({6}) -> ".format(
                                  manager.IndexToNode(index),locations[manager.IndexToNode(index)],
                                  solution.Value(fuel_var),
                                  solution.Min(time_var), solution.Max(time_var),solution.Value(slack_var), solution.Value(dist_var))

                              previous_index = index
                              index = solution.Value(routing.NextVar(index))  
                              
                               
                              if locations[manager.IndexToNode(index)] == UGV_end_stop:
                                  
                                  if locations[manager.IndexToNode(index)] != locations[manager.IndexToNode(previous_index)] :
                                      
                                  
                                     self.action_heu.append((self.recharge_map_dict[locations[manager.IndexToNode(index)]], 1))
                              
                                     UGV_travel_path, UGV_travel_time, _, _ = self.path_(ugv_starting_location, UGV_end_stop)
                              
                                     for i in UGV_travel_path :
                                          self.action_heu.append((self.map_dict[i], 0))
                              
                                     self.action_heu.append((self.recharge_map_dict[i], 0))
                                     ugv_starting_location = UGV_travel_path[-1]
                              
                              else:
                                     self.action_heu.append((self.map_dict[locations[manager.IndexToNode(index)]], 1))

                                     
                          
                          dist_var = distance_dimension.CumulVar(index)
                          fuel_var = fuel_dimension.CumulVar(index)
                          time_var = time_dimension.CumulVar(index)
                          
                          
                      
                          Travelled_node.append(locations[manager.IndexToNode(index)])
                          Travelled_time_start.append(solution.Min(time_var))
                          Travelled_time_end.append(solution.Max(time_var))
                          waiting_time_UAV.append(solution.Max(slack_var))
                          Fuel_UAV.append(solution.Value(fuel_var))
     
                          
                              
                          plan_output += "{0} Node {1} Fuel({2}) Time({3},{4}) TimeSlack ({5}) Distance({6}) -> ".format(
                              manager.IndexToNode(index),locations[manager.IndexToNode(index)],
                              solution.Value(fuel_var),
                              solution.Min(time_var), solution.Max(time_var),solution.Value(slack_var), solution.Value(dist_var))
                          plan_output += "Distance of the route: {} ft\n".format(solution.Value(dist_var))
                          plan_output += "Remaining Fuel of the route: {}\n".format(solution.Value(fuel_var))
                          plan_output += "Total Time of the route: {} seconds\n".format(solution.Value(time_var))
                          print(plan_output)
                          total_distance += solution.Value(dist_var)
                          total_fuel += solution.Value(fuel_var)
                          total_time += solution.Value(time_var)
                      
                          
                          
                              
                          
                          
                          return dropped_locs
                      
                      
            
                      #self.UGV_route_plan.append({'UGV Time':last_time+Travelled_time_by_UGV[i][0], 'Coordinate' : Travelled_node_by_UGV[i+1]})
                                  
                      #self.UGV_route_plan.append({'UGV Time':last_value['Time'], 'Coordinate' : last_value['Coordinates']})
                      
                      
                      
                      
                      
                      
                      
                      



              def main():
                  
                  """Solve the CVRP problem."""
                  
                  # Instantiate the data problem.

                  data = create_data_model() 
                  refuel_stations = data['refuel_stations']
                  fuel_limit = data['fuel_limit']
                  starting_node = data['starting node'] 
                  mission_point_node = data['mission_point_node']
                  refuel_stop_nodes = data['refuel_stop_nodes']
                  
                  
                  
                  end_node = data['end_node']                 
                  fuel_capacity = data['fuel_limit']
                  
                  print('Number of Mission points : \n {}'.format(len(data['Mission_locs'])))
                  print('Refuel stops: \n {}'.format(data['Refuel_locs']))
                
                  # Create the routing index manager.
                  manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                                         data['num_vehicles'],data['starts'],
                                                         data['ends'])

                  # Create Routing Model.
                  routing = pywrapcp.RoutingModel(manager)
                  
                  
                  
                  #---------------------------------------------------------------------------------------------#

                  
                  def distance_callback(from_index, to_index):
                      """Returns the distance between the two nodes."""
                      # Convert from routing variable Index to distance matrix NodeIndex.
                      from_node = manager.IndexToNode(from_index)
                      to_node = manager.IndexToNode(to_index)
                      return data['distance_matrix'][from_node][to_node]

                  transit_callback_index = routing.RegisterTransitCallback(distance_callback)

                  # Add Distance constraint.
                  dimension_name = 'Distance'

                  routing.AddDimension(
                      transit_callback_index,
                      0,  # no slack
                      300000000000,  # vehicle maximum travel distance
                      True,  # start cumul to zero
                      dimension_name)

                  distance_dimension = routing.GetDimensionOrDie(dimension_name)
                  
                  
                  
                  
                  
                  
                  #--------------------------------------------------------------------------------------------------#

                  
                  def time_callback(from_index, to_index):
                      from_node = manager.IndexToNode(from_index)
                      to_node = manager.IndexToNode(to_index)
                      
                      t = int(data["distance_matrix"][from_node][to_node] / data["vehicle_speed"])
                          
                      return t


                  time_callback_index = routing.RegisterTransitCallback(time_callback)
                  routing.SetArcCostEvaluatorOfAllVehicles(time_callback_index)

                  
                  routing.AddDimension(
                      time_callback_index,
                      1000,  # max slack/wait time
                      10000000000000,  # vehicle max time
                      False,  # don't force cumul to zero
                      'Time')
                  
                  time_dimension = routing.GetDimensionOrDie('Time')
                  #time_dimension.SetGlobalSpanCostCoefficient(100)
                  
                  #for location_idx, time_window in enumerate(data["time_windows"]):
                      #if location_idx in data["starts"] :
                          #index = manager.NodeToIndex(location_idx)
                          #time_dimension.SlackVar(index) == 0
                  
                  for location_idx, time_window in enumerate(data["time_windows"]):
                      if location_idx in data["starts"] or location_idx in data["ends"]:
                          continue
                      index = manager.NodeToIndex(location_idx)
                      routing.AddToAssignment(time_dimension.SlackVar(index))
                      #time_dimension.SlackVar(index).SetValue(0)
                      time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
                      
                      routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(index))


                  for vehicle_id in range(data["num_vehicles"]):
                      index = routing.Start(vehicle_id)
                      #time_dimension.SlackVar(index).SetValue(0)
                      time_dimension.CumulVar(index).SetRange(data["time_windows"][0][0],
                                                              data["time_windows"][0][1])                      
                      routing.AddToAssignment(time_dimension.SlackVar(index))

                  for vehicle_id in range(data["num_vehicles"]):
                      index = routing.End(vehicle_id)
                      #routing.AddToAssignment(time_dimension.SlackVar(index))
                      #time_dimension.SlackVar(index).SetValue(0)
                      time_dimension.CumulVar(index).SetRange(data["time_windows"][-1][0],
                                                              data["time_windows"][-1][1])
                  
                    
 
                  
                  
                  for i in range(manager.GetNumberOfVehicles()): 
                      routing.AddVariableMinimizedByFinalizer(
                          time_dimension.CumulVar(routing.Start(i)))
                      routing.AddVariableMinimizedByFinalizer(
                          time_dimension.CumulVar(routing.End(i)))
                      
                      
                  #--------------------------------------------- fuel constraints -----------------------------------------------------#
                 
                  
                  def fuel_callback(from_index,to_index):
                      from_node = manager.IndexToNode(from_index)
                      to_node = manager.IndexToNode(to_index)
                      
                      
                          
                      if to_node in refuel_stations :
                          refuel = - fuel_limit
                      else:
                          refuel = 0
                      return ( refuel+ 198*int(data["distance_matrix"][from_node][to_node] / data['vehicle_speed']))
                  

                  fuel_callback_index = routing.RegisterTransitCallback(fuel_callback)
                  routing.AddDimension(
                      fuel_callback_index,
                      data['fuel_limit'],
                      data['fuel_limit'],
                      False,
                      'Fuel')

                  fuel_dimension = routing.GetDimensionOrDie('Fuel')
                  
                  solver = routing.solver()
                  
                   
                  
                  
                  '''--- accounts for recharging time -------'''
                  
                  for node1 in refuel_stop_nodes : 
                      index1 = manager.NodeToIndex(node1) 
                      cond1 =  (routing.ActiveVar(index1) == 1)
                      t_j = time_dimension.SlackVar(index1)
                      

                      solver.Add( cond1*t_j ==  900 )
                      
                  
                  '''for vehicle_id in range(data["num_vehicles"]):
                      for location_idx0, time_window0 in enumerate(data["time_windows"]):
                            if location_idx0 in data["starts"] or location_idx0 in data["ends"]:
                                continue
                            index0 = manager.NodeToIndex(location_idx0)
                            cond0 =  (routing.NextVar(index0) == routing.End(vehicle_id))
                            solver.Add( cond0*time_dimension.SlackVar(index0) ==  cond0*900 )'''
                  
                    
                  
                  '''for vehicle_id in range(data["num_vehicles"]):
                      for location_idx0, time_window0 in enumerate(data["time_windows"]):
                            if location_idx0 in data["starts"] or location_idx0 in data["ends"]:
                                continue
                            index0 = manager.NodeToIndex(location_idx0)
                            
                            for node in refuel_stop_nodes : 
                                index1 = manager.NodeToIndex(node) 
                                cond0 =  (routing.NextVar(index0) == index1)
                                
                                f_i =  fuel_dimension.CumulVar(index0)
                                f_ij = int((data["distance_matrix"][location_idx0][node] / data["vehicle_speed"])*198)
                                f_j = f_i + f_ij
                                t_j = cond0*time_dimension.CumulVar(index1)
                                t_Re = f_j
                                
                                solver.Add( cond0*311*t_j ==  cond0*f_j )           
                                
                                #print( node, int((data["distance_matrix"][location_idx0][node] / data["vehicle_speed"])*198)/311 )
                            
                            for node in data["ends"] : 
                                cond0 =  (routing.NextVar(index0) == routing.End(vehicle_id))
                                solver.Add( 311*(cond0*time_dimension.CumulVar(index1) )            
                                == cond0*( ( time_dimension.CumulVar(index0)*311 +  (( fuel_dimension.CumulVar(index0) +  int((data["distance_matrix"][location_idx0][node] / data["vehicle_speed"])*198) ))  ))     )
                            '''                   
                            
                  
                  
                  for node in refuel_stop_nodes: 
                      idx = manager.NodeToIndex(node) 
                      #fuel_dimension.SlackVar(idx).SetValue(0)
                      fuel_dimension.CumulVar(idx).SetValue(0)
                      
                  for node in mission_point_node:
                      idx = manager.NodeToIndex(node) 
                      fuel_dimension.SlackVar(idx).SetValue(0)
                  
                  ''' 
                  for vehicle_id in range(data["num_vehicles"]):
                      index = routing.Start(vehicle_id)     
                      solver.Add(fuel_dimension.CumulVar(index) == ((time_dimension.CumulVar(index))*198))'''   
                  
                  
                  # ------------------------------------ penalty ------------------------------------------------------- #
                  
                  for node in refuel_stop_nodes:
                      
                      routing.AddDisjunction([manager.NodeToIndex(node)], 0)
                      
                  penalty = 1000000000
                  for i in range(len(mission_point_node)):
                      
                      node =  mission_point_node[i]
                      routing.AddDisjunction([manager.NodeToIndex(node)], penalty)                 
                  
                 
                  

                  #------------------------------------ solver -----------------------------------------------------------#

                  search_parameters = pywrapcp.DefaultRoutingSearchParameters()
                  search_parameters.first_solution_strategy = (
                      routing_enums_pb2.FirstSolutionStrategy.PATH_MOST_CONSTRAINED_ARC)
                  
                  
                  if self.metaheuristics == "TABU_SEARCH":
                       meta_heuristics_enum = routing_enums_pb2.LocalSearchMetaheuristic.TABU_SEARCH
                  elif self.metaheuristics == "SIMULATED_ANNEALING":
                       meta_heuristics_enum = routing_enums_pb2.LocalSearchMetaheuristic.SIMULATED_ANNEALING
                  elif self.metaheuristics == "GUIDED_LOCAL_SEARCH":
                       meta_heuristics_enum = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
                  search_parameters.local_search_metaheuristic = (meta_heuristics_enum)
                  
                  
                  search_parameters.time_limit.FromSeconds(self.solver_time)


                  # Solve the problem.
                  solution = routing.SolveWithParameters(search_parameters)

                  

                  # Print solution on console.
                  if solution:
                      dropped_locs = print_solution(data, manager, routing, solution)
                      
                  else:
                       print('no solution')   
                  
                    
                  return dropped_locs
              
                
              #if __name__ == '__main__':
              dropped_locs = main()
             
               
              return dropped_locs
            
      
    def animation(self,period_number):
      
        from scipy.interpolate import interp1d
        import numpy as np
        import pandas as pd
        
        #print('UAV Mission time : {} min'.format(self.UAV_Mission_time/60))
        #print('UGV Mission time : {} min'.format(self.UGV_Mission_time/60))
        print(self.UAV_node_visit_time[-1])
        print(self.UGV_node_visit_time[-1])
        self.UAV_node_visit_time = self.UAV_node_visit_time  + [self.UGV_node_visit_time[-1]]
        self.UAV_node_visit = self.UAV_node_visit + [self.UGV_node_visit[-1]]
        
        UAV_X = []
        UAV_Y = []
        for i in self.UAV_node_visit:
            UAV_X.append(i[0])
            UAV_Y.append(i[1])
         
        UAV_timesteps =     np.array(self.UAV_node_visit_time)/1
        UGV_timesteps =     np.array(self.UGV_node_visit_time)/1
        UAV_UGV_timesteps = np.sort(np.unique(np.concatenate((UAV_timesteps, UGV_timesteps))))
        
        # Example list with values at different timesteps
        timesteps = self.UAV_node_visit_time
        timesteps = [x / 1 for x in timesteps]
        
        

        # create interpolation function
        interp_func = interp1d(timesteps, UAV_X)
        new_timesteps = np.arange(0, UAV_UGV_timesteps[-1],1)
        new_timesteps = np.sort(np.unique(np.concatenate((UAV_UGV_timesteps, new_timesteps))))
        
        UAV_X = interp_func(new_timesteps)
        

        # create interpolation function
        interp_func = interp1d(timesteps,UAV_Y)
        new_timesteps = np.arange(0, UAV_UGV_timesteps[-1],1)
        new_timesteps = np.sort(np.unique(np.concatenate((UAV_UGV_timesteps, new_timesteps))))
        UAV_Y = interp_func(new_timesteps)
        



        UGV_X = []
        UGV_Y = []
        for i in self.UGV_node_visit:
            UGV_X.append(i[0])
            UGV_Y.append(i[1])

        # create interpolation function

        timesteps = self.UGV_node_visit_time
        timesteps = [x / 1 for x in timesteps]
        


        interp_func = interp1d(timesteps, UGV_X)
        new_timesteps = np.arange(0, UAV_UGV_timesteps[-1],1)
        new_timesteps = np.sort(np.unique(np.concatenate((UAV_UGV_timesteps, new_timesteps))))
        UGV_X = interp_func(new_timesteps)

        # create interpolation function
        interp_func = interp1d(timesteps,UGV_Y)
        new_timesteps = np.arange(0,UAV_UGV_timesteps[-1],1)
        new_timesteps = np.sort(np.unique(np.concatenate((UAV_UGV_timesteps, new_timesteps))))
        UGV_Y = interp_func(new_timesteps)


        print(len(UGV_X))
        print(len(UGV_Y))


        # dictionary of lists  
        dict = {'Time': new_timesteps.tolist(), 'UAV_X': UAV_X.tolist(), 'UAV_Y': UAV_Y.tolist(), 'UGV_X': UGV_X.tolist(),'UGV_Y': UGV_Y.tolist()}  
               
        df = pd.DataFrame(dict) 
            
        # saving the dataframe 
        df.to_csv('Demo_Route{}.csv'.format(period_number))
        
               
            
    

###############################-----------------------    Main    -------------------------###############################


   
   

def run_scenario_with_metaheuristic(metaheuristic, ins, folder_name):
    
    
    full_path = os.path.join(folder_name,'actions_Heu_{}.csv'.format(metaheuristic))

    if not os.path.exists(folder_name):
       os.makedirs(folder_name) 

    start_time = time.time()
    
    cprint('\n -----Solving one scenario ----- \n', 'yellow', attrs = ['bold'])
    
    uav_data_points = []
    df = pd.read_csv((os.path.join(folder_name, 'scenarios_copy.csv')))
    uav_data_points = df['UAV loc'][ins]
    
    ugv_data_points = []
    df = pd.read_csv((os.path.join(folder_name, 'scenarios_copy.csv')))
    ugv_data_points = df['UGV loc'][ins]
    
    
    uav_data_points = ast.literal_eval(uav_data_points)
    ugv_data_points = ast.literal_eval(ugv_data_points)
    
    uav_data_points = [(round(x, 2), round(y, 2)) for x, y in uav_data_points]
    ugv_data_points = [(round(x, 2), round(y, 2)) for x, y in ugv_data_points]
    
    depot = ast.literal_eval(df['Depot'][ins])


    All_Mission_points = list(dict.fromkeys(uav_data_points + ugv_data_points))
    Targets = list(dict.fromkeys(uav_data_points + ugv_data_points))
    Locations = ugv_data_points
    starting  = ugv_data_points[depot[0]] 

    Mission_time = 0
    Planning_horizon = 1000*60  
    
    replan = Major_Replan(uav_data_points, ugv_data_points,Targets,Locations,starting, metaheuristic)
    
    fig, ax = plt.subplots() 
    for i in replan.uav_data_points:
        ax.scatter(i[0],i[1],s=40,color = 'black',marker = '.')

    for i in replan.ugv_data_points:
        ax.scatter(i[0],i[1],s=40,color = 'b',marker = '.')
        ax.set_xlim(0,12)
        ax.set_ylim(0,12)
        

    
    UAV_starting_point = replan.UAV_starting_point
    UAV_Fuels = replan.UAV_fuel
    
    
    mission_points_unvisited  =    replan.Unvisited_Mission_points 
    road_network = Locations
    UGV_location = replan.UGV_starting_point[0]
    
    
    
    '''----- minimum set cover algorithm ------'''
    stop_locs_list = replan.CP_set_cover(mission_points_unvisited, road_network, UGV_location) # it is used to make UGV traverse between them
    
    
    if len(stop_locs_list) == 1:    #when there is only 1  refuel stop
        stop_locs_list = stop_locs_list + stop_locs_list
    
    '''----- Allocation ------'''

    Allocation = replan.Allocation_function(mission_points_unvisited, road_network, UGV_location)
    
    
    
    
    ''' ---- Loop until all points rea covered ----'''
    
    No_of_times = 0
    for key, value in Allocation.items() :
        

        UGV_stops = key
        UAV_points = value
        
        UGV_start_stop = UGV_stops[0]
        UGV_end_stop = UGV_stops[1]
        
        UGV_end_stop_time = int(replan.cu_dis1(UGV_start_stop, UGV_end_stop)/replan.ugv_speed)
        
        
        Mission_points = UAV_points

        starting_location = UGV_stops[0]   # starting location of the UAV 
        starting_time = 0  # starting time of the UAV
        
        
            
        dropped_locs =  Mission_points
        
        while len(dropped_locs) != 0 : 
            
            print('---------------------------- solving -----------------------')
            
            dropped_locs = replan.UAV_planner(starting_time, starting_location, Mission_points, UGV_end_stop, UGV_end_stop_time)
            Mission_points = dropped_locs
            starting_location = UGV_end_stop
            UGV_end_stop_time = 0
            No_of_times = No_of_times + 1 
            if No_of_times == 15:
                break

    end_time = time.time() 
    total_time = end_time - start_time
    
    #plt.show()  
    
    with open(full_path, mode='a', newline='') as file:
       writer = csv.writer(file)
       if os.path.getsize(full_path) == 0:
          writer.writerow(['route', 'time'])
       writer.writerow([remove_consecutive_duplicates(replan.action_heu), total_time])       
            
            
    
        
def main():
    
    metaheuristics = ['TABU_SEARCH', 'SIMULATED_ANNEALING', 'GUIDED_LOCAL_SEARCH']
    folder_name = r'./Tours_output'

    for ins in range(100):
        processes = []
        for meta in metaheuristics:
            process = multiprocessing.Process(target=run_scenario_with_metaheuristic, args=(meta, ins, folder_name))
            processes.append(process)
            process.start()

        for process in processes:
            process.join()

if __name__ == '__main__':
    main()    
    
    
    
    
    
