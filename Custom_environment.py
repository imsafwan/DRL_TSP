# -*- coding: utf-8 -*-
"""


@author: safwan
"""


'''

Custom environment for the scenario

Following Attention learn to solve routing
here, I have used (x, y, o ) representation in encoder for mp

v6 : to incorporate the changes of mission completion criteria. i.e, When all mission poiints are visited and UAV completes its recharging



'''


import os
import gym
from gym import spaces
import numpy as np
import networkx as nx
import importlib.util
#from ortools.sat.python import cp_model
#import matplotlib.pyplot as plt
import pandas as pd
import time
#from ortools.constraint_solver import routing_enums_pb2
#from ortools.constraint_solver import pywrapcp
import math
import random
import gym
from gym import spaces
from torch.autograd.profiler import profile, record_function
import torch
import csv
from termcolor import cprint


device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

print(device)


import torch


class scenario(): 
    
     
         
         
     def __init__(self, csv):
         
         self.df = pd.read_csv(csv)
         self.ugv_data_points = [(self.df['UGV_X'][i], self.df['UGV_Y'][i]) for i in range(len(self.df['UGV_X']))]
         self.uav_fuel = 287700
         self.batch_size = 256
         
         # define UGV road network as a graph #
         G = nx.Graph()
         
         for coord in self.ugv_data_points:
                 G.add_node(coord)
         
         for i in range(0,6):
                  weight = int(np.sqrt((self.ugv_data_points[i+1][0] - self.ugv_data_points[i][0])**2 + (self.ugv_data_points[i+1][1] - self.ugv_data_points[i][1])**2)*5280)
                  G.add_edge(self.ugv_data_points[i], self.ugv_data_points[i+1], weight=weight)
         for i in range(7,18):
                  weight = int(np.sqrt((self.ugv_data_points[i+1][0] - self.ugv_data_points[i][0])**2 + (self.ugv_data_points[i+1][1] - self.ugv_data_points[i][1])**2)*5280)
                  G.add_edge(self.ugv_data_points[i], self.ugv_data_points[i+1], weight=weight)
         for i in range(19,29):
                  weight = int(np.sqrt((self.ugv_data_points[i+1][0] - self.ugv_data_points[i][0])**2 + (self.ugv_data_points[i+1][1] - self.ugv_data_points[i][1])**2)*5280)
                  G.add_edge(self.ugv_data_points[i], self.ugv_data_points[i+1], weight=weight)
         
         
         nodes_list = list(G.nodes())
         dist_matrix = np.zeros((len(nodes_list), len(nodes_list))) 
         for i, node1 in enumerate(nodes_list):
             for j, node2 in enumerate(nodes_list):
                 try:
                     dist_matrix[i][j] = nx.shortest_path_length(G, source=node1, target=node2, weight='weight')
                 except nx.NetworkXNoPath:
                     dist_matrix[i][j] = float('inf')  # or some large number
         
         
         self.dist_matrix = dist_matrix # distance matrix among all points in the road network
         self.nodes_list = nodes_list   # all nodes on the road network
         
         N = len(nodes_list)
         paths_tensor = torch.full((N, N, N), -1, dtype=torch.int64, device=device)  # Using -1 for padding


         for i, node1 in enumerate(nodes_list):
            for j, node2 in enumerate(nodes_list):
                    try:
                        path = nx.shortest_path(G, source=node1, target=node2, weight='weight')
                        path_indices = [nodes_list.index(node) for node in path]  # Convert node to index
                        path_length = len(path_indices)
                        if path_length <= N:
                            for k in range(path_length):
                                paths_tensor[i, j, k] = path_indices[k]
                        else:
                            # Optionally handle paths longer than N
                            print(f"Path from {node1} to {node2} exceeds maximum length N.")
                    except nx.NetworkXNoPath:
                        continue 
         
         
         self.path_storage = paths_tensor # to get path between two points on the raod network


     
     def path_retrieve(self, node1, node2, UGV_points, N_uav):
         
        '''
         

         Parameters
         ----------
         node1 : tensor
             point 1
         node2 : tensor
             point 2
         UGV_points : tensor
             UGV points that are present in a scenario
         N_uav : Int
             Number of UAV points

         Returns
         -------
         unique_path_indices_mask : tensor
             indices of the ugv points that are present

        '''
       
        #st = time.time()    
        A = self.nodes_list
        A_tensor = torch.tensor(A, dtype=torch.float32).unsqueeze(0).repeat(node1.size(0), 1, 1).to(device)  # [B, N, 2]
        match1 = (A_tensor == node1.unsqueeze(1)).all(-1)  # [B, N]
        valid1, ix1 = match1.max(1)  # [B]
        match2 = (A_tensor == node2.unsqueeze(1)).all(-1)  # [B, N]
        valid2, ix2 = match2.max(1)  # [B]
        paths_tensor = self.path_storage    # [N, N, M]
        
        
        paths = paths_tensor[ix1, ix2,:] # [ B, M]
        
        paths = paths.long()  # Convert paths to long, as required for indexing
 
        
        batch_indices = torch.arange(0, A_tensor.size(0), device=device).unsqueeze(1)
     
        
        batch_indices = batch_indices.expand(-1, paths.size(1))
     
        A1_tensor = torch.tensor(A+[(20,20)], dtype=torch.float32).unsqueeze(0).repeat(node1.size(0), 1, 1).to(device)  # [B, N, 2]
        path_coordinates = A1_tensor[batch_indices, paths, :]  # [B, M, 2]
        
        
        
        UGV_points_exp = UGV_points.unsqueeze(1)  # Shape: [B, 1, N, 2]
        path_coordinates_exp = path_coordinates.unsqueeze(2)  # Shape: [B, M,1,  2]
 
        
        matches = (UGV_points_exp ==  path_coordinates_exp).all(dim=-1)  
        any_match = matches.any(dim=1)
        
        unique_path_indices_mask = torch.cat((any_match, torch.zeros(self.batch_size, N_uav, device = device).bool()), dim = 1)
        
        #print('path time --->', time.time()- st)
        return unique_path_indices_mask       
         
         
         
         
         
     def cu_dis1(self, node1, node2):
          
        cu_dis, A = self.dist_matrix, self.nodes_list
        A_tensor = torch.tensor(A, dtype=torch.float32).unsqueeze(0).repeat(node1.size(0), 1, 1).to(device)  # [B, N, 2]
        
        match1 = (A_tensor == node1.unsqueeze(1)).all(-1)  # [B, N]
        valid1, ix1 = match1.max(1)  # [B]
        match2 = (A_tensor == node2.unsqueeze(1)).all(-1)  # [B, N]
        valid2, ix2 = match2.max(1)  # [B]
        cu_dis_tensor = torch.tensor(cu_dis, dtype=torch.float32).to(device)  # [N, N]
        distances_tensor = torch.zeros(node1.size(0), dtype=torch.float32, device=device)
        valid_indices = valid1 & valid2
        
        distances_tensor[valid_indices] = cu_dis_tensor[ix1[valid_indices], ix2[valid_indices]]
        #print('Cus_dis time --->', time.time()- st)
        
        return distances_tensor
                       
     def cu_dis2(self, node1, node2):
        '''
        

        Parameters
        ----------
        node1 : tensor [ B, 2 ]
            one UGV point
        node2 : tensor [ B, M, 2 ]
            other UGV points

        Returns
        -------
        distances_tensor : tensor
            distance between node1 and other points in node2
            It helps in masking in feasibility function

        '''
        
        # node1 = [ B, 2], node2 = [B, M, 2]
    
        cu_dis, A = self.dist_matrix, self.nodes_list
        
        A_tensor = torch.tensor(A, dtype=torch.float32).unsqueeze(0).repeat(node1.size(0), 1, 1).to(device)  # [B, N, 2]
        A = self.nodes_list
        
        node2_expanded = node2.unsqueeze(2)  # Shape [B, M, 1, 2]
        A_tensor_expanded = A_tensor.unsqueeze(1)  # Shape [B, 1, N, 2]
        
        matches = (node2_expanded == A_tensor_expanded).all(-1)  # Shape [B, M, N]
        valid_cols, col_indices = matches.max(2) # [ B, M]
        
        
        valid_rows, rows_indices = (A_tensor == node1.unsqueeze(1)).all(-1).max(1)  # [B]
        rows_indices = rows_indices.unsqueeze(1).repeat(1, node2.size(1)) # [ B, M]
        
        distances_tensor = torch.zeros(node1.size(0), node2.size(1), dtype=torch.float32, device=device) #[ B, M]
        
        cu_dis_tensor = torch.tensor(cu_dis, dtype=torch.float32, device=device)  # Shape [N, N]
        
        
        
        selected_distances = cu_dis_tensor[rows_indices, col_indices]
        
        distances_tensor = torch.zeros(node1.size(0), node2.size(1), dtype=torch.float32, device=device) #[ B, M]
        valid_rows = valid_rows.unsqueeze(-1)  #[B, 1]
        valid = valid_rows & valid_cols # [ B. M]
        
        
        distances_tensor[valid] = selected_distances[valid]
        
        return distances_tensor
        
        
                              
                  
                        
                       

class UAV_UGV_Env(gym.Env):
    
    
    def __init__(self):
        
         
         '''-------------------------------------------------------------------''' 
        
         self.scene = scenario('ugv_road.csv')
         self.batch_size = self.scene.batch_size
         self.uav_speed = 33  #constant
         self.ugv_speed = 15  #constant
         self.solver_time = 10 #constant
         self.mission_starting_depot_coord = (0.6,8.07)                        #constant
         self.Rendzvous_locs = [(2.77, 7.57), (7.62, 3.69)]                    #constant
         self.max_planning_horizon = 1 #250*60                                    #constant
         self.prv_action = [None] * self.batch_size                            #initialize
         self.uav_mission_points = torch.rand( 100, 2, device = device)                           # tensor [N, d]
         self.ugv_mission_points = torch.rand( 100, 2, device = device)                           # tensor [N, d] 
         
         self.uav_fuel_constant = 287700
         
         '''-------------------------------------------------------------------'''
                          
         
          
         self.all_mission_points = torch.cat([self.uav_mission_points, self.ugv_mission_points], dim=0)                    # tensor [N, d] 
         self.unique_mission_points = torch.cat([self.uav_mission_points, self.ugv_mission_points], dim=0) 
         self.unique_mission_points_np = torch.tensor(pd.DataFrame(self.unique_mission_points.cpu().numpy()).drop_duplicates(keep='first').values, device=self.unique_mission_points.device, dtype=self.unique_mission_points.dtype)
         # tensor [N, d]
         self.unique_mission_points = self.unique_mission_points_np.unsqueeze(0)                                           # tensor [1, N_unique, d]
         
                                              
                                              
         ones = torch.ones(1, self.unique_mission_points.size(1), 1, device = device)
         zeros = torch.zeros(1, self.ugv_mission_points.size(0), 1, device = device)
         
         unique_mission_points_with_ones = torch.cat([self.unique_mission_points, ones], -1)
         ugv_mission_points_with_zeros = torch.cat([self.ugv_mission_points.unsqueeze(0), zeros], -1)
         
         self.encoder_mission_space = torch.cat([unique_mission_points_with_ones, ugv_mission_points_with_zeros], 1).repeat(self.batch_size, 1, 1)
         
         
         
         

         
         self.agent =  torch.full((self.batch_size,), 1, dtype=torch.float32, device = device)                              # tensor [B]
         self.uav_fuel_limit = torch.full((self.batch_size,), 287700.0, dtype=torch.float32, device = device)               # tensor [B]
         self.uav_fuel = torch.full((self.batch_size,), 287700.0, dtype=torch.float32, device = device)                     # tensor [B]
         self.uav_pos_coord = torch.stack([self.uav_mission_points[-1]] * self.batch_size)                                  # tensor [B, d]
         self.ugv_pos_coord = torch.stack([self.ugv_mission_points[0]] * self.batch_size)                                   # tensor [B, d]
         self.refuel_stop = torch.stack([self.ugv_mission_points[0]] * self.batch_size)                                     # [B,d]
         self.depot_index = torch.stack([self.uav_mission_points[-1]] * self.batch_size)   
         self.mission_status = torch.zeros(self.batch_size, len(self.unique_mission_points[0]), device = device)            # tensor [B, N]
         self.mission_time_elapsed = torch.zeros(self.batch_size, device = device)                                          # tensor [B, N] 
         self.infeasibility = torch.zeros(self.batch_size, device = device).bool()                                          # tensor [B]
         self.recharging_location = torch.zeros(self.batch_size, device = device).bool()                                    # tensor [B]
         self.last_score = torch.zeros(self.batch_size, len(self.unique_mission_points[0]), device = device)                # tensor [B, N]
         self.refuel_stop_time = torch.zeros(self.batch_size, device = device)                                              # tensor [B]
         
         self.UGV_time = torch.zeros(self.batch_size, device = device)
         self.done_completely = torch.zeros(self.batch_size, device = device) 
         
         
         
         '''-------------------------------------------------------------------'''
         
         
         

    def calculate_distances(self, p1, p2):
          # To do: replace with eu_dis1   Compute Euclidean distance between two sets of points
         return torch.norm((p1 - p2), dim=-1) * 5280 #torch.sqrt(torch.sum((p1 - p2) ** 2, dim=-1))*5280
    
    def eu_dis1(self, a, b):
       a = a.unsqueeze(1)
       b = b.unsqueeze(1)
       diff = a - b
       distance = torch.norm(diff, dim=2) * 5280
       return distance.permute(1,0)[0]
   
    
        
    def step(self, actions, step):
        
        
        if step == 0:
            if not ((self.refuel_stop_time == 0).all()):
                non_zero_elements = self.refuel_stop_time[self.refuel_stop_time != 0]
                print(f"Non-zero elements in refuel_stop_time: {non_zero_elements}")
                cprint(self.refuel_stop_time[0], 'cyan')
            assert (self.refuel_stop_time == 0).all()
        
        
        action_indices = actions
        self.prv_action = actions

        batch_size = self.batch_size
        assert batch_size == actions.size(0)
        

        infeasible_penalties = torch.zeros(batch_size, device = device)        # initialize infeasible penalties 

        
        #------------ action coordinates ---------- #
        
        is_refuel_stop = action_indices < len(self.ugv_mission_points[0])       # action : recharge
        is_refuel_stop = is_refuel_stop.unsqueeze(-1)
        
        is_visit_point = (action_indices >= len(self.ugv_mission_points[0]))    # action: visiting mission points
        is_visit_point = is_visit_point.unsqueeze(-1)


        if self.encoder_mission_space.shape[0] != batch_size:
               self.encoder_mission_space = self.encoder_mission_space.repeat(batch_size, 1, 1)
        
               
        temp_refuel_points = self.encoder_mission_space[:, :, :2].clone() 
        
        refuel_coords = torch.where(is_refuel_stop, temp_refuel_points[torch.arange(batch_size, device = device), action_indices], torch.zeros(action_indices.shape[0], 2, device = device))
        visit_coords = torch.where(is_visit_point, temp_refuel_points[torch.arange(batch_size, device = device), action_indices], torch.zeros(action_indices.shape[0], 2, device = device))
        
        
        action_coords = refuel_coords + visit_coords                            # action coordinates [B, 2]
        
        

        # ------- unique action indices ------ #
        
        expanded_action_coords = action_coords.unsqueeze(1)
        matches = torch.all(self.unique_mission_points == expanded_action_coords, dim=-1)  # Shape: [B, N]
        matches_int = matches.int()
        unique_action_indices = torch.argmax(matches_int, dim=-1)                          # indices of action shape [B]
        assert batch_size == len(unique_action_indices)
        
        
        
        is_refuel_stop = is_refuel_stop.any(dim=1)
        is_visit_point = is_visit_point.any(dim=1)
        
        
        # ------- effect of actions --------- #
        
        
        
        step_elapsed_times_uav = (torch.norm((action_coords - self.uav_pos_coord), dim=1)*5280 / self.uav_speed)*self.agent 
        self.uav_fuel = self.uav_fuel - (step_elapsed_times_uav * 198).int() # fuel depletion
        fuel_depleted_mask = self.uav_fuel < 0 # Check for fuel levels less than zero for each batch instance
        infeasible_penalties = torch.where(fuel_depleted_mask, torch.ones(batch_size, device = device), torch.zeros(batch_size, device = device))
        
        step_elapsed_times_ugv = (self.scene.cu_dis1(action_coords, self.ugv_pos_coord)/ self.ugv_speed)*(1 - self.agent)
        
        
        mask_agent_a = (self.agent ==1)
        mask_agent_g = (self.agent == 0)
        mask_Re_a = (is_refuel_stop*self.agent == 1)
        mask_Re_g = (is_refuel_stop*(1-self.agent) == 1)
        
        
        
        
        self.refuel_stop_time_uav[mask_Re_a] =  self.refuel_stop_time_uav[mask_Re_a] + (self.uav_time_elapsed + step_elapsed_times_uav)[mask_Re_a]
        
        
        recharge_time = torch.where(mask_Re_g, ((287700 - self.uav_fuel)/311), torch.zeros_like(action_indices).float().to(device))
        

        waiting_time = torch.where(mask_Re_g, (  (self.ugv_time_elapsed + step_elapsed_times_ugv) - self.refuel_stop_time_uav ) , torch.zeros_like(action_indices, device = device).float())  # waiting time for recharging
        waiting_time = torch.clamp(waiting_time, min=0) # if UGV arrives earlier UAV will have no waiting time
        

        step_elapsed_times_uav = step_elapsed_times_uav + recharge_time + waiting_time # accounts for recharging time
        
        exceeded_indices = torch.where(self.done_completely == 1)[0]
        if exceeded_indices.numel() > 0:
               step_elapsed_times_uav[exceeded_indices] = 0
        
        
        
        # -------- reward collection -------- #
        
        
        reward_collect = step_elapsed_times_uav/60    #[B]
        
        
        
            
        # ----------- update state --------- #
        
        
        self.mission_status[torch.arange(self.batch_size, device = device)[mask_agent_a], unique_action_indices[mask_agent_a]] = 1 # updates age period of visited mission points as 0
        
        ugv_path_indices =  self.scene.path_retrieve(self.ugv_pos_coord, action_coords, self.ugv_mission_points, self.uav_mission_points.size(1))
        self.mission_status[(ugv_path_indices* mask_agent_g.unsqueeze(1))] = 1
        
 
        
        #print('mission status --->', self.mission_status[0] )
        
        self.uav_time_elapsed = self.uav_time_elapsed + step_elapsed_times_uav
        
        self.ugv_time_elapsed = self.ugv_time_elapsed + step_elapsed_times_ugv
        self.ugv_time_elapsed[mask_Re_g] = self.uav_time_elapsed[mask_Re_g]
        
        self.mission_time_elapsed = self.mission_time_elapsed + step_elapsed_times_uav  # mission time elapsed
        
        
        
        
        
        #print('mission time ----->', self.mission_time_elapsed/60)
        self.uav_pos_indices = actions.unsqueeze(1) 
        self.uav_pos_coord[mask_agent_a] = action_coords[mask_agent_a]
        self.ugv_pos_coord[mask_agent_g] = action_coords[mask_agent_g]
        self.uav_fuel[mask_Re_g] = torch.full((self.batch_size,), 287700.0, dtype=torch.float32).to(device) [mask_Re_g]
        self.agent[mask_Re_a] = 0
        self.agent[mask_Re_g] = 1
        
        
        self.recharging_location[is_visit_point] = torch.ones(self.batch_size).bool().to(device)[is_visit_point]    #   in those instances where after visiting self.recharging_location becomes false
        self.recharging_location[mask_Re_g] = torch.zeros(self.batch_size).bool().to(device)[mask_Re_g]   #   in those instances where after recharging self.recharging_location becomes false
        
        
        
        self.refuel_stop_index[mask_Re_a] = actions[mask_Re_a]
        self.refuel_stop[mask_Re_a] = action_coords[mask_Re_a]
        self.refuel_stop_time_uav[mask_Re_a] = self.mission_time_elapsed.clone()[mask_Re_a]
        
        

        # --------- reward from step ---------- #
        

        rewards = reward_collect 
        
        # -------- call state ------- #
        
        states = self.get_observation() 
        exceeded_indices = torch.where(self.done_completely == 1)[0]
        
    
        
        if exceeded_indices.numel() > 0:
            shrink = exceeded_indices.shape[0] 
        else :
            shrink = 0
        
        
        step_condition = torch.tensor([step > 40], dtype=torch.bool, device = device)
        shrink_condition = torch.tensor([shrink > int(batch_size*0.90)], dtype=torch.bool, device = device)
        
        
        
       
        
        
        dones = torch.logical_and(torch.all(self.mission_status == 1, dim=1), mask_Re_g)
        dones = torch.logical_or(dones, self.infeasibility)
        
        self.done_completely = dones.clone().int()
        
        if self.done_completely[0] == 1 :
            cprint('<------------------Done-------------------->', 'yellow', attrs = ['bold'])
        
        exceeded_indices = torch.where(self.done_completely == 1)[0]
        if exceeded_indices.numel() > 0:
            self.agent[exceeded_indices] = 0
        

        #dones = torch.logical_or(torch.all(self.mission_status == 1, dim=1), self.infeasibility)
        dones = torch.logical_or(dones, step_condition)
        dones = torch.logical_or(dones, shrink_condition)
        dones = dones.to(device)
        
        rewards = rewards.to(device)
        if dones.all():
           unfinished_batch_indices = torch.where(self.done_completely != 1)[0]
           rewards[unfinished_batch_indices] = 800 # this acts like a penalty for those cases where all mission points are not visited
           
           for i, time1 in enumerate(self.mission_time_elapsed):
               if time1 == 0:
                  cprint(f"Instance {i} has a value of zero", 'cyan', attrs = ['bold'] )
                  print(self.encoder_mission_space[i])
                  cprint(self.depot_index[i], 'cyan', attrs = ['bold'])
        
        info = {}  
        
        
        return states, rewards, dones, info





    def reset(self, input):
       
    
         self.uav_mission_points =  input['uav loc']          # tensor [B, N, d]
         self.batch_size = self.uav_mission_points.size(0)
         
         
         self.ugv_mission_points =   input ['ugv loc']           # tensor [B, N, d]
         
         
         
         self.unique_mission_points = input['unique loc']     # tensor [B, N, d]
         
         self.encoder_mission_space = input['encoder space']  # tensor [B, N, d]
         
         self.agent =  torch.full((self.batch_size,), 1, dtype=torch.float32, device = device)

         depot_ix = input['depot'].clone()              # [B, 1]
         self.depot_index = depot_ix
         self.uav_pos_coord = input['ugv loc'][torch.arange(self.batch_size).unsqueeze(1),depot_ix].squeeze(1)      # tensor [B, d]
         self.ugv_pos_coord = input['ugv loc'][torch.arange(self.batch_size).unsqueeze(1),depot_ix].squeeze(1)      # tensor [B, d]
         self.refuel_stop =   input['ugv loc'][torch.arange(self.batch_size).unsqueeze(1),depot_ix].squeeze(1)      # tensor [B, d]   
         self.refuel_stop_index = self.depot_index.squeeze(1) 
         self.uav_pos_indices = depot_ix.clone()                 # tensor [B,1] 
         self.uav_fuel_limit = torch.full((self.batch_size,), 287700.0, dtype=torch.float32, device = device)               # tensor [B]
         self.uav_fuel = torch.full((self.batch_size,), 287700.0, dtype=torch.float32, device = device)                     # tensor [B]
         self.refuel_stop_time = torch.zeros(self.batch_size, device = device)      
         self.refuel_stop_time_uav = torch.zeros(self.batch_size, device = device)      
             
  
         self.mission_status = torch.zeros_like(self.unique_mission_points, device = device)[:,:,0]  # tensor [B, N]
         self.mission_time_elapsed = torch.zeros(self.batch_size, device = device)                   # tensor [B]
         self.uav_time_elapsed = torch.zeros(self.batch_size, device = device)                   # tensor [B]
         self.ugv_time_elapsed = torch.zeros(self.batch_size, device = device)                   # tensor [B]
         self.infeasibility = torch.zeros(self.batch_size, device = device).bool()                   # tensor [B]
         self.recharging_location = torch.zeros(self.batch_size, device = device).bool()             # tensor [B]
         
         self.prv_action = [None] * self.batch_size                                               #initialize
         
         
         
         self.done_completely = torch.zeros(self.batch_size, device = device)
         
         
         state = self.get_observation()
         
         return state
    
    
    
    
    def get_observation(self):
       return (self.uav_pos_indices, self.uav_fuel, self.mission_status)

            
        
    
    def feasible_action(self):
       '''
       Making for feasible action 
       
       '''
        
       
        
       batch_size = self.batch_size
       N = self.encoder_mission_space.shape[1]
       assert batch_size == self.uav_pos_coord.size(0)

       if self.ugv_mission_points.dim() == 2:
                 self.ugv_mission_points = self.ugv_mission_points.unsqueeze(0).repeat(batch_size, 1, 1)   # [B, N, d ]
                 
       if self.uav_mission_points.dim() == 2:
              self.uav_mission_points = self.uav_mission_points.unsqueeze(0).repeat(batch_size, 1, 1)  # [B, N, d ]
       
       
       ''' ----- repeated feasible region --- '''
       repeated_feasible_region = (self.mission_status != 1).to(device).int()       
       tensor1 = torch.ones([batch_size, len(self.ugv_mission_points[0])], device=device)
       repeated_feasible_region = torch.cat((tensor1, repeated_feasible_region), dim=1)  # [ B, N]

       
       '''# ----- ugv feasible region ------ #'''
       
       action_positions = self.encoder_mission_space[:,:,:2]                   # [ B, N ,2] # position of action points
       ugv_pos = self.ugv_pos_coord    # [ B,2]     # position of UGV
       
       dist_to_action_nodes = self.scene.cu_dis2(ugv_pos, action_positions)/self.ugv_speed  # Shape: [B, M]
       time_to_action_nodes = self.ugv_time_elapsed.unsqueeze(1).repeat(1, dist_to_action_nodes.size(1) ) + dist_to_action_nodes
       
       
       cutoff_expanded = self.refuel_stop_time_uav.unsqueeze(-1)  # Shape [B, 1] to allow broadcasting with A's shape [B, M]
       ugv_time_feasible_mask = ( time_to_action_nodes <= cutoff_expanded).float().to(device)  # This creates a boolean mask, but with True where time_value <= cutoff
       
       
       ugv_points_slice = slice(len(self.ugv_mission_points[0]), 2*len(self.ugv_mission_points[0])) 
       ugv_feasible_mask = torch.zeros(batch_size, self.encoder_mission_space.shape[1], device = device)
       ugv_feasible_mask[:, ugv_points_slice] = ugv_time_feasible_mask[:, ugv_points_slice] 
       ugv_feasible_mask[torch.arange(batch_size), self.refuel_stop_index] = 1.0
       
       ugv_feasible_mask = ugv_feasible_mask * repeated_feasible_region
       
       # when all points are visited UGV needs to do recharge action 
       exceeded_indices = torch.where(torch.all(self.mission_status == 1, dim=1))[0]
       if exceeded_indices.numel() > 0:
         ugv_feasible_mask[exceeded_indices, :] = 0 
         ugv_feasible_mask[torch.arange(batch_size), self.refuel_stop_index] = 1.0
       
       
       
       '''# -------- uav feasible mask -----------#'''
       

       
       action_positions = self.encoder_mission_space[:,:,:2]
       expanded_uav_pos = self.uav_pos_coord.unsqueeze(1).expand(-1, N, -1)
       dist_to_uav_nodes = self.calculate_distances(expanded_uav_pos, action_positions)
       action_positions_expanded = action_positions.unsqueeze(2)  # Shape: [B, N, 1, 2]
       ugv_positions_expanded = self.ugv_mission_points.unsqueeze(1)  # Shape: [B, 1, N, 2]
       dist_to_ugv_nodes = self.calculate_distances(action_positions_expanded, ugv_positions_expanded)  # Shape: [B, N, N]
       min_dist_to_ugv,  min_ugv_indices = torch.min(dist_to_ugv_nodes, dim=2) #[0]  # Shape: [B, N]
       total_distance = dist_to_uav_nodes + min_dist_to_ugv
       return_feasible_region = (((total_distance/ self.uav_speed)*198).int() <= (self.uav_fuel.unsqueeze(1)).int()).to(device) 
       
       
       rows_with_all_zeros = ~torch.any(return_feasible_region, dim=1)
       

                                                                     #
       uav_feasible_mask = torch.zeros(batch_size, self.encoder_mission_space.shape[1], device = device)
       
       uav_feasible_mask[:,:] = return_feasible_region
       uav_feasible_mask = uav_feasible_mask * repeated_feasible_region
       assert torch.any(uav_feasible_mask, dim=1).all(), "A row with all zeros (False) found in the mask."
       
       # after recharging vsisiting is performed #
       a = uav_feasible_mask.clone()
       ugv_indices = torch.where(~self.recharging_location)[0].to(device)
       uav_indices = torch.where(self.recharging_location)[0].to(device)
       ugv_slice = slice(0, len(self.ugv_mission_points[0]))
       uav_slice = slice(len(self.ugv_mission_points[0]), self.encoder_mission_space.shape[1])
       uav_feasible_mask[ugv_indices, ugv_slice] = torch.zeros(batch_size, len(self.ugv_mission_points[0]), device = device)[ugv_indices]   # makes sure after recharging visiting action is performed
       
       # if after one recharging no feasible visiting action is available #
       rows_with_all_zeros = ~torch.any(uav_feasible_mask, dim=1)
       if torch.any(rows_with_all_zeros):
             rows_with_all_zeros_indices = torch.where(rows_with_all_zeros)[0]
             uav_feasible_mask[rows_with_all_zeros_indices] = a.clone()[rows_with_all_zeros_indices]
             
       # when all points are visited UAV needs to return to UGV       
       exceeded_indices = torch.where(torch.all(self.mission_status == 1, dim=1))[0]
       if exceeded_indices.numel() > 0:
         uav_feasible_mask[exceeded_indices, :] = 0 
         uav_feasible_mask[exceeded_indices, ugv_slice] = return_feasible_region.float()[exceeded_indices, ugv_slice]

       assert torch.any(uav_feasible_mask, dim=1).all(), "A row with all zeros (False) found in the mask."
       
       '''# ------ feasible mask ------ #'''
       
       feasible_mask = uav_feasible_mask * self.agent.view(batch_size, 1) + ugv_feasible_mask * (1-self.agent).view(batch_size, 1)
       
 
       

       assert torch.any(feasible_mask, dim=1).all(), "A row with all zeros (False) found in the mask."
       
       
       '''#   ------further refining ------- #'''
       
       if self.prv_action != [None]* self.batch_size :
           f_mask = self.prv_action < len(self.ugv_mission_points[0])                       # same refuel action = same visit action is avoided
           indices = torch.arange(self.batch_size, device=device)[f_mask]
           feasible_mask[indices, self.prv_action[indices] + len(self.ugv_mission_points[0])] = 0
       
       
       # once mission is done, previous action is repeated, to make sure sequence length is equal in all intsnaces in a batch #
       exceeded_indices = torch.where(self.done_completely == 1)[0]
       if exceeded_indices.numel() > 0:
         feasible_mask[exceeded_indices, :] = 0 
         feasible_mask[exceeded_indices, self.prv_action[exceeded_indices]] = 1
       
       
            
       assert torch.any(feasible_mask, dim=1).all(), "A row with all zeros (False) found in the mask."

       return feasible_mask




'''rows_with_all_zeros = ~torch.any(feasible_mask, dim=1)
rows_with_all_zeros_indices = torch.where(rows_with_all_zeros)[0]
if torch.any(rows_with_all_zeros):
     rows_with_all_zeros_indices = torch.where(rows_with_all_zeros)[0]
     print("Indices of rows with all zeros:", rows_with_all_zeros_indices.tolist())
     print(self.prv_action[rows_with_all_zeros_indices.tolist()[0]])
     print(return_feasible_region[rows_with_all_zeros_indices.tolist()[0]])
     print(feasible_mask[rows_with_all_zeros_indices.tolist()[0]])
     
     kk'''





































