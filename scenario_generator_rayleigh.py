'''
--- Rayleigh distribution ----
random scenario generators 
It generates seperate UGV points, sperate UAV points 
Any UAV point is not away > 6.5 km than any UGV point
Any UAV point is not < 2 km than its closest UGV point 

'''


# Libraries #

import numpy as np
import matplotlib.pyplot as plt
import random
from scipy.spatial import distance
import os
import pandas as pd
import scipy.stats as stats
import matplotlib
#np.random.seed(0)
#random.seed(0)
matplotlib.use('TkAgg')

# ---- initialization ------- #

output_folder = 'generated_scenarios'
if not os.path.exists(output_folder):
    os.makedirs(output_folder)


all_ugv_points = all_ugv_points = [[ 6. ,   8.  ],
 [ 5.5 ,  8.  ],
 [ 5.  ,  8.  ],
 [ 4.5  , 8.  ],
 [ 4.  ,  8.  ],
 [ 3.5 ,  8.  ],
 [ 3.   , 8.  ],
 [ 6.36 , 8.36],
 [ 6.73 , 8.73],
 [ 7.09 , 9.09],
 [ 7.45 , 9.45],
 [ 7.82 , 9.82],
 [ 8.18 ,10.18],
 [ 8.55 ,10.55],
 [ 8.91 ,10.91],
 [ 9.27 ,11.27],
 [ 9.64 ,11.64],
 [10.  , 12.  ],
 [ 6.4 ,  7.7 ],
 [ 6.8  , 7.4 ],
 [ 7.2  , 7.1 ],
 [ 7.6  , 6.8 ],
 [ 8.   , 6.5 ],
 [ 8.4   ,6.2 ],
 [ 8.8  , 5.9 ],
 [ 9.2  , 5.6 ],
 [ 9.6 ,  5.3 ],
 [10.  ,  5.  ]]

def sample_in_circle(center, radius, num_samples):
    angles = np.random.uniform(0, 2 * np.pi, num_samples)
    radii = np.sqrt(np.random.uniform(0, radius**2, num_samples))
    x_vals = center[0] + radii * np.cos(angles)
    y_vals = center[1] + radii * np.sin(angles)
    return np.column_stack((x_vals, y_vals))

    
def generate_uav_points(ugv_points, ua_p_d_f, safe_radius):
    
    rayleigh_center = [7,8.25]
    sigma = 7
    all_potential_points = np.vstack([sample_in_circle(ugv, safe_radius, ua_p_d_f) for ugv in ugv_points])
    ugv_points_set = set(map(tuple, ugv_points))
    all_potential_points = np.array([point for point in all_potential_points if tuple(point) not in ugv_points_set])
    
    if len(all_potential_points) > ua_p_d_f:
        # Calculate the distance of each point from the Rayleigh center
        distances = np.linalg.norm(all_potential_points - rayleigh_center, axis=1)
        
        # Calculate the weight for each point using a Rayleigh PDF
        weights = stats.rayleigh.pdf(distances, scale=sigma)
        weights /= weights.sum()  # Normalize the weights
        
        # Sample points based on the weights
        chosen_indices = np.random.choice(len(all_potential_points), ua_p_d_f, replace=False, p=weights)
        uav_points = all_potential_points[chosen_indices]
    else:
        uav_points = all_potential_points
    
    return uav_points




def scenario_gen(uav_graph_size=15, ugv_graph_size=5):
    ua_p_d_f = uav_graph_size
    ug_p_d_f = ugv_graph_size
    safe_radius = 7.0

    mandatory_points = [[3, 8], [10, 5], [10, 12]]
    # Assuming all_ugv_points is defined elsewhere in your code
    remaining_points = [point for point in all_ugv_points if point not in mandatory_points]
    sampled_points = random.sample(remaining_points, ug_p_d_f - len(mandatory_points))
    ugv_points = mandatory_points + sampled_points
    ugv_points_miles = np.round(np.array(ugv_points) * 0.62137, 2)
    
    assert len(ugv_points_miles) == len(set(map(tuple, ugv_points_miles))), "Duplicates found in UGV points"

    uav_points_miles = None

    while True:
        unique_uav_points_found = False
        while not unique_uav_points_found:
            uav_points = generate_uav_points(ugv_points, uav_graph_size, safe_radius)
            uav_points_miles = np.round(uav_points * 0.62137, 2)
            if len(uav_points_miles) == len(set(map(tuple, uav_points_miles))):
                unique_uav_points_found = True
        
        uav_set = set(map(tuple, uav_points_miles))
        ugv_set = set(map(tuple, ugv_points_miles))

        if uav_set.isdisjoint(ugv_set):
            break

    assert uav_set.isdisjoint(ugv_set), "Duplicates found between UAV and UGV points"
    
    fig, ax = plt.subplots()

    # Scatter plot for UAV points
    ax.scatter(uav_points_miles[:, 0]/0.62137, uav_points_miles[:, 1]/0.62137, label='UAV Points')
    
    # Scatter plot for UGV points
    ax.scatter(ugv_points_miles[:, 0]/0.62137, ugv_points_miles[:, 1]/0.62137, label='UGV Points')
    
    # Adding labels and title, if needed
    ax.set_xlabel('Distance (km)')
    ax.set_ylabel('Distance (km)')
    ax.set_title('UAV and UGV Points')
    ax.legend()
    ax.set_xlim([0,20])
    ax.set_ylim([0,20])
    
    # Show the plot
    plt.show()
    kkk
    

    return uav_points_miles, ugv_points_miles

    
    '''uav_points_miles = np.array([[6.44, 5.19],
 [3.59, 6.64],
 [1.82, 6.65],
 [2.03, 7.08],
 [3.36, 3.29],
 [3.92, 7.29],
 [4.19, 2.43],
 [2.72, 3.49],
 [5.88, 5.04],
 [2.4 , 6.59],
 [3.36, 2.56],
 [4.41, 7.9 ],
 [5.79, 4.98],
 [6.7 , 5.2 ],
 [6.13, 4.81],
 [3.66, 3.42],
 [2.55, 3.6 ],
 [3.9 , 2.64],
 [3.15, 6.44],
 [2.75, 6.94],
 [2.93, 3.46],
 [2.9 , 7.59],
 [3.05, 6.33],
 [6.89, 4.98],
 [6.15, 5.38]])'''
    
    '''np.array([[2.88 ,6.85],
 [3.76, 7.21],
 [2.51, 6.6 ],
 [3.52, 7.1 ],
 [3.25, 3.04],
 [1.68, 6.59],
 [3.09, 3.51],
 [2.19, 6.89],
 [6.43, 5.06],
 [2.06, 6.61],
 [3.17, 6.21],
 [7.19, 5.77],
 [3.53, 7.56],
 [7.2 , 4.73],
 [3.08, 3.62],
 [1.54, 6.3 ],
 [3.4 , 6.97],
 [3.23, 6.69],
 [6.36, 5.28],
 [7.83, 5.86],
 [6.32, 5.18],
 [3.6 , 8.07],
 [2.48, 3.58],
 [4.47, 2.73],
 [3.47, 3.21]])'''
    
    '''np.array([[6.32, 4.95],
                        [4.38 ,7.37],
                         [5.41, 1.95],  
                         [4.92,8.46],
                         [6.6 , 5.1],
                         [3.83, 8.18] ,
                         [3.3 , 3.32] ,
                         [0.78 ,2.79] ,
                         [2.76 ,2.7 ] ,
                         [8.5 , 8.34] ,
                         [2.29, 2.71] ,
                         [5.33, 1.28] ,
                         [7.78 ,4.7 ],
                         [2.2 , 6.25],
                         [6.96, 6.44],
                         [7.46, 2.42],
                         [3.11, 3.2 ],
                         [4.75, 2.27],
                         [1.3,  7.24],
                         [7.3,  4.95],
                         [4.87 ,9.06],
                         [5.21,0.85],
                         [6.48, 1.12],
                         [2.9 , 7.58],
                         [2.56, 3.66]])'''
    
    
    
    #return uav_points_miles, ugv_points_miles
    
    '''
    
    df = pd.read_csv('ugv_data_points_v2.csv')
    xy_tuples = df[['UGV_X', 'UGV_Y']].to_numpy()
    xy_array = np.array(list(map(tuple, xy_tuples)))
    ugv_points_miles = xy_array[:,:]
    
    df = pd.read_csv('uav_data_points_v2.csv')
    xy_tuples = df[['UAV_X', 'UAV_Y']].to_numpy()
    xy_array = np.array(list(map(tuple, xy_tuples)))
    uav_points_miles = xy_array[:,:]
    
    #uav_points_miles = xy_array[0].reshape(1,2)
    
    
    
    
    
    

    return uav_points_miles, ugv_points_miles'''
    