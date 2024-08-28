#!/usr/bin/env python3
from ctypes import sizeof
from re import escape
import csv
import numpy as np
import pandas as pd
import math
import cmath
import random
from pymoo.core.repair import Repair
from pymoo.algorithms.soo.nonconvex.ga import GA
from pymoo.optimize import minimize
from pymoo.problems.single.traveling_salesman import create_random_tsp_problem
from pymoo.operators.sampling.rnd import PermutationRandomSampling
from pymoo.operators.crossover.ox import OrderCrossover
from pymoo.operators.mutation.inversion import InversionMutation
from pymoo.termination.default import DefaultSingleObjectiveTermination
from pymoo.core.problem import ElementwiseProblem
from pymoo.algorithms.soo.nonconvex.pso import PSO
import matplotlib.pyplot as plt

class StartFromZeroRepair(Repair):

    def _do(self, problem, X, **kwargs):
        I = np.where(X == 0)[1]

        for k in range(len(X)):
            i = I[k]
            X[k] = np.concatenate([X[k, i:], X[k, :i]])

        return X

class FumWirelessChragingSequence(ElementwiseProblem):

    def __init__(self, considered_waypoint_profile, distance_matrix, **kwargs):

        self.considered_waypoint_profile = considered_waypoint_profile
        # print("###############")
        # print(self.considered_waypoint_profile)
        
        self.n_waypoints, _ = considered_waypoint_profile.shape
        self.distance_matrix = distance_matrix # distance between the considered waypoints

        self.waypoints_coordinates = considered_waypoint_profile[:,0:2]
        # print(self.waypoints_coordinates)

        #robot position
        self.robot_position = self.considered_waypoint_profile[0, 0:2]
        # print("robot initial position = ",self.robot_position)
        
        #robot velocity in m/sec
        self.robot_velocity = self.considered_waypoint_profile[0][3]
        # print("robot velocity = ", self.robot_velocity)

        self.activity = considered_waypoint_profile[:, 2]  # charging or fumigation
        self.initial_charges = considered_waypoint_profile[:, 3]
        self.actual_waypoint_number = considered_waypoint_profile[:, 4]

        self.final_output_sequence = np.arange(self.n_waypoints)
        # print("self.final_output_sequence = ", self.final_output_sequence)
        self.final_total_time = 1000000

        self.iteration_counter = 0
        self.iteration = np.empty((0,1), int)
        self.total_time_np = np.empty((0,1), int)
        
        super(FumWirelessChragingSequence, self).__init__(
            n_var = self.n_waypoints,
            n_obj = 1,
            xl = 0,
            xu = self.n_waypoints,
            type_var = int,
            **kwargs
        )

    def _evaluate(self, x, out, *args, **kwargs):
        

        out['F'] = self.estimate_route_travel_time(self.n_waypoints, self.robot_position, self.robot_velocity, 
                                                   self.initial_charges, self.activity, self.distance_matrix, x)
        
        # print("seq under evaluation = ", x)
        # print("evaluated time = ", out['F'])
        # print("self.final_output_sequence = ",self.final_output_sequence)
        # print("self.final_total_time = ", self.final_total_time)
        # print("-------------------------------------")


    def estimate_route_travel_time(self, n_waypoints, robot_initial_position, robot_velocity, 
                                   initial_charges, activity, distance_matrix, x):
        
        total_time = 0
        cycle_time = 0
        charge_cur = initial_charges
        max_charge = 95
    
        for k in range(1, n_waypoints):

            dist_to_k = distance_matrix[x[k-1]][x[k]]
            robot_travel_time = dist_to_k/ robot_velocity
            charge_n1 = self.estimate_discharge(robot_travel_time, charge_cur, 0, k, activity)
            charge_time = self.estimate_time_to_charge(charge_n1[k], max_charge, activity[k])

            charge_n2 = self.estimate_discharge(charge_time, charge_n1, 1, k, activity )
            cycle_time = robot_travel_time + charge_time

            charge_cur = charge_n2
            total_time += cycle_time

        self.iteration_counter = self.iteration_counter + 1
        self.iteration = np.append(self.iteration, self.iteration_counter)

        if (total_time <=  self.final_total_time):
            
            self.final_total_time = total_time
            mission_seq = np.zeros(len(x))
            for i in range(len(x)):
                mission_seq[i] = int(self.actual_waypoint_number[x[i]])
            # print("temp = ", temp, "temp len =", len(x) )
            print("###############")
            self.final_output_sequence = mission_seq
            print("mission sequence = ", mission_seq)
            print("iteration number =",self.iteration_counter -1)
            print("self.final_output_sequence = ",self.final_output_sequence)
            print("self.final_total_time = ", self.final_total_time)
            self.total_time_np = np.append(self.total_time_np, total_time)
        else:
            self.total_time_np = np.append(self.total_time_np, self.final_total_time)

        return total_time
    
    def estimate_discharge(self, time, Qcur, mode, index, activity_type ): #Qcur charge current array for all waypoints
        
        row = len(Qcur)
        Qdis = Qcur
        if(mode == 0):
            for i in range(index, row):
                if ((activity_type[i] == 0) | (activity_type[i] == 1)):
                    Qdis[i] = 0
                elif(activity_type[i] == 2):
                    tempo = Qcur[i] - time/144
                    if(tempo <= 0):
                        Qcur[i] = 0

        elif(mode == 1):
            for i in range(index+1, row):
                if((activity_type[i] == 0) | (activity_type[i] == 1)):
                    Qdis[i] = 0
                elif(activity_type[i] == 2):
                    tempo = Qcur[i] - time/144
                    if(tempo <= 0):
                        Qcur[i] = 0 
                    #Qcur[i] = tempo

        return np.round(Qdis, 3) # Qdis charge after discharge
    
    def estimate_time_to_charge(self, current_charge, max_charge, activity_type):

        if (activity_type == 1):

            ChargeTime = 50
            return ChargeTime
        
        elif (activity_type == 2):
            DoD = max_charge - current_charge

            # Charging time = (3.2Ah * DoD)/(4A * 90)
            # 3.2Ah is capacity of the battery on the mosquito catcher
            # 4A is the charging current
            # 90 is the efficiency of the charging

            ChargeTime = (3.2*DoD)/(4*90) #This is in  hrs
            ChargeTime = np.round(ChargeTime*60*60,3)    
            return ChargeTime

def create_fum_charge_tsp(n_points):

    profile = np.zeros((n_points, 5))
    # Robot Start Position in the map (2D coordinates)
    profile[0, :] = [1, 1, 0, 0.2, 0] #Robot intial point 

    for i in range(1, n_points):
        profile[i][0] = random.uniform(0.8, 2.5)
        profile[i][1] = random.uniform(2, 4)
        profile[i][2] = random.randint(0,1)
        profile[i][3] = random.uniform(0,100)
        profile[i][4] = i
    profile = np.round(profile, 3)

    filtered_rows = []
    # Iterate over each row in the original matrix
    for row in profile:
        # Check the filtration criteria
        if row[2] == 0 or (row[2] == 1 and row[3] < 80):
            # Add the row to the filtered list
            filtered_rows.append(row)
    way_points_considered = np.array(filtered_rows)

    num_of_rows = way_points_considered.shape[0]
    distance_matrix = np.round(np.random.rand(num_of_rows, num_of_rows), 3)

    return (way_points_considered, distance_matrix, profile)

def get_problem_inputs():

    waypoints_csv = pd.read_csv('/home/sutd/test_ws/src/Astar/scripts/point_profile.csv')

    # Convert DataFrame to NumPy array
    profile = waypoints_csv.to_numpy()
    profile = np.round(profile, 3)
    filtered_rows = []
    # Iterate over each row in the original matrix
    for row in profile:
        # Check the filtration criteria
        if row[2] == 0 or row[2] == 1 or (row[2] == 2 and row[3] < 80):
            # Add the row to the filtered list
            filtered_rows.append(row)
    way_points_considered = np.array(filtered_rows)
    


    distances_matrix_df = pd.read_csv('/home/sutd/test_ws/src/Astar/scripts/distance_matrix.csv', index_col=False)
    total_distance_matrix = distances_matrix_df.iloc[:, 1:].values

    num_of_rows = way_points_considered.shape[0]
    distance_matrix = np.zeros((num_of_rows, num_of_rows))
    for i in range(0, num_of_rows):
        for j in range(0, num_of_rows):
            row = int(way_points_considered[i][4])
            column = int(way_points_considered[j][4])
            distance_matrix[i][j] = total_distance_matrix[row][column]

    return (way_points_considered, distance_matrix, profile)

def write_to_csv(data, filename):
    # Reshape the array to have one column
    data_reshaped = data.reshape(-1, 1)

    # Write the reshaped data to CSV file
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['optimized_sequence'])
        for row in data_reshaped:
            writer.writerow(row)

def main():
    considered_points, dist_matrix, all_points = get_problem_inputs()
    
    print("all points considered")
    print(all_points)
    print("------------------------")
    print("considerd way_points")
    print(considered_points)
    print("-------------------------")
    problem = FumWirelessChragingSequence(considered_waypoint_profile = considered_points, distance_matrix = dist_matrix )

    algorithm = GA(
        pop_size=10,
        sampling=PermutationRandomSampling(),
        mutation=InversionMutation(),
        crossover=OrderCrossover(),
        repair=StartFromZeroRepair(),
        eliminate_duplicates=True
    )

    # if the algorithm did not improve the last 200 generations then it will terminate (and disable the max generations)
    termination = DefaultSingleObjectiveTermination(period=200, n_max_gen=np.inf)

    
    res = minimize(
        problem,
        algorithm,
        termination,
        seed=1,
    )


    print("final output sequence = ", problem.final_output_sequence)
    print("final total time = ", problem.final_total_time)

    filename = '/home/sutd/test_ws/src/Astar/scripts/optimed_path.csv'
    write_to_csv(problem.final_output_sequence, filename)
    print(f'Data has been written to {filename}.')

    # fig=None
    # ax=None
    show=True
    label=True
    # if fig is None or ax is None:
    #     fig, ax = plt.subplots(1)
    
    plt.figure()
    plt.plot (problem.iteration, problem.total_time_np)
    plt.title('Convergence of the total time')
    plt.xlabel('iterations')
    plt.ylabel('total time')
    plt.grid(linestyle='dotted')

    if show:
        print("TRUE")
        plt.show()


if __name__ == '__main__':
    main()
