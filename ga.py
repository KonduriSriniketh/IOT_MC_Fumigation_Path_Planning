#!/usr/bin/env python3
#!/usr/bin/env python3
from ctypes import sizeof
import numpy as np
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
from scipy.spatial.distance import cdist


class StartFromZeroRepair(Repair):

    def _do(self, problem, X, **kwargs):
        I = np.where(X == 0)[1]

        for k in range(len(X)):
            i = I[k]
            X[k] = np.concatenate([X[k, i:], X[k, :i]])

        print("++++++")
        print(X)
        print("++++++")


        return X

class Salesman(ElementwiseProblem):

    def __init__(self, cities, **kwargs):
        """
        A two-dimensional traveling salesman problem (TSP)

        Parameters
        ----------
        cities : numpy.array
            The cities with 2-dimensional coordinates provided by a matrix where where city is represented by a row.

        """
        n_cities, _ = cities.shape

        self.cities = cities
        self.D = cdist(cities, cities)
        
        print("!!!!!!!!!!!")
        print("self.D")
        print(self.D)
        print("!!!!!!!!!!!")

        super(Salesman, self).__init__(
            n_var=n_cities,
            n_obj=1,
            xl=0,
            xu=n_cities,
            vtype=int,
            **kwargs
        )

    def _evaluate(self, x, out, *args, **kwargs):
        print("^^^^^^^^^^^^^^^^^")
        print("x =", x)
        print("self.get_route_length(x) =", self.get_route_length(x))
        print("$$$$$$$$$$$$$$$$$")
        out['F'] = self.get_route_length(x)

    def get_route_length(self, x):
        n_cities = len(x)
        dist = 0
        for k in range(n_cities - 1):
            i, j = x[k], x[k + 1]
            dist += self.D[i, j]

        last, first = x[-1], x[0]
        dist += self.D[last, first]  # back to the initial city
        return dist


def generate_random_tsp_problem(n_cities, grid_width=100.0, grid_height=None, seed=None):
    if seed is not None:
        np.random.seed(seed)
    grid_height = grid_height if grid_height is not None else grid_width
    cities = np.random.random((n_cities, 2)) * [grid_width, grid_height]
    return Salesman(cities)


problem = generate_random_tsp_problem(3, 100, seed=2)

def main():

    for i in range(2,2):
        print("i = ", i)
    

    print("PROBLEM")
    print(problem)
    robot_profile = np.zeros((1,3))
    robot_pt = np.array([0, 0, 0])
    print(robot_profile[0,0:2])
    print(robot_profile[0][2])

     

    print(robot_profile.shape)
    print(robot_pt.shape)
    
    
    
    # algorithm = GA(
    #     pop_size=2,
    #     sampling=PermutationRandomSampling(),
    #     mutation=InversionMutation(),
    #     crossover=OrderCrossover(),
    #     repair=StartFromZeroRepair(),
    #     eliminate_duplicates=True
    # )
    # print("algorithm = ", algorithm)
    # # if the algorithm did not improve the last 200 generations then it will terminate (and disable the max generations)
    # termination = DefaultSingleObjectiveTermination(period=200, n_max_gen=np.inf)

    # print("termination = ", termination)
    
    # res = minimize(
    #     problem,
    #     algorithm,
    #     termination,
    #     seed=1,
    # )

    # print("Traveling Time:", np.round(res.F[0], 3))
    # print("Function Evaluations:", res.algorithm.evaluator.n_eval)

if __name__ == '__main__':
    main()