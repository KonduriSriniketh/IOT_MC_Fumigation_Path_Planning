#!/usr/bin/env python3
import numpy as np
from pymoo.core.repair import Repair
from pymoo.algorithms.soo.nonconvex.ga import GA
from pymoo.optimize import minimize
from pymoo.problems.single.traveling_salesman import create_random_tsp_problem
from pymoo.operators.sampling.rnd import PermutationRandomSampling
from pymoo.operators.crossover.ox import OrderCrossover
from pymoo.operators.mutation.inversion import InversionMutation
from pymoo.termination.default import DefaultSingleObjectiveTermination

class StartFromZeroRepair(Repair):

    def _do(self, problem, X, **kwargs):
        I = np.where(X == 0)[1]
        # print(I)

        for k in range(len(X)):
            i = I[k]
            X[k] = np.concatenate([X[k, i:], X[k, :i]])
        # print ("---------------")
        # print ("X =")
        # print (X)
        # print ("###############")
        return X
    



problem = create_random_tsp_problem(10, 100, seed=2)

def main():

    print("PROBLEM")
    print(problem)
    cities = np.random.random((5, 2)) 
    print(cities)

    algorithm = GA(
        pop_size=2,
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

    print("Traveling Time:", np.round(res.F[0], 3))
    print("Function Evaluations:", res.algorithm.evaluator.n_eval)

if __name__ == '__main__':
    main()
