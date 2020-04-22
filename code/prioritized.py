import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost, epea


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        # constraints = [{'agent': 1, 'loc': [(1,4)], 'timestep': 4}]
        # constraints = [{'agent': 1, 'loc': [(1, 2), (1, 1)], 'timestep': 0},]
        constraints = []

        for i in range(self.num_of_agents):  # Find path for each agent
            path = epea(self.my_map, self.starts[i], self.goals[i],
                          i, constraints)
            if path is None:
                raise BaseException('No solutions')
            result.append(path)

            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches
            
            for j in range(self.num_of_agents):
                for idx, loc in enumerate(path):
                    if i < j:
                        # add vertext constraint
                        if idx == len(path) - 1: # if loc is the goal loc, then 
                            # add constraint with a negative time step which refer to all furture time steps > idx
                            constraints.append({'agent': j, 'loc': [loc], 'timestep': -idx, 'positive': 0})
                        constraints.append({'agent': j, 'loc': [loc], 'timestep': idx, 'positive': 0})
                        # add edge constraints
                        if idx == len(path) - 1:
                            continue
                        constraints.append({'agent': j, 'loc': [loc, path[idx+1]], 'timestep': idx + 1, 'positive': 0})
                        constraints.append({'agent': j, 'loc': [path[idx+1], loc], 'timestep': idx + 1, 'positive': 0})


            print("constraints", constraints)

            ##############################

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
