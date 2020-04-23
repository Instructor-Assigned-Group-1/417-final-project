import time as timer
from ict import ICT
from mdd import MDD
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class ICTSSolver(object):
    # Increasing Cost Tree Search
    # Contains high level functions for ICTS implementation

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)
        
        self.ict = self.generate_ict()

        self.CPU_time  = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    
    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""
        result = []
        expanded_nodes = 0
        mdd_list = {}
        open_list = self.ict.get_open_list()
        
        start_time = timer.time()
        
        while open_list:
            node_to_expand = open_list[0]
            node_cost = node_to_expand['cost']
            
            agent_paths = self.find_agent_paths(node_cost, mdd_list)
            
            if agent_paths:
                break
                return agent_paths
            else:
                self.ict.expand_node(node_to_expand)
            
            self.ict.pop_node()
        

        self.CPU_time = timer.time() - start_time

        if agent_paths:
            print("\n Found a solution! \n")
            print("CPU time (s):    {:.2f}".format(self.CPU_time))
            return agent_paths
        else:
            print("\n Could not find a solution \n")
            print("CPU time (s):    {:.2f}".format(self.CPU_time))
            return []
    
    def find_agent_paths(self, path_costs):
        mdds_generated = []
        
        for cost in path_costs:
            cost_ind = path_costs.index(cost)
            mdd = MDD(self.my_map, self.starts[cost_ind], self.goals[cost_ind], cost)
            mdds_generated.append(mdd)
            
        return self.find_solution_joint_mdd(mdds_generated)
                
    
    def generate_ict(self):
        optimal_costs = []
        optimal_paths = []
        
        for agent in range(self.num_of_agents):
            optimal_paths.append(a_star(self.my_map, self.starts[agent], self.goals[agent], 
                                        self.heuristics[agent], agent, []))
        
        for path in optimal_paths:
            if not path:
                break
            optimal_costs.append(max(len(path)))
        
        ict = ICT(self.my_map, self.starts, self.goals, optimal_costs)
        
