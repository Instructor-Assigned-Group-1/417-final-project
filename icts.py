import time as timer
import itertools
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
        
        self.CPU_time  = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))
        
        self.ict = self.generate_ict()
        self.cost_limit = self.determine_cost_limit()

    
    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""
        open_list = self.ict.get_open_list()
        
        start_time = timer.time()
        
        while open_list:
            node_to_expand = open_list[0]
            node_cost = node_to_expand['cost']
            
            if sum(node_cost) > self.cost_limit:
                self.ict.pop_node()
                continue
            
            agent_paths = self.find_agent_paths(node_cost)
            
            if agent_paths:
                break
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
            mdd = MDD(self.my_map, self.starts[cost_ind], self.goals[cost_ind], cost_ind, cost)
            mdds_generated.append(mdd)
        
        return self.find_mdd_solution(mdds_generated)
                
    
    def generate_ict(self):
        optimal_costs = []
        optimal_paths = []
        
        for agent in range(self.num_of_agents):
            optimal_paths.append(a_star(self.my_map, self.starts[agent], self.goals[agent], 
                                        self.heuristics[agent], agent, []))
            
        for path in optimal_paths:
            if not path:
                break
            optimal_costs.append(len(path))
        
        return ICT(self.my_map, self.starts, self.goals, optimal_costs)
        
    def find_mdd_solution(self, mdds_generated):
        mdd_start = []
        mdd_depth = []
        for mdd in mdds_generated:
            mdd_start.append(mdd.get_start())
            mdd_depth.append(mdd.get_depth())

        visited = set()
        mdd_start_nodes = (tuple(mdd_start), 0)
        
        solution, visited = self.mdd_dfs(mdds_generated, mdd_start_nodes, visited, max(mdd_depth))
        return False
    
    def mdd_dfs(self, mdds_generated, nodes, visited, max_depth):
        is_goal = False
        if (nodes[0] in visited) or (nodes[1] > max_depth):
            return [], visited
        
        visited.add(nodes)
        for mdd in mdds_generated:
            node = nodes[0][mdds_generated.index(mdd)]
            if (mdd.get_depth() <= nodes[1]) and (mdd.get_goal()==node):
                is_goal = True
                
        if is_goal:
            return [nodes], visited
        
        joint_children = self.get_joint_children(mdds_generated, nodes)
        
        solution_path = [nodes]
        for child_loc in joint_children:
            child = (child_loc, nodes[1]+1)
            solution, visited = self.mdd_dfs(mdds_generated, child, visited, max_depth)
            if solution:
                solution_path.extend(solution)
                return solution_path, visited
        return [], visited
        
    def get_joint_children(self, mdds, nodes):
        children = []

        for node in nodes[0]:
            mdd_obj = mdds[nodes[0].index(node)]
            mdd = mdd_obj.get_mdd()

            if (mdd_obj.goal == node) and (mdd_obj.depth == nodes[1]):
                children.append(mdd_obj.goal)
                continue

            node_children = [mdd[(node, nodes[1])]]
            node_children_location = [child[0] for child in node_children]
            children.append(node_children_location)
        
        joint_children = list(itertools.product(*children))

        return joint_children
            
    def determine_cost_limit(self):
        open_spaces = 0
        cost_limit = 0
        
        for row in self.my_map:
            for space in row:
                if not space:
                    open_spaces = open_spaces+1
                    
        cost_limit = sum([(agent+1)*open_spaces for agent in range(self.num_of_agents)])
        
        return cost_limit