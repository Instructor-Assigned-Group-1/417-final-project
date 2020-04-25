import heapq

class ICT:
    def __init__(self, my_map, starts, goals, init_cost):
        
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)
        
        self.init_cost = init_cost
        self.root = {'cost':init_cost,'children':[]}
        
        self.open_list = []
        self.open_list.append(self.root)
        self.closed_list = set([tuple(init_cost)])
        
        
    def push_node(self, node):
        self.open_list.append(node)


    def pop_node(self):
        return self.open_list.pop(0)
    
    def add_open_node(self, node):
        # Check if node in closed list
        node_cost = tuple(node['cost'])
        if not (node_cost in self.closed_list):
            self.open_list.append(node)
            self.closed_list.add(node_cost)
            
    def expand_node(self, node):
        for cost in node['cost']:
            cost_list = node['cost']
            cost_list[node['cost'].index(cost)] = cost + 1
            
            node_child = {'cost':cost_list,'children':[]}
            node['children'].append(node_child)
            self.add_open_node(node_child)
            
    def get_open_list(self):
        return self.open_list
            
