import heapq

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values

def manhattan_distance(my_map, goal):
    h_values = dict()
    for i in range(len(my_map)):
        for j in range(len(my_map[0])):
            h_values[(i, j)] = abs(i - goal[0]) + abs(i - goal[1])
    return h_values


def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.
    constraint_table = dict()
    for constraint in constraints:
        if constraint['agent'] == agent:
            if constraint['timestep'] in constraint_table: # existed for timestep
                constraint_table[constraint['timestep']].append(constraint)
            else: # new list
                constraint_table[constraint['timestep']] = [constraint]
        else: # other agents
            if constraint['positive']:
                if constraint['timestep'] in constraint_table: # existed for timestep
                    constraint_table[constraint['timestep']].append(constraint)
                else: # new list
                    constraint_table[constraint['timestep']] = [constraint]

    return constraint_table



def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path

def check_constraints(curr_loc, next_loc, constraints):
    for constraint in constraints:
        if len(constraint['loc']) == 1: # Vectex Constraint
            if next_loc in constraint['loc']:
                return True
        elif len(constraint['loc']) == 2: # Edge Constraint
            if curr_loc == constraint['loc'][0] and next_loc == constraint['loc'][1]:
                return True 
        else:
            raise BaseException('Unexpected constraint Loc length ' + str(len(constraint['loc'])) + ', should be either 1 (vertex) or 2 (edge)')

def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.
    constrained = False
    negative_timesteps = [k for k in constraint_table.keys() if k < 0]
    for neg_timestep in negative_timesteps:
        if next_time > -neg_timestep:
            constrained = check_constraints(curr_loc, next_loc, constraint_table[neg_timestep])
    if next_time in constraint_table:
        constrained = check_constraints(curr_loc, next_loc, constraint_table[next_time])

    return constrained


def push_node(open_list, node):
    tmp = list(map(lambda _tuple: _tuple[0:3], open_list))
    # item = (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node)
    if (node['g_val'] + node['h_val'], node['h_val'], node['loc']) not in tmp:
        heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))

def push_node2(open_list, node):
    tmp = list(map(lambda _tuple: _tuple[0:4], open_list))
    # item = (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node)
    if (node['F_val'], node['g_val'] + node['h_val'], node['h_val'], node['loc']) not in tmp:
        heapq.heappush(open_list, (node['F_val'], node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr

def pop_node2(open_list):
    _, _, _, _, curr = heapq.heappop(open_list)
    return curr

def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']

def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    open_list = []

    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]
    constraint_table = build_constraint_table(constraints, agent)
    # print(constraint_table)
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': 0}
    push_node(open_list, root)
    closed_list[(root['loc'], earliest_goal_timestep)] = root
    max_iteration = len(my_map) * len(my_map[0]) * 4
    while len(open_list) > 0:
        max_iteration = max_iteration - 1
        if max_iteration < 0:
            break
        curr = pop_node(open_list)
        # print("Expanding ", curr['loc'])

        next_time = curr['timestep'] + 1
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        
        if curr['loc'] == goal_loc:
            if not constraint_table or max(constraint_table) < next_time: #empty dict or max time step in constraint_table is less than next_time
                return get_path(curr)

        for dir in range(5):
            if dir == 4:
                child_loc = curr['loc']
            else:
                child_loc = move(curr['loc'], dir)
            
            if min(child_loc[0], child_loc[1]) < 0 or child_loc[0] >= len(my_map) or child_loc[1] >= len(my_map[0]) or my_map[child_loc[0]][child_loc[1]]:
                continue
            if is_constrained(curr['loc'], child_loc, next_time, constraint_table):
                continue

            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'parent': curr,
                    'timestep': next_time}
            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child)
                
    return None  # Failed to find

def epea(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """
    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    # h_value = abs(start_loc[0] - goal_loc[0]) + abs(start_loc[1] - goal_loc[1])
    constraint_table = build_constraint_table(constraints, agent)
    # print(constraint_table)
    root = {'loc': start_loc, 'F_val': h_values[start_loc], 'g_val': 0, 'h_val': h_values[start_loc], 'parent': None, 'timestep': 0}
    push_node2(open_list, root)
    closed_list[(root['loc'], earliest_goal_timestep)] = root
    max_iteration = len(my_map) * len(my_map[0]) * 4
    # FF = root['F_val']
    while len(open_list) > 0:
        max_iteration = max_iteration - 1
        if max_iteration < 0:
            break
        curr = pop_node2(open_list)
        # print("Expanding ", curr['loc'])
        # if FF != curr['g_val'] + curr['h_val']:
        #     FF = curr['g_val'] + curr['h_val']
        #     print("NEW FF:", curr)
        next_time = curr['timestep'] + 1
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        
        if curr['loc'] == goal_loc:
            if not constraint_table or max(constraint_table) < next_time: #empty dict or max time step in constraint_table is less than next_time
                # print("result: ", curr)
                return get_path(curr)
        # print("curr:", curr)
        N, F_next = OSF(my_map, curr, goal_loc, h_values)
        # print('N:', N)
        # print('F_next:', F_next)
        for child_loc in N:
            if is_constrained(curr['loc'], child_loc, next_time, constraint_table):
                # print("constrainted     ", child_loc, next_time)
                continue

            h_value = h_values[child_loc]
            # abs(child_loc[0] - goal_loc[0]) + abs(child_loc[1] - goal_loc[1])
            child = {'loc': child_loc,
                    'F_val': curr['g_val'] + 1 + h_value,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_value,
                    'parent': curr,
                    'timestep': next_time}
            push_node2(open_list, child)

        curr['F_val'] = F_next
        # if not is_constrained(curr['loc'], curr['loc'], next_time, constraint_table):
        # print("open_list:", open_list)
        # print("curr", curr)
        push_node2(open_list, curr)
        # if curr in open_list:
        #     continue
        
        # print("--------------------------------------------------------------------------------:")

        # print("N:", N)
        # print("f_next:", f_next)
        # for dir in range(5):
        #     if dir == 4:
        #         child_loc = curr['loc']
        #     else:
        #         child_loc = move(curr['loc'], dir)
            
        #     if min(child_loc[0], child_loc[1]) < 0 or child_loc[0] >= len(my_map) or child_loc[1] >= len(my_map[0]) or my_map[child_loc[0]][child_loc[1]]:
        #         continue
        #     if is_constrained(curr['loc'], child_loc, next_time, constraint_table):
        #         continue

        #     child = {'loc': child_loc,
        #             'g_val': curr['g_val'] + 1,
        #             'h_val': h_values[child_loc],
        #             'parent': curr,
        #             'timestep': next_time}
        #     if (child['loc'], child['timestep']) in closed_list:
        #         existing_node = closed_list[(child['loc'], child['timestep'])]
        #         if compare_nodes(child, existing_node):
        #             closed_list[(child['loc'], child['timestep'])] = child
        #             push_node(open_list, child)
        #     else:
        #         closed_list[(child['loc'], child['timestep'])] = child
        #         push_node(open_list, child)
                
    return None  # Failed to find


def OSF(my_map, curr, goal_loc, h_values):
    F_val = curr['F_val']
    F_next = float('inf')
    N = []
    # PDB = [1, 2, float('inf')]
    for dir in range(5):
        if dir == 4:
            child_loc = curr['loc']
        else:
            child_loc = move(curr['loc'], dir)

        if min(child_loc[0], child_loc[1]) < 0 or child_loc[0] >= len(my_map) or child_loc[1] >= len(my_map[0]) or my_map[child_loc[0]][child_loc[1]]:
            continue
        
        h_value = h_values[child_loc]
        # abs(child_loc[0] - goal_loc[0]) + abs(child_loc[1] - goal_loc[1])

        # child = {'loc': child_loc,
        #         'g_val': curr['g_val'] + 1,
        #         'h_val': h_value,
        #         'parent': curr,
        #         'timestep': next_time}
        f_c = curr['g_val'] + 1 + h_value
        # delta_f_c = f_c - f_n

        if f_c > F_val:
            F_next = min(f_c, F_next)
            continue
        elif f_c == F_val:
            N.append(child_loc)
    
    return N, F_next

[
    {'loc': (3, 1), 'g_val': 1, 'h_val': 4, 'parent': {'loc': (2, 1), 'g_val': 0, 'h_val': 5, 'parent': None, 'timestep': 0}, 'timestep': 1}, 
    {'loc': (2, 2), 'g_val': 1, 'h_val': 4, 'parent': {'loc': (2, 1), 'g_val': 0, 'h_val': 5, 'parent': None, 'timestep': 0}, 'timestep': 1}
    ]


OPEN: [
    (5, 4, (2, 2), {'loc': (2, 2), 'g_val': 1, 'h_val': 4, 'parent': {'loc': (2, 1), 'g_val': 0, 'h_val': 6, 'parent': None, 'timestep': 0}, 'timestep': 1}), 
    (5, 4, (3, 1), {'loc': (3, 1), 'g_val': 1, 'h_val': 4, 'parent': {'loc': (2, 1), 'g_val': 0, 'h_val': 6, 'parent': None, 'timestep': 0}, 'timestep': 1}), 
    (6, 6, (2, 1), {'loc': (2, 1), 'g_val': 0, 'h_val': 6, 'parent': None, 'timestep': 0})]


