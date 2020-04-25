import heapq
from collections import defaultdict

class MDD:
    def __init__(self, my_map, start, goal, agent, depth):
        
        self.my_map = my_map
        self.start = start
        self.goal = goal
        self.agent = agent
        self.depth = depth
        self.mdd = self.build_mdd(my_map)

        
        
    def build_mdd(self, my_map):
        bfs_tree = self.build_bfs_tree(my_map, self.start, self.goal, self.depth)
        goal_depth = (self.goal, self.depth)
        
        #if not bfs_tree[goal_depth]:
        #    return None
        
        
        mdd = defaultdict(dict)
        mdd_trim = []
        goal_nodes = bfs_tree[goal_depth]
        if len(goal_nodes)==2:
            mdd_trim.append((goal_nodes,goal_depth))
        else:
            for node in goal_nodes:
                mdd_trim.append((node,goal_depth))

        while mdd_trim:
            node, child = mdd_trim.pop(0)
            if child not in mdd[node]:
                mdd[node] = child

            if len(bfs_tree[node])==2:
                mdd_trim.append((bfs_tree[node], node))
            else:
                for parent in bfs_tree[node]:
                    mdd_trim.append((parent, node))
                
        return mdd
        
        
    def build_bfs_tree(self, my_map, start, goal, max_depth):
        bfs_open = []
        bfs_open.append((start, 0))
        bfs_tree = defaultdict(dict)
        bfs_visited = set()

        while bfs_open:
            location, depth = bfs_open.pop(0)
            
            if (location,depth) in bfs_visited:
                continue
            
            bfs_visited.add((location,depth))
            
            x, y = location[0], location[1]
            possible_location = [(x,y+1),(x,y-1),(x+1,y),(x-1,y),(x,y)]

            for loc in possible_location:
                if my_map[loc[0]][loc[1]]:
                    continue
                valid_child = ((loc[0],loc[1]),depth+1)
                
                if valid_child[1] <= max_depth:
                    bfs_tree[valid_child] = (location, depth)
                    if not valid_child in bfs_visited:
                        #bfs_visited.add(valid_child)
                        bfs_open.append(valid_child)

        return bfs_tree
                        
    def get_start(self):
        return self.start
    
    def get_goal(self):
        return self.goal
    
    def get_depth(self):
        return self.depth
    
    def get_mdd(self):
        return self.mdd