import heapq

class MDD:
    def __init__(self, my_map, start, goal, agent, depth):
        
        self.my_map = my_map
        self.start = start
        self.goal = goal
        self.agent = agent
        self.depth = depth
        self.level = {}

        
        
    def build_mdd(self, my_map):
        bfs_tree = self.build_bfs_tree(my_map, self.start, self.goal, self.depth)
        goal_depth = (self.goal, self.depth)
        
        if not bfs_tree[goal_depth]:
            return None
        
        mdd = {}
        mdd_trim = []
        for node in bfs_tree[goal_depth]:
            mdd_trim.append((node,goal_depth))
            
        while mdd_trim:
            node, child = mdd_trim.pop(0)
            if child not in mdd[node]:
                mdd[node] = child
            for parent in bfs_tree[node]:
                mdd_trim.append((parent, node))
        
        self.level[0] = [self.start]
        for neighbor in mdd.values():
            for node in neighbor:
                self.level[node[1]].append(node[0])
                
        self.mdd = mdd
        
        
    def build_bfs_tree(self, my_map, start, goal, max_depth):
        bfs_open = []
        bfs_open.append((start, 0))
        bfs_tree = {}
        bfs_visited = set()
        while bfs_open:
            location, depth = bfs_open.pop(0)
            
            if (location,depth) in bfs_visited:
                continue
            
            x, y = location[0], location[1]
            possible_location = [(x,y+1),(x,y-1),(x+1,y),(x-1,y),(x,y)]
            
            for loc in possible_location:
                if my_map[loc[0]][loc[1]]:
                    continue
                valid_child = ((loc[0],loc[1]),depth+1)
                
                if valid_child[1] <= max_depth:
                    bfs_tree[valid_child] = (location, depth)
                    if not valid_child in bfs_visited:
                        bfs_visited.add(valid_child)
                        bfs_open.append(valid_child)
            
        return bfs_tree
                        