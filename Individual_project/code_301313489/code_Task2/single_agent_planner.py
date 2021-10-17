import heapq

def move(loc, dir):
    directions = [(0,0), (0, -1), (1, 0), (0, 1), (-1, 0)]
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


def build_constraint_table(constraints, agent):
    constraint_table = []
    
    for constraint in constraints:
        agent = constraint['agent']
        loc = constraint['loc']
        timestep = constraint['timestep']

        curr_loc = loc[0]
        next_loc = None
        if len(loc) >= 2:
            next_loc = loc[1]
            
        constraint_table.append([agent, timestep, curr_loc, next_loc])
        
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


def is_constrained(curr_loc, next_loc, next_time, constraint_table):  
    for _agent, _timestep, _curr_loc, _next_loc in constraint_table:
        if curr_loc == _curr_loc and next_loc == _next_loc and next_time == _timestep:
            return True
    return False
    

def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']

# Task 2.1: from getting the constrained max timestep from Task 1.4
def get_max_timestep(constraint_table):
    max_timestep = -1
    for agent, timestep, curr_loc, next_loc in constraint_table:
        if timestep > max_timestep:
            max_timestep = timestep
    return max_timestep


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """
    constraint_table = build_constraint_table(constraints, agent)
    # Task 2.1
    max_timestep = get_max_timestep(constraint_table)   
    
    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': 0}
    push_node(open_list, root)
    closed_list[(root['loc'], root['timestep'])] = root
    
    while len(open_list) > 0:
        curr = pop_node(open_list)
        
        # Task 2.1 : e.g. the goal(cell, timestep) of the agent_1 becomes the environment for agent_0
        if curr['loc'] == goal_loc and curr['timestep'] >= max_timestep:
            return get_path(curr)
        
        for dir in range(5):
            child_loc = move(curr['loc'], dir)
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            
            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'parent': curr,
                    'timestep': curr['timestep'] + 1}
            
            # vertex constraint
            curr_loc = child['loc']
            next_loc = None
            time_step = child['timestep'] 
            if is_constrained(curr_loc, next_loc, time_step, constraint_table):
                continue              

            # edge constraint
            curr_loc = curr['loc']
            next_loc = child['loc']
            time_step = child['timestep']             
            if is_constrained(curr_loc, next_loc, time_step, constraint_table):
                continue  
                        
            if (child['loc']) in closed_list:
                existing_node = closed_list[(child['loc'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child)

    return None  # Failed to find solutions
