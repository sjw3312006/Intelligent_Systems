import sys
import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost, move
from copy import deepcopy
from itertools import combinations


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.

    cs = []
    if path1 is None or path2 is None: # base case for both path empty
        return cs
    
    max_len = max(len(path1), len(path2))
    
    # detecting vertex collision
    v1 = []
    for i in range(max_len):
        li, ti = get_location(path1, i)
        v1.append(((li, None), i)) # None since there's no next in vertex constraint
        
    v2 = []
    for k in range(max_len):
        lk, tk = get_location(path2, i)
        v2.append(((lk, None), k)) # None since there's no next in vertex constraint 
        
    # v1 and v2 => changed to set since order does not matter when they're set (not tuple)
    v1 = set(v1)
    v2 = set(v2)
    vv = v1.intersection(v2)
    
    # detecting edge collision
    e1 = []
    for i in range(max_len - 1):
        lci, tci = get_location(path1, i)
        lni, tni = get_location(path1, i + 1)
        e1.append(((lci, lni), i + 1)) #((current, next), time + 1)
        
    e2 = []
    for k in range(max_len - 1):
        lck, tck = get_location(path2, k)
        lnk, tnk = get_location(path2, k + 1)
        e2.append(((lnk, lck), k + 1)) #((next, current), time + 1)
    
    # e1 and e2 => changed to set since order does not matter when they're set (not tuple)    
    e1 = set(e1)
    e2 = set(e2)
    ee = e1.intersection(e2)  
    
    for l, t in vv:
        cs.append((l, t))
        
    for l, t in ee:
        cs.append((l,t))  
    
    # vertex ((l, None), t) U edge((lc, ln), t) = cs => final list    
    return cs 


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.

    collisions = []
    
    path_index = list(range(0, len(paths)))
    path_index_pairs = list(combinations(path_index, 2)) # checking for collision by combination of 2 path
    for i, k in path_index_pairs:
        cs = detect_collision(paths[i], paths[k]) # checking for collision using detect_collision function
        if len(cs) <= 0:
            continue
        for loc, timestep in cs:
            collision = [i, k, loc, timestep]
            if collision not in collisions:
                collisions.append(collision)
                
    return collisions


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep

    agent_0, agent_1, loc, timestep = collision
    
    constraints = []
    
    # checking for vertex constraint
    if loc[1] is None: 
        constraint = dict()
        constraint['agent'] = agent_0
        constraint['loc'] = [loc[0], None]
        constraint['timestep'] = timestep
        constraints.append(constraint)
        
        constraint = dict()
        constraint['agent'] = agent_1
        constraint['loc'] = [loc[0], None]
        constraint['timestep'] = timestep
        constraints.append(constraint)

    # checking for edge constraints
    if loc[1] is not None: 
        constraint = dict()
        constraint['agent'] = agent_0
        constraint['loc'] = [loc[0], loc[1]]
        constraint['timestep'] = timestep
        constraints.append(constraint)
        
        constraint = dict()
        constraint['agent'] = agent_1
        constraint['loc'] = [loc[1], loc[0]]
        constraint['timestep'] = timestep
        constraints.append(constraint)     
    
    return constraints  


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly

    agent_0, agent_1, loc, timestep = collision
    constraints = []
    
    if bool(random.randint(0, 1)): #choose the agent randomly

        # checking for vertex constraint
        if loc[1] is None:
            constraint = dict()
            constraint['agent'] = agent_0
            constraint['loc'] = [loc[0], None]
            constraint['timestep'] = timestep
            constraint['positive'] = False # stop from moving
            constraints.append(constraint)

            # Agent_0 allowed to move, Agent_1 not allowed to move
            constraint = dict()
            constraint['agent'] = agent_0
            constraint['loc'] = [loc[0], None]
            constraint['timestep'] = timestep
            constraint['positive'] = True # allow to move
            constraints.append(constraint)
            
        # checking for edge constraints           
        else: 
            constraint = dict()
            constraint['agent'] = agent_0
            constraint['loc'] = [loc[0], loc[1]]
            constraint['timestep'] = timestep
            constraint['positive'] = False
            constraints.append(constraint)

            constraint = dict()
            constraint['agent'] = agent_0
            constraint['loc'] = [loc[0], loc[1]]
            constraint['timestep'] = timestep
            constraint['positive'] = True
            constraints.append(constraint)
               
    else: # -> if loc[1] is not None:
        # checking for vertex constraint for agent_1   
        if loc[1] is None:      
            constraint = dict()
            constraint['agent'] = agent_1
            constraint['loc'] = [loc[0], None]
            constraint['timestep'] = timestep
            constraint['positive'] = False # agent_1 not allowed, agent_0 allowed
            constraints.append(constraint)

            constraint = dict()
            constraint['agent'] = agent_1
            constraint['loc'] = [loc[0], None]
            constraint['timestep'] = timestep
            constraint['positive'] = True
            constraints.append(constraint)
        
        # checking for edge constraints   
        else:
            constraint = dict()
            constraint['agent'] = agent_1
            constraint['loc'] = [loc[1], loc[0]]
            constraint['timestep'] = timestep
            constraint['positive'] = False
            constraints.append(constraint)
            
            constraint = dict()
            constraint['agent'] = agent_1
            constraint['loc'] = [loc[1], loc[0]]
            constraint['timestep'] = timestep
            constraint['positive'] = True
            constraints.append(constraint)

    return constraints


# Task 4.3: helper function for adjusting the High-Level Search
def paths_violate_constraint(constraint, paths):
    agent_ids = [] 
    
    for agent_id in range(len(paths)):
        if agent_id == constraint['agent']:
            continue
        
        agent_path = paths[agent_id]
        prev_loc   = get_location(agent_path, constraint['timestep'] - 1)
        curr_loc   = get_location(agent_path, constraint['timestep'])
        
        if constraint['loc'][1] == None:
            # vertex violation
            if curr_loc == constraint['loc'][0]:
                agent_ids.append(agent_id)
        else:
            # edge violation
            if prev_loc == constraint['loc'][0] and curr_loc == constraint['loc'][1]: 
                agent_ids.append(agent_id)
    
    return agent_ids # list of agent ids of agents that violate a given positive constraint 


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        for collision in root['collisions']:
            print(collision)

        # Task 3.2: Testing
        for collision in root['collisions']:
            print(standard_splitting(collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit

        while len(self.open_list) > 0:
            P = self.pop_node()
            if len(P['collisions']) <= 0: #checking for collision
                return P['paths']

            # For Task 4.1
            # constraints = standard_splitting(P['collisions'][0])
            
            # For Task 4.3
            constraints = disjoint_splitting(P['collisions'][0])
            for constraint in constraints:
                ai = constraint['agent'] 
                
                Q = deepcopy(P)
                Q['constraints'].append(constraint)
                path = a_star(self.my_map, self.starts[ai], self.goals[ai],
                              self.heuristics[ai], ai, Q['constraints'])
                
                if len(path) > 0:
                    Q['paths'][ai] = path
                
                    ######################################
                    # Task 4.3: computing a list of agent ids of agents that violate a given positive constraint
                    if constraint['positive']:
                        vids = paths_violate_constraint(constraint, Q['paths'])
                        for vid in vids: # not a,b and rest of c, d, e, f,........agent
                            
                            C = {'agent': vid,
                                 'loc': constraint['loc'],
                                 'timestep': constraint['timestep'],
                                 'positive': False} # a or b will go to that location, thus c = false
                            Q['constraints'].append(C) # new constraint add
                            
                            path = a_star(self.my_map, self.starts[vid], self.goals[vid], 
                                          self.heuristics[vid], vid, Q['constraints']) # agents' path collision from c, and a,b
                            if len(path) > 0:
                                Q['paths'][vid] = path   # path update for c (becomes new path of c)                     
                    ######################################
                    
                    Q['cost'] = get_sum_of_cost(Q['paths'])
                    Q['collisions'] = detect_collisions(Q['paths'])
                    self.push_node(Q)

        self.print_results(root)
        return root['paths']


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
