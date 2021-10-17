import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


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
        constraints = []
        
        ##############################
        # Task 2.4 : Addressing Failure
        #reordering agent priority using heuristic costs
        agent_max_cost = []
        for agent in range(self.num_of_agents):
            max_cost = -1
            for loc, cost in self.heuristics[agent].items():
                print(loc, cost)
                if cost > max_cost:
                    max_cost = cost
            agent_max_cost.append([cost, agent])
        agent_max_cost.sort()
        agent_max_cost.reverse()
        
        for _, i in agent_max_cost:
        ##############################

        # original
        # for i in range(self.num_of_agents):  # Find path for each agent
        
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            if path is None:
                raise BaseException('No solutions')
            result.append(path)
            
            print('Agent[{}]'.format(i), 'path = ', path)
            
            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches

            # Task 2.1 : adding vertex constraints
            # for node in path:
            #     constraint = dict()
            #     constraint['agent'] = i
            #     constraint['loc'] = [node[0], None]
            #     constraint['timestep'] = node[1]
            #     constraints.append(constraint)
            
            # build constraint from a path
            for p in range(len(path)):
                curr_loc = path[p]
                next_loc = None
                curr_timestep = p
                
                # vertex constraint
                constraint = dict()
                constraint['agent'] = i
                constraint['loc'] = [curr_loc, next_loc]
                constraint['timestep'] = curr_timestep
                constraints.append(constraint)
                
                # edge constraint except start cell
                if p > 0:
                    prev_loc = path[p - 1]
                    next_timestep = p
                    
                    constraint = dict()
                    constraint['agent'] = i
                    constraint['loc'] = [curr_loc, prev_loc]
                    constraint['timestep'] = next_timestep
                    constraints.append(constraint)
                    
            ##############################
            
            # Task 2.3: additional constraints
            # agent_0 reached its goal cell at timestep 3,
            # but, agent_1 will reach to the goal cell at timestep 7
            # then, agent_0 will be wait 
            
            # but, we don't know the value 7
            # so, just estimate this value with the prior knowledge
            # for example, # of a valid cell
            
            ##############################
                        
            last_loc = path[-1]
            last_timestep = len(path) -1
            
            max_cells = 0
            
            for x in range(len(self.my_map)):
                for y in range(len(self.my_map[0])):
                    if not self.my_map[x][y]:
                        max_cells += 1
                        
            for timestep in range(last_timestep + 1, max_cells):
                constraint = dict()
                constraint['agent'] = i
                constraint['loc'] = [last_loc, None]
                constraint['timestep'] = timestep
                constraints.append(constraint)               

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
