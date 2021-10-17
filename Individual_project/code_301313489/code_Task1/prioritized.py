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
        
        """For Task 1.2
        Test: creating a vertex constraint for an agent 0 on timestep 4 who can't be in location (1,5)"""
        # constraint = dict()
        # constraint['agent'] = 0
        # constraint['loc'] = [(1,5)]
        # constraint['timestep'] = 4
        # # adding the constraint in the constraints => 'lists'
        # constraints.append(constraint)
        
        """For Task 1.3
        Test: adding an edge constraints that prohibits agent 2 from moving from cell (1,1) to cell (1,2) 
        from time step 4 to time step 5"""
        # constraint = dict()
        # constraint['agent'] = 1
        # constraint['loc'] = [(1,2), (1,3)]
        # constraint['timestep'] = 1
        # # adding the constraint in the constraints => 'lists'
        # constraints.append(constraint) 
        
        """For Task 1.4
            Test: prohibiting agent 0 from being at its goal cell (1,5) at time step 10"""
        constraint = dict()
        constraint['agent'] = 0
        constraint['loc'] = [(1,5)]
        constraint['timestep'] = 10
        # adding the constraint in the constraints => 'lists'
        constraints.append(constraint)  
        
        """For Task 1.5
            Test: constriants for collision-free paths with a minimal sum of path lengths"""             
        
        # constraint = dict()
        # constraint['agent'] = 0
        # constraint['loc'] = [(1,2), (1,1)], [(1,2), (1,2)]
        # constraint['timestep'] = 2
        # constraints.append(constraint) 
        
        # constraint = dict()
        # constraint['agent'] = 0
        # constraint['loc'] = [(1,3), (1,2)], [(1,3), (1,3)], [(1,3), (2,3)]
        # constraint['timestep'] = 3
        # constraints.append(constraint) 
        
        # constraint = dict()
        # constraint['agent'] = 0
        # constraint['loc'] = [(1,4), (1,3)], [(1,4), (1,4)]
        # constraint['timestep'] = 4
        # constraints.append(constraint) 
        
        # constraint = dict()
        # constraint['agent'] = 1
        # constraint['loc'] = [(1,2), (1,1)] , [(1,2), (1,2)]
        # constraint['timestep'] = 1
        # constraints.append(constraint) 
        
        # constraint = dict()
        # constraint['agent'] = 1
        # constraint['loc'] = [(1,3), (1,4)] , [(1,3), (1,3)], [(1,3), (1,2)]
        # constraint['timestep'] = 2
        # constraints.append(constraint) 

        # constraint = dict()
        # constraint['agent'] = 1
        # constraint['loc'] = [(2,3), (2,3)]
        # constraint['timestep'] = 3
        # constraints.append(constraint) 
        
        # constraint = dict()
        # constraint['agent'] = 1
        # constraint['loc'] = [(1,3), (1,4)] , [(1,3), (1,3)], [(1,3), (1,2)]
        # constraint['timestep'] = 4
        # constraints.append(constraint) 
               
        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            if path is None:
                raise BaseException('No solutions')
            result.append(path)
            
            print('Agent[{}]'.format(i), 'path = ', path) #To check the path of the agent            

            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches


            ##############################

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
