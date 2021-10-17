# -*- coding: utf-8 -*-
"""
Created on Mon Apr 20 2021

@author: jeongwoon_suh
"""

import numpy as np
from random import seed
import random
from copy import deepcopy
import math
import heapq
import functools
import time
from sys import getsizeof
from OSF import cal_osf
import pickle
from heapq import heappop, heappush
from collections import defaultdict

seed(1)

# Reference: Modified on basis of aima-python
class Node:
    def __init__(self, state, parent=None, action=None, path_cost=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost
        self.depth = 0
        self.of = 0
        if parent:
            self.depth = parent.depth + 1

    def __repr__(self):
        return "<Node {}>".format(self.state)
    
    def expand(self, problem):
        return [self.child_node(problem, action)
                for action in problem.actions(self.state)]

    def child_node(self, problem, action):
        next_state = problem.result(self.state, action)
        next_node = Node(next_state, self, action, problem.path_cost(self.path_cost))
        return next_node

    def solution(self):
        return [node.action for node in self.path()[1:]]

    def path(self):
        node, path_back = self, []
        while node:
            path_back.append(node)
            node = node.parent
        return list(reversed(path_back))
    
    def __eq__(self, other):
        return isinstance(other, Node) and self.state == other.state
    
    def __hash__(self):
        return hash(self.state)
    
    def __lt__(self, node):
        return self.state < node.state

# Reference: Modified on basis of aima-python
def memoize(fn, slot=None):
    def memoized_fn(obj, *args):
        if hasattr(obj, slot):
            return getattr(obj, slot)
        else:
            val = fn(obj, *args)
            setattr(obj, slot, val)
            return val
    return memoized_fn

# Reference: Modified on basis of aima-python()
class PriorityQueue:

    def __init__(self, f):
        self.heap = []
        self.f = f
        self.hash_table = {}

    def append(self, item):
        hash_key = hash(item.state)
        if hash_key in self.hash_table:
            return
        heapq.heappush(self.heap, (self.f(item), item))
        self.hash_table[hash_key] = self.f(item)

    def extend(self, items):
        for item in items:
            self.append(item)

    def pop(self):
        if self.heap:
            item = heapq.heappop(self.heap)[1]
            hash_key = hash(item.state)
            del self.hash_table[hash_key]
            return item
        else:
            raise Exception('Empty PQ.')

    def __len__(self):
        return len(self.heap)

    def __contains__(self, key):
        if key.state in self.hash_table:
            return True
        return False
        # return any([item.state == key.state for _, item in self.heap])

    def __getitem__(self, key):
        # for value, item in self.heap:
            # if item == key:
                # return value
        hash_key = hash(key.state)
        return self.hash_table[hash_key]
        raise KeyError(str(key) + " not in PQ")

    def __delitem__(self, key):
        try:
            del self.heap[[item == key for _, item in self.heap].index(True)]
        except ValueError:
            raise KeyError(str(key) + " not in PQ")
        heapq.heapify(self.heap)

class Puzzle():
    def __init__(self,size):
        n = size + 1
        self.init_size = size if isinstance(size, list) else size
        self.size = size

        self.n = int(math.sqrt(self.init_size+1))
        self.goal =  list(range(1,n))
        self.goal.append(0)
        
        init = deepcopy(self.goal)
        prev = None
        for i in range(150):
            random_choice = random.choice(self.actions(init))
            init = self.result(init, random_choice)

        self.init = init
        self.init = tuple(self.init)
        self.goal = tuple(self.goal)
        return

    def load_pattern_database(self):
        file = open("database_A", "rb")
        self.__pattern_database_A = pickle.load(file)
        file.close()

        file = open("database_B", "rb")
        self.__pattern_database_B = pickle.load(file)
        file.close()

        file = open("database_C", "rb")
        self.__pattern_database_C = pickle.load(file)
        file.close()

        self.__is_loaded = True # need to revise
    
    def get_heuristic_value(self, node):
        cur_state = node.state
        A = [a if a in [1, 2, 3, 5, 6] else 0 for a in cur_state]
        B = [b if b in [4, 7, 8, 11, 12] else 0 for b in cur_state]
        C = [c if c in [9, 10, 13, 14, 15] else 0 for c in cur_state]
        ret = 0
        ret += self.__pattern_database_A[str(A)]
        ret += self.__pattern_database_B[str(B)]
        ret += self.__pattern_database_C[str(C)]
        return ret
    
    def goal_test(self,state):
        return state == self.goal
    
    def find_blank_square(self, state):
        return state.index(0)
    
    def actions(self, state):        
        n = self.n
        possible_actions = ['UP', 'DOWN', 'LEFT', 'RIGHT']
        index_blank_square = self.find_blank_square(state)
        if index_blank_square % n == 0:
            possible_actions.remove('LEFT')
        if index_blank_square < n:
            possible_actions.remove('UP')
        if index_blank_square % n == n-1:
            possible_actions.remove('RIGHT')
        if index_blank_square > n*n-n-1:
            possible_actions.remove('DOWN')
        return possible_actions
    
    def disp(self,state=None):
        if state == None:
            state = self.init
        n = self.n
        print("Puzzle:")
        for i in range(n):
            for j in range(n):
                if state[n*i+j] == 0:
                    print(" ", end ="\t") 
                else:
                    print(state[n*i+j], end ="\t")
            print()
        print()
    
    def result(self, state, action):

        # blank is the index of the blank square
        blank = self.find_blank_square(state)
        new_state = list(state)
        n = self.n
        delta = {'UP': -n, 'DOWN': n, 'LEFT': -1, 'RIGHT': 1}
        neighbor = blank + delta[action]
        new_state[blank], new_state[neighbor] = new_state[neighbor], new_state[blank]
        return tuple(new_state)
    
    def solvable(self, state = None):
        if state == None:
            state = self.init
        inversion = 0
        for i in range(len(state)-1):
            for j in range(i + 1, len(state)):
                if (state[i] > state[j] and state[i] != 0 and state[j] != 0):
                    inversion += 1

        if self.n % 2 == 1:
            return inversion % 2 == 0
        else:
            index = self.find_blank_square(state)
            index = self.init_size - index
            index = (index + 1) // self.n + 1
            # blank is on an even row counting from the bottom and number of inversions is odd.
            if index % 2 == 0 and inversion % 2 == 1:
                return True
            # blank is on an odd row counting from the bottom and number of inversions is even.
            elif index % 2 == 1 and inversion % 2 == 0:
                return True
            else:
                return 
            
    def path_cost(self, c):
        return c + 1
    
    def h_manhattan(self, node):
        print("Error: Should be using PBD heuristic")
        cur_state = node.state
        end_state = self.goal
        dist = 0
        n = self.n
        for i in range(n):
            for j in range(n):
                if cur_state[i*n+j] == 0:
                    continue
                if cur_state[i*n+j] == end_state[i*n+j]:
                    continue
                goal = cur_state[i*n+j]
                y = (goal - 1) // n
                x = goal - n * y - 1
                dist += (abs(y - i) + abs(x - j))
        return dist
       
def PEASTAR(Puzzle, f):
    total_generated = 0
    total_expanded = 0
    max_mem = 0
    
    # Create the f-func that memorize the f values
    f = memoize(f, 'f')
    opened = PriorityQueue(f)
    closed = set()

    node = Node(Puzzle.init)
    # Initialize start F_val as the value of the root node
    F_val = f(node)
    opened.append(node)
    
    while opened:
        max_mem = max(max_mem, getsizeof(opened.heap))
        
        # Get n with lowest F(n) from OPEN
        node = opened.pop()
        F_val = node.f
        
        # if n is goal then exit
        if Puzzle.goal_test(node.state):
            total_expanded += 1 
            return node, node.depth, total_generated, total_expanded, max_mem
        
        # Choosing the node for expansion
        F_next = math.inf
        for child in node.expand(Puzzle):
            total_generated += 1 
            # Calculate f-value of each child
            f(child)
            # Find the min val greater than current node
            if child.f != F_val:
                if child.f > F_val:
                    F_next = min(F_next,child.f)
                continue
            # Expand nodes only with f = F
            # Duplicate detection
            child.f = f(child)
            if child.state not in closed and child not in opened:
                opened.append(child)
            elif child in opened:
                if f(child) < opened[child]:
                    del opened[child]
                    opened.append(child)
        total_expanded += 1
        
        if F_next == math.inf:
            # visited and it is not a goal state, put it into close
            closed.add(node.state)
        else:
            node.f = F_next
            opened.append(node)
    return None, 0, 0, 0, 0

def EPEASTAR(Puzzle, f):
    osf_table = cal_osf(Puzzle.size + 1, Puzzle.goal)
    total_generated = 0
    total_expanded = 0
    max_mem = 0
    n = Puzzle.n
    # Create the f-func that memorize the f values
    f = memoize(f, 'f')
    opened = PriorityQueue(f)
    closed = set()

    node = Node(Puzzle.init)
    # Initialize start F_val as the value of the root node
    node.of = f(node)
    F_val = node.of 
    opened.append(node)
    
    while opened:
        max_mem = max(max_mem, getsizeof(opened.heap))
        
        # Get n with lowest F(n) from OPEN
        node = opened.pop()
        F_val = node.f
        
        # if n is goal then exit
        if Puzzle.goal_test(node.state):
            total_expanded += 1 
            return node, node.depth, total_generated, total_expanded, max_mem
        
        blank = node.state.index(0)
        move = [-n, n, -1, 1]
        actions = ['UP', 'DOWN', 'LEFT', 'RIGHT']
        osf = {}
        
        for i in range(len(move)):
            pos = blank + move[i]
            # Checking if it valid entry in OSF
            if pos < 0 or pos >= n*n:
                continue
            val = node.state[pos] - 1
            delta_f = osf_table[blank][val][i]
            osf[actions[i]] = delta_f
        
        F_next = math.inf
        delta_f = node.f - node.of
        
        for i in osf:
            if osf[i] == delta_f:
                child = node.child_node(Puzzle, i)
                child.f = f(child)
                child.of = child.f
                total_generated += 1 
                if child.state not in closed and child not in opened:
                    opened.append(child)
                elif child in opened:
                    if f(child) < opened[child]:
                        del opened[child]
                        opened.append(child)
            elif osf[i] > delta_f:
                F_next = min(F_next, osf[i])
        
        total_expanded += 1
        if F_next == math.inf:
            # visited and it is not a goal state, put it into close
            closed.add(node.state)
        else:
            node.f = node.f + F_next
            opened.append(node)
    return None, 0, 0, 0, 0
            
def ASTAR(Puzzle, f):    
    total_generated = 0
    total_expanded = 0
    max_mem = 0
    
    init_state = Puzzle.init
        
    f = memoize(f, 'f')
    node = Node(Puzzle.init)
    opened = PriorityQueue(f)
    opened.append(node)
    closed = set()
    
    while opened:
        max_mem = max(max_mem, getsizeof(opened.heap))
        node = opened.pop()

        if Puzzle.goal_test(node.state):
            total_expanded += 1
            return node, node.depth, total_generated, total_expanded, max_mem

        closed.add(node.state)
        for child in node.expand(Puzzle):
            total_generated += 1
            if child.state not in closed and child not in opened:
                opened.append(child)
            elif child in opened:
                if f(child) < opened[child]:
                    del opened[child]
                    opened.append(child)
        total_expanded += 1
    return None, 0, 0, 0, 0, 0

def IDASTAR(Puzzle, f):
    Puzzle.load_pattern_database()
    node = Node(Puzzle.init)
    bound = node.path_cost
    path = [node]
    problem = Puzzle
    f = memoize(f, 'f')
    
    max_mem = 0
    max_mem = max(max_mem, getsizeof(path))
    total_generated = 0
    total_expanded = 0
    while True:
        t, max_mem, total_generated, total_expanded = IDASTARSearch(problem, path, 0, bound, f, max_mem, total_generated , total_expanded)
        if t == -1:
            return path, bound, max_mem, total_generated, total_expanded
        if t > math.inf:
            return None
        bound = t
        
def IDASTARSearch(problem, path, g, bound, f, max_mem, total_generated , total_expanded):

    max_mem = 0
    max_mem = max(max_mem, getsizeof(path))
    node = path[-1]
    f(node)
    
    if node.f > bound:
        return node.f, max_mem, total_generated, total_expanded

    if problem.goal_test(node.state):
        path.append(node)
        total_expanded += 1
        return -1, max_mem, total_generated, total_expanded
    
    Min = math.inf
    for child in node.expand(problem):
        total_generated += 1
        if child not in path:
            path.append(child)
            t, max_mem, total_generated, total_expanded = IDASTARSearch(problem, 
                    path, g + 1, bound, f,
                    max_mem, total_generated , total_expanded)
            if t == -1:
                return -1, max_mem, total_generated, total_expanded
            if t < Min:
                Min = t
            path.pop()
    total_expanded += 1
    return Min, max_mem, total_generated, total_expanded

# From Additive pattern database heuristic, using "5-5-5 Statically-Partitioned Additive Database Heuristic" of A* algorithm
def experiment_pdb(size,n):
    i = 0
    while i < n:
        PuzzleInstance = Puzzle(size)
        Solvable = PuzzleInstance.solvable()
        if Solvable:
            i += 1
            h = memoize(PuzzleInstance.get_heuristic_value, 'h')
            PuzzleInstance.disp()
            PuzzleInstance.load_pattern_database()

            print("PuzzleInstance init: {}\n".format(PuzzleInstance.init))

            
            start = time.perf_counter()
            Soln, Cost, Generated, Expanded, max_mem = ASTAR(PuzzleInstance, lambda n: n.path_cost + h(n))
            elapsed = time.perf_counter() - start
            if Soln == None:
                return("Error in Implementation")
            print("\nPDB A*")
            print("Solving instance {}\
                    \n\tSolution:{}\
                    \n\tTime: {} secs\
                    \n\tMemory: {} bytes\
                    \n\tcost = {}\
                    \n\tnumber of generated nodes = {}\
                    \n\tnumber of expanded nodes = {}".format(PuzzleInstance.init, Soln, elapsed, max_mem, Cost, Generated, Expanded))
            ACOST = Cost
            
            start = time.perf_counter()
            Soln, Cost, Generated, Expanded, max_mem = PEASTAR(PuzzleInstance, lambda n: n.path_cost + h(n))
            elapsed = time.perf_counter() - start
            if Soln == None:
                return("Error in Implementation")
            print("\nPDB PEA*")
            print("Solving instance {}\
                    \n\tSolution:{}\
                    \n\tTime: {} secs\
                    \n\tMemory: {} bytes\
                    \n\tcost = {}\
                    \n\tnumber of generated nodes = {}\
                    \n\tnumber of expanded nodes = {}".format(PuzzleInstance.init, Soln, elapsed, max_mem, Cost, Generated, Expanded))
            EPEACOST = Cost
            
experiment_pdb(15,1)

# execute : python3 PDB.py -> to generate pattern data base first
# next execute: python3 Puzzle.py -> to generate puzzle using a* search

# For the test purpose of PDB heuristics (partitioning)

# if __name__ == "__main__":
    #init_input = list(map(int, "0 12 9 13    15 11 10 14    3 7 2 5    4 8 6 1".split())) # 80 step impossible
    #init_input = list(map(int, "0 12 10 13    15 11 14 9    3 7 2 5    4 8 6 1".split())) # 80 step impossible
    #init_input = list(map(int, "0 15 8 3    12 11 7 4    14 10 6 5    9 13 2 1".split())) # 70 step impossible
    #init_input = list(map(int, "6 5 9 13    2 1 10 14    3 7 0 15    4 8 12 11".split())) # 72 step impossible
    # init_input = list(map(int, "15 14 1 6    9 11 4 12    0 10 7 3    13 8 5 2".split())) # 52 step 94sec
    # init_input = list(map(int, "1 10 15 4    13 6 3 8    2 9 12 7    14 5 0 11".split())) # 35 step 0.4sec

    # init_input = list(map(int, input("Type the instances of 15-puzzle (ex : 1 10 15 4 13 6 3 8 2 9 12 7 14 5 0 11):\n-> ").split()))
    # test = Solver(init_input)
    # test.solve()