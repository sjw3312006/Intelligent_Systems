"""
Created on Mon Apr 20 2021

@author: jeongwoon_suh
"""

from collections import deque
import numpy as np
import pickle

# generating the PDB heuristics of 5-5-5 static-partitioning of 15 puzzles
class Pattern_generater():
    def __init__(self, A_pattern, B_pattern, C_pattern):
        self.__A_pattern = A_pattern
        self.__B_pattern = B_pattern
        self.__C_pattern = C_pattern

    # psedocode that is added to the Final_report
    def generate_bfs(self, init_pattern):
        pdb = dict()
        X = [1, 0, -1, 0]
        Y = [0, 1, 0, -1]
        pattern_numbers = [n for n in init_pattern if n != 0]

        queue = deque([(init_pattern, 0)])
        pdb[str(init_pattern)] = 0

        while queue:
            cur_state, cur_moves = queue.popleft()

            for pattern_number in pattern_numbers:
                move_tile = cur_state.index(pattern_number)
                i, j = move_tile // 4, move_tile % 4
                for dx, dy in zip(X, Y):
                    x, y = i + dx, j + dy
                    new_state = np.array(np.array(cur_state).reshape(4, 4))
                    if 0 <= x < 4 and 0 <= y < 4 and new_state[x, y] == 0:
                        new_state[i, j], new_state[x, y] = new_state[x, y], new_state[i, j]
                        new_state = new_state.flatten().tolist()
                        if str(new_state) not in pdb:
                            queue.append((new_state, cur_moves + 1))
                            pdb[str(new_state)] = cur_moves + 1
        return pdb 

    def generate_pdb_pattern(self):
        print("Start generating the pdb pattern ...")

        A_pattern_database = self.generate_bfs(self.__A_pattern)
        database = open("database_A", "wb")
        pickle.dump(A_pattern_database, database)
        database.close()
        print("database A generated")

        B_pattern_database = self.generate_bfs(self.__B_pattern)
        database = open("database_B", "wb")
        pickle.dump(B_pattern_database, database)
        database.close()
        print("database B generated")

        C_pattern_database = self.generate_bfs(self.__C_pattern)
        database = open("database_C", "wb")
        pickle.dump(C_pattern_database, database)
        database.close()
        print("database C generated")


if __name__ == "__main__":
    A = list(map(int, "1 2 3 0 5 6 0 0 0 0 0 0 0 0 0 0".split()))
    B = list(map(int, "0 0 0 4 0 0 7 8 0 0 11 12 0 0 0 0".split()))
    C = list(map(int, "0 0 0 0 0 0 0 0 9 10 0 0 13 14 15 0".split()))
    test_generater = Pattern_generater(A, B, C)
    test_generater.generate_pdb_pattern()
  
  
"""
For example, 3 different partitioning are as follows:

The 5-5-5 partition is this
        pattern A                  pattern B                  pattern C
=========================  =========================  =========================
|  1  |  2  |  3  |  4  |  |  *  |  *  |  *  |  *  |  |  *  |  *  |  *  |  *  |
|  *  |  *  |  7  |  *  |  |  5  |  6  |  *  |  *  |  |  *  |  *  |  *  |  8  |
|  *  |  *  |  *  |  *  |  |  9  | 10  |  *  |  *  |  |  *  |  *  | 11  | 12  |
|  *  |  *  |  *  |  *  |  | 13  |  *  |  *  |  *  |  |  *  | 14  | 15  |  *  |
=========================  =========================  =========================
"""
"""
The 6-6-3 partition is this
        pattern A                  pattern B                  pattern C
=========================  =========================  =========================
|  1  |  *  |  *  |  *  |  |  *  |  *  |  *  |  *  |  |  *  |  2  |  3  |  4  |
|  5  |  6  |  *  |  *  |  |  *  |  *  |  7  |  8  |  |  *  |  *  |  *  |  *  |
|  9  | 10  |  *  |  *  |  |  *  |  *  | 11  | 12  |  |  *  |  *  |  *  |  *  |
| 13  |  *  |  *  |  *  |  |  *  | 14  | 15  |  *  |  |  *  |  *  |  *  |  *  |
=========================  =========================  =========================
"""
"""
The 7-8 partition is this
        pattern A                  pattern B
=========================  =========================
|  1  |  2  |  3  |  4  |  |  *  |  *  |  *  |  *  |
|  5  |  6  |  7  |  8  |  |  *  |  *  |  *  |  *  |
|  *  |  *  |  *  |  *  |  |  9  | 10  | 11  | 12  |
|  *  |  *  |  *  |  *  |  | 13  | 14  | 15  |  *  |
=========================  =========================
"""