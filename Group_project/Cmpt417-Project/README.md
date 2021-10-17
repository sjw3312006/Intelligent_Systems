# Cmpt417 Project

Jeongwoon Suh: 301313489/
Wei-Liang(Amos) Li: 301315811/
Huaiyan Chen: 301306475

## Introduction
 
This project will present the search algorithms, A*, IDA*, PEA* and EPEA* to solve the sliding-tile puzzle problems. 
Test instances will contain both the 3x3 8-Puzzle and 4x4 15-Puzzle of randomize instances that we have implemented. 
The performance of A*, IDA*, PEA* and EPEA* will be compared with different metrics. 
Specifically, the size of the open list, length of the solution, time complexity, number of generated and expanded nodes used in these searches for the measure of performance. 
The performance of heuristic search algorithms depends crucially on the effectiveness of the heuristic. 

In this report, we shall examine a pattern database(PDB), which is a powerful heuristic in the form of a pre-computed lookup table. 
Standard PDBs have a distinct entry in the table for each subproblem instance. 
Heuristics are typically implemented as functions from the states of the domain to a non-negative number. 
For example, a well-known heuristic for the sliding-tile puzzles is the Manhattan distance. 

Later in this report, we would like to analyze how the additive pattern databases allow us to sum the values of multiple pattern databases without violating admissibility, and are much more effective when they are applicable by using the A* and PEA* search algorithm that we have implemented, and specifically using the 5-5-5, and 6-6-3 partitioning of 15 puzzles for 21 random instances. 
Lastly, we will compare the efficiency of the performance of A* and PEA* for using these heuristics by using the different measure of the performance as mentioned above.

### Execution
We have implemented the source code based on Python 3.7.0.

#### Searches without Heuristics 
To execute the Puzzle problem without pdb heuristics, Execute 
	
	python3 Puzzle.py
It will automatically solve one random instance of the puzzle of the specified size. To modified how many instances should be generate, increase the value in the function call to experiment

#### PDB Heuristics
Please remember to modify the list of pattern that is located in PDB.py if you wish to change the PDB pattern
If you have modifiy the pattern of the PDB, please also change the pattern in get_heuristic_value function of Puzzle_pdb.py

To execute the 5-5-5 statistic-partitioning of randomized 15 puzzles,

	python3 PDB.py
	python3 Puzzle_pdb.py
	
To execute the 6-6-3 statistic-partitioning of randomized 15 puzzles,

	python3 PDB.py
	python3 Puzzle_pdb.py
	
