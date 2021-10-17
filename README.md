# Intelligent_Systems
Intelligent Systems using modern constraint programming and heuristic search methods. A survey of this rapidly advancing technology as applied to scheduling, planning, design and configuration. An introduction to constraint programming, heuristic search, constructive (backtrack) search, iterative improvement (local) search, mixed-initiative systems and combinatorial optimization.

## Individual Project
In this project, I have implemented based on Multi-Agent Path Findings (MAPF) and implement a single-angle solver, namely space-time A*, and parts of three MAPF solvers, namely prioritized planning, Conflict-Based Search (CBS), and CBS with disjoint splitting.

This project requires a Python3 installation with the numpy and matplotlib packages. On Ubuntu Linux, download python by using:
  
     sudo apt install python3 python3 numpy python3 matplotlib
     
### Execute the independent MAPF solver by using: 
      python run_experiments.py --instance instances/exp0.txt --solver Independent
      
## Task 1 : Implement Space-Time A*
### Task 1.1 : Searching in the Space-Time Domain
     python run_experiments.py --instance instances/exp1.txt --solver Independent
     
### Task 1.2 : Handling Vertex Constraints
     python run_experiments.py --instance instances/exp1.txt --solver Prioritized
   
         
## Task 2 : Implement Prioiritized Planning
### Task 2.1 : Adding Vertex Constraints 
     python run_experiments.py --instance instances/exp2_1.txt --solver Prioritized

### Task 2.3 : Adding Additional Constraints 
      python run_experiments.py --instance instances/exp2_2.txt --solver Prioritized
      
### Task 2.4 : Addressing Failures
       python run_experiments.py --instance instances/exp2_3.txt --solver Prioritized
       python run_experiments.py --instance instances/exp2_2.txt --solver Prioritized
                
## Task 3 : Implement Conflict-Based Search (CBS)
### Task 3.1 : Detecting Collisions
     python run_experiments.py --instance instances/exp3_1.txt --solver CBS

### Task 3.4 : Testing Implementation
      python run_experiments.py --instance "instances/test_*" --solver CBS --batch
      
 ## Task 4 : Implementing Conflict-Based Search (CBS) with Disjoint Splitting
      python run_experiments.py --instance instances/exp4.txt --solver CBS
