#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os  # for time functions
import math  # for infinity
from search import *  # for search engines
from sokoban import sokoban_goal_state, SokobanState, Direction, PROBLEMS  # for Sokoban specific classes and problems

# SOKOBAN HEURISTICS
def heur_alternate(state):
    # IMPLEMENT
    '''a better heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # heur_manhattan_distance has flaws.
    # Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    # Your function should return a numeric value for the estimate of the distance to the goal.
    # EXPLAIN YOUR HEURISTIC IN THE COMMENTS. Please leave this function (and your explanation) at the top of your solution file, to facilitate marking.
    
    # This heuristic function estimates the cost to reach the goal state in the Sokoban problem by combining 
    # box-to-storage and robot-to-box matching based on Manhattan distance, while accounting for deadlock 
    # situations. The function is designed to optimize assignments and detect unsolvable states caused 
    # by deadlocks or insufficient storages.

    # The function has the following steps:
    # 1. Goal Check: Returns 0 if all boxes are on storages.
    # 2. Deadlock Detection: Identifies immovable boxes due to corners, walls, or clusters. If any box is in a deadlock, returns -99.
    # 3. Box-to-Storage Matching: Greedily assigns boxes to the closest available storages using Manhattan distance. Returns -99 if there are insufficient storages.
    # 4. Robot-to-Box Matching: Greedily assigns robots to the closest available boxes using Manhattan distance.
    
    # Return zero if we are in the goal state
    if sokoban_goal_state(state):
        return 0

    # Used to check if the box is locked in place because of a corner or walls
    def in_deadlock(box, state):
        x, y = box

        if (x, y) in state.storage:
            return False
        
        # Corner deadlock if a box is stuck in a corner without storage
        if ((x-1,y) in state.obstacles or (x+1,y) in state.obstacles) and ((x,y-1) in state.obstacles or (x,y+1) in state.obstacles):
            return True
        
        # Blocked by another box, plus two obstacles in cluster
        if ((x,y-1) in state.boxes) and (((x+1,y) in state.obstacles and (x+1,y-1) in state.obstacles) or ((x-1,y) in state.obstacles and (x-1,y-1) in state.obstacles)):
            return True

        # Cluster of boxes
        if ((x-1,y) in state.boxes) and ((x,y+1) in state.boxes) and ((x-1,y+1) in state.boxes):
            return True

    total_distance = 0

    # Greedy Box-to-Storage Matching
    boxes = list(state.boxes)
    storages = list(state.storage)

    # Check if enough storages
    if len(boxes) > len(storages):
        return -99

    assigned_storages = set()
    unassigned_boxes = set(boxes)

    # loop through until all boxes are assigned
    while unassigned_boxes:
        best_box = None
        best_storage = None
        best_distance = float('inf')

        # loop through boxes
        for box in unassigned_boxes:
            # Check for deadlock
            if in_deadlock(box, state):
                return -99
            
            # Loop through storages
            for storage in storages:
                # Skip already assigned storage
                if storage in assigned_storages:
                    continue  
                # Calculated manhattan distance to storage
                distance = abs(box[0] - storage[0]) + abs(box[1] - storage[1])
                # Assign best distance
                if distance < best_distance:
                    best_distance = distance
                    best_box = box
                    best_storage = storage
        # Update closest storage and remove box from unassigned
        if best_box and best_storage:
            total_distance += best_distance
            assigned_storages.add(best_storage)
            unassigned_boxes.remove(best_box)

    # Greedy Robot-to-Box Matching
    robots = list(state.robots)
    boxes = list(state.boxes)
    
    # If there are still boxes to be assigned to a robot
    if robots and boxes:
        assigned_robots = set()
        assigned_boxes = set()
        total_robot_distance = 0
        # loop through until all boxes are assigned
        while len(assigned_robots) < len(robots) and len(assigned_boxes) < len(boxes):
            best_distance = float('inf')
            best_robot = None
            best_box = None

            for robot in robots:
                if robot in assigned_robots:
                    continue
                
                for box in boxes:
                    if box in assigned_boxes:
                        continue
                    # Calculate manhattan distance to box
                    distance = abs(robot[0] - box[0]) + abs(robot[1] - box[1])
                    if distance < best_distance:
                        best_distance = distance
                        best_robot = robot
                        best_box = box
            # Update closest robot and remove box from unassigned
            if best_robot and best_box:
                assigned_robots.add(best_robot)
                assigned_boxes.add(best_box)
                total_robot_distance += best_distance

    total_distance += total_robot_distance
    return total_distance
   
def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0

def heur_manhattan_distance(state):
    # IMPLEMENT
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # We want an admissible heuristic, which is an optimistic heuristic.
    # It must never overestimate the cost to get from the current state to the goal.
    # The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to it is such a heuristic.
    # When calculating distances, assume there are no obstacles on the grid.
    # You should implement this heuristic function exactly, even if it is tempting to improve it.
    # Your function should return a numeric value; this is the estimate of the distance to the goal.
    total_distance = 0

    for box in state.boxes:
        # Calculate the Manhattan distance to each storage
        if box not in state.storage:
            distances = [abs(box[0] - storage[0]) + abs(box[1] - storage[1]) for storage in state.storage]
            # Find the minimum distance for the current box
            min_distance = min(distances)
            # Add the minimum distance to the total
            total_distance += min_distance

    return total_distance

def fval_function(sN, weight):
    # IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
    g_n = sN.gval
    h_n = sN.hval

    # Weighted A* f-value formula
    return g_n + weight * h_n  

# SEARCH ALGORITHMS
def weighted_astar(initial_state, heur_fn, weight, timebound):
    # IMPLEMENT    
    '''Provides an implementation of weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of weighted astar algorithm'''
    # Initialize the search engine with the "custom" search strategy
    # Use full cycle checking
    se = SearchEngine(strategy="custom", cc_level="full")

    # Wrap fval_function to provide the weight parameter
    wrapped_fval_function = lambda sN: fval_function(sN, weight)

    # Initialize search with the given heuristic and custom f-value function
    se.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_function)

    # Run the search in the given timebound
    goal_state, search_stats = se.search(timebound)

    # Return the goal state and search statistics
    return goal_state, search_stats


def iterative_astar(initial_state, heur_fn, weight=1, timebound=5):  # uses f(n), see how autograder initializes a search line 88
    # IMPLEMENT
    '''Provides an implementation of realtime a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of iterative astar algorithm'''
    start_time = os.times()[0]
    best_solution = None
    best_stats = None
    best_cost = math.inf

    # Reduce weight in each iteration
    current_weight = weight
    decay_factor = 0.5  

    remaining_time = timebound

    while remaining_time > 0:
        # Run weighted A* search with the current weight
        goal_state, search_stats = weighted_astar(initial_state, heur_fn, current_weight, remaining_time)

        # Check if a solution was found
        if goal_state:
            # Update best solution if this one is better
            if goal_state.gval < best_cost:
                best_solution = goal_state
                best_stats = search_stats
                best_cost = goal_state.gval

        # Reduce weight (to favor optimal paths) and update time
        current_weight *= decay_factor
        elapsed_time = os.times()[0] - start_time
        remaining_time = timebound - elapsed_time

        # Stop if no more nodes to expand (search fully exhausted)
        if not search_stats or search_stats.states_generated == 0:
            break

        if remaining_time <= 0:
            break

    return best_solution, best_stats

def iterative_gbfs(initial_state, heur_fn, timebound=5):  # only use h(n)
    # IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of iterative gbfs algorithm'''
    
    start_time = os.times()[0]
    best_solution = None
    best_stats = None
    best_cost = math.inf

    remaining_time = timebound

    # GBFS always prioritizes h(n), so we use a best-first search strategy
    search_engine = SearchEngine('best_first', 'full')  
    search_engine.init_search(initial_state, sokoban_goal_state, heur_fn)

    while remaining_time > 0:
        # Run GBFS with current remaining time, prune if g(n) is worse than the best found solution
        goal_state, search_stats = search_engine.search(remaining_time, costbound=(best_cost, math.inf, math.inf))

        if goal_state:
            # Update if this solution is better
            if goal_state.gval < best_cost:  
                best_solution = goal_state
                best_stats = search_stats
                best_cost = goal_state.gval

        # Update remaining time
        elapsed_time = os.times()[0] - start_time
        remaining_time = timebound - elapsed_time

        # Stop if no more nodes to expand
        if not search_stats or search_stats.states_generated == 0:
            break

    return best_solution, best_stats