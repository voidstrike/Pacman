# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()

class WrappedFringe:
    # Wrapper class for different fringe implementations
    def __init__(self, inner):
        self.inner = inner  # Actual data structure, could be util.Stack, Queue or PriorityQueue
        if isinstance(inner, util.PriorityQueue):
            self.flag = 2
        else:
            self.flag = 1

    def push(self, item, cost):
        if self.flag == 1:
            self.inner.push(item)
        else:
            self.inner.push(item, cost)

    def pop(self):
        return self.inner.pop()

    def isEmpty(self):
        return self.inner.isEmpty()

def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def constructActionList(parent_map, final_state):
    action_list = []  # initialize the action list
    while True:
        if parent_map[final_state] is not None:
            print parent_map[final_state]
            action_list.append(parent_map[final_state][1])  # Append action in reverse order
            final_state = parent_map[final_state][0]  # Update state according to its parent
        else:
            break
    return list(reversed(action_list))

def genericSearch(problem, fringe, after_check_flag, heuristic=nullHeuristic):
    # Variables initialization
    parent_map = {}  # HashMap stores the parent information
    visited_state = {}  # HashMap stores the visited position
    start_state = problem.getStartState()
    flag = False # Boolean Flag used to define whether the goal state has been found
    final_state = None

    # Item in fringe should have the form (STATE, COST) - PRIORITY
    # STATE -- Current State
    # COST -- The total actual path cost from start state to this state
    # PRIORITY -- Estimate cost from this state to goal state f(n) = g(n) + h(n)
    fringe.push((start_state, 0), heuristic(start_state, problem))
    parent_map[start_state] = None
    visited_state[start_state] = True

    # Tree expansion process
    if after_check_flag:  # The algorithm is required to perform goal test during tree expansion
        while not fringe.isEmpty():
            current_state, cost_so_far = fringe.pop()
            visited_state[current_state] = True
            if problem.isGoalState(current_state):
                final_state = current_state
                flag = True
                break

            successor_list = problem.getSuccessors(current_state) # generate legal successor states
            for state, direction, cost in successor_list:
                if visited_state.has_key(state):
                    continue
                else:
                    parent_map[state] = (current_state, direction)
                    fringe.push((state, cost + cost_so_far), cost + cost_so_far + heuristic(state, problem))

    else:  # The algorithm is required to perform goal test while first encounter a state
        while not fringe.isEmpty():
            current_state, cost_so_far = fringe.pop()
            successor_list = problem.getSuccessors(current_state)

            for state, direction, cost in successor_list:
                if visited_state.has_key(state):
                    continue
                else:
                    parent_map[state] = (current_state, direction)
                    visited_state[state] = True
                    if problem.isGoalState(state):
                        final_state = state
                        flag = True
                        break
                    fringe.push((state, cost + cost_so_far), cost + cost_so_far + heuristic(state, problem))

            if flag:
                break

    # Action list construction
    return constructActionList(parent_map, final_state)

def depthFirstSearch(problem):
    """ Search the deepest nodes in the search tree first. """
    # # Variables initialization
    # fringe = WrappedFringe(util.Stack())
    # parent_map = {}  # HashMap stores the parent information
    # visited_state = {}  # HashMap stores the visited position
    # start_state = problem.getStartState()
    # flag = False  # Boolean Flag used to define whether the goal state has been found
    # final_state = None
    #
    # # Item in fringe should have the form (STATE, COST) - PRIORITY
    # # STATE -- Current State
    # # COST -- The total actual path cost from start state to this state
    # # PRIORITY -- Estimate cost from this state to goal state f(n) = g(n) + h(n)
    # fringe.push((start_state, 0), 0)
    # parent_map[start_state] = None
    # visited_state[start_state] = True
    #
    # # Tree expansion process
    # while not fringe.isEmpty():
    #     current_state, cost_so_far = fringe.pop()
    #     visited_state[current_state] = True
    #     if problem.isGoalState(current_state):
    #         final_state = current_state
    #         flag = True
    #         break
    #
    #     successor_list = problem.getSuccessors(current_state)  # generate legal successor states
    #     for state, direction, cost in successor_list:
    #         if visited_state.has_key(state):
    #             continue
    #         else:
    #             parent_map[state] = (current_state, direction)
    #             fringe.push((state, 0), 0)
    #
    # # Action list construction
    # return constructActionList(parent_map, final_state)
    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    # # Variables initialization
    # fringe = WrappedFringe(util.Queue())  # Queue as fringe in BFS
    # parent_map = {}  # HashMap stores the parent information
    # visited_state = {}  # HashMap stores the visited position
    # start_state = problem.getStartState()
    # flag = False  # Boolean Flag used to define whether the goal state has been found
    # final_state = None
    #
    # # Item in fringe should have the form (STATE, COST) - PRIORITY
    # # STATE -- Current State
    # # COST -- The total actual path cost from start state to this state
    # # PRIORITY -- Estimate cost from this state to goal state f(n) = g(n) + h(n)
    # fringe.push((start_state, 0), 0)
    # parent_map[start_state] = None
    # visited_state[start_state] = True
    #
    # # Tree expansion process
    # while not fringe.isEmpty():
    #     current_state, cost_so_far = fringe.pop()
    #     successor_list = problem.getSuccessors(current_state)
    #
    #     for state, direction, cost in successor_list:
    #         if visited_state.has_key(state):
    #             continue
    #         else:
    #             parent_map[state] = (current_state, direction)
    #             visited_state[state] = True
    #             if problem.isGoalState(state):
    #                 final_state = state
    #                 flag = True
    #             fringe.push((state, cost + cost_so_far), 0)
    #     if flag:
    #         break
    #
    # # Action list construction
    # return constructActionList(parent_map, final_state)
    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    # Variables initialization
    fringe = WrappedFringe(util.PriorityQueue()) # PriorityQueue as fringe in UCS
    parent_map = {}  # HashMap stores the parent information
    visited_state = {}  # HashMap stores the visited position
    start_state = problem.getStartState()
    final_state = None

    # Item in fringe should have the form (STATE, COST) - PRIORITY
    # STATE -- Current State
    # COST -- The total actual path cost from start state to this state
    # PRIORITY -- Estimate cost from this state to goal state f(n) = g(n) + h(n)
    fringe.push((start_state, 0), 0)
    parent_map[start_state] = None
    # visited_state[(start_state, 0)] = True

    # Tree expansion process
    while not fringe.isEmpty():
        current_state, cost_so_far = fringe.pop()
        print(current_state)

        if visited_state.has_key(current_state):
            continue  # This state has benn visited with smaller path cost
        else:
            visited_state[current_state] = True
            if problem.isGoalState(current_state):
                final_state = current_state
                break

            successor_list = problem.getSuccessors(current_state)  # generate legal successor states
            for state, direction, cost in successor_list:
                parent_map[state] = (current_state, direction)
                fringe.push((state, cost + cost_so_far), cost + cost_so_far)

    # Action list construction
    print final_state
    return []
    # return constructActionList(parent_map, final_state)
    # util.raiseNotDefined()

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    # fringe = util.PriorityQueue() # PriorityQueue as fringe and consider heuristic function in A*
    # return genericSearch(problem, WrappedFringe(fringe), True, heuristic)
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
