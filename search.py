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


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def genericSearch(problem, frointer="stack"):
    import util
    from game import Directions
    pass




def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    fringe = util.Stack()  # Initialize fringe as STACK for DFS
    parent_map = {}
    visited_state = {}
    flag = False

    fringe.push(problem.getStartState())
    parent_map[problem.getStartState()] = None
    visited_state[problem.getStartState()] = True
    final_state = None

    while not fringe.isEmpty():
        current_state = fringe.pop()
        successor_list = problem.getSuccessors(current_state)

        for state, direction, cost in successor_list:
            if not visited_state.has_key(state):
                visited_state[state] = True
                parent_map[state] = (current_state, direction)
                if problem.isGoalState(state):
                    final_state = state
                    flag = True
                    break
                fringe.push(state)
            else:
                continue

        if flag:
            break

    action_list = []  # initialize the action list
    if flag:
        while True:
            if parent_map[final_state] is not None:
                action_list.append(parent_map[final_state][1])  # Append action in reverse order
                final_state = parent_map[final_state][0]  # Update state according to its parent
            else:
                break

    return list(reversed(action_list))

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    fringe = util.Queue()  # Initialize fringe as STACK for BFS
    parent_map = {}
    visited_state = {}
    flag = False

    fringe.push(problem.getStartState())
    parent_map[problem.getStartState()] = None
    visited_state[problem.getStartState()] = True
    final_state = None

    while not fringe.isEmpty():
        current_state = fringe.pop()
        successor_list = problem.getSuccessors(current_state)

        for state, direction, cost in successor_list:
            if not visited_state.has_key(state):
                visited_state[state] = True
                parent_map[state] = (current_state, direction)
                if problem.isGoalState(state):
                    final_state = state
                    flag = True
                    break
                fringe.push(state)
            else:
                continue

        if flag:
            break

    action_list = []  # initialize the action list
    if flag:
        while True:
            if parent_map[final_state] is not None:
                action_list.append(parent_map[final_state][1])  # Append action in reverse order
                final_state = parent_map[final_state][0]  # Update state according to its parent
            else:
                break

    return list(reversed(action_list))

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    fringe = util.PriorityQueue()  # Initialize fringe as STACK for UCS
    parent_map = {}
    visited_state = {}
    flag = False

    fringe.push((problem.getStartState(), 0), 0)
    parent_map[problem.getStartState()] = None
    visited_state[problem.getStartState()] = True
    final_state = None

    while not fringe.isEmpty():
        current_state, cost_so_far = fringe.pop()
        if problem.isGoalState(current_state):
            final_state = current_state
            flag = True
            break

        successor_list = problem.getSuccessors(current_state)

        for state, direction, cost in successor_list:
            if not visited_state.has_key(state):
                visited_state[state] = True
                parent_map[state] = (current_state, direction)
                fringe.push((state, cost+cost_so_far), cost+cost_so_far)
            else:
                continue

    action_list = []  # initialize the action list
    if flag:
        while True:
            if parent_map[final_state] is not None:
                action_list.append(parent_map[final_state][1])  # Append action in reverse order
                final_state = parent_map[final_state][0]  # Update state according to its parent
            else:
                break

    return list(reversed(action_list))

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    fringe = util.PriorityQueue()  # Initialize fringe as STACK for UCS
    parent_map = {}
    visited_state = {}
    flag = False
    start_state = problem.getStartState()

    fringe.push((start_state, 0), heuristic(start_state, problem))
    parent_map[start_state] = None
    visited_state[start_state] = True
    final_state = None

    while not fringe.isEmpty():
        current_state, cost_so_far = fringe.pop()
        if problem.isGoalState(current_state):
            final_state = current_state
            flag = True
            break

        successor_list = problem.getSuccessors(current_state)

        for state, direction, cost in successor_list:
            if not visited_state.has_key(state):
                visited_state[state] = True
                parent_map[state] = (current_state, direction)
                fringe.push((state, cost + cost_so_far), cost + cost_so_far + heuristic(state, problem))
            else:
                continue

    action_list = []  # initialize the action list
    if flag:
        while True:
            if parent_map[final_state] is not None:
                action_list.append(parent_map[final_state][1])  # Append action in reverse order
                final_state = parent_map[final_state][0]  # Update state according to its parent
            else:
                break

    return list(reversed(action_list))


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
