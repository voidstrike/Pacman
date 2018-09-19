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


class SearchController:
    def __init__(self, wrappedFringe, problem):
        self.fringe = wrappedFringe  # fringe implementation for different search algorithm
        self.parent_map = {}  # HashMap stores the parent state information ?
        self.visited_state = {}  # HashMap stores visited state in this problem
        self.start_state = problem.getStartState() # Generate the start state of this problem
        self.final_state = None

    def checkVisited(self, tar):
        return tar in self.visited_state

    def visit(self, tar):
        self.visited_state[tar] = True

    def addParent(self, child, parent, direction):
        self.parent_map[child] = (parent, direction)

    def getParent(self, state):
        return self.parent_map[state]

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

def genericSearch(problem, fringe, heuristic=nullHeuristic):
    # Variables initialization
    sc = SearchController(fringe, problem)

    # Item in fringe should have the form (STATE, DIR, PARENT, COST) - PRIORITY
    #   STATE -- Current State
    #   DIR -- Direction from PARENT
    #   PARENT -- THe actual parent state of STATE
    #   COST -- The total actual path cost from start state to this state
    #   PRIORITY -- Estimate cost from this state to goal state f(n) = g(n) + h(n)
    sc.fringe.push((sc.start_state, None, None, 0), 0)
    sc.addParent(sc.start_state, None, None)

    # Tree expansion
    while not sc.fringe.isEmpty():
        current_state, dir_from_parent, parent_state, cost_so_far = sc.fringe.pop()

        if sc.checkVisited(current_state):
            continue  # This state has been visited before
        else:
            sc.visit(current_state)
            sc.addParent(current_state, parent_state, dir_from_parent)
            if problem.isGoalState(current_state):
                sc.final_state = current_state
                break

            successor_list = problem.getSuccessors(current_state)  # generate legal successor states
            for state, direction, cost in successor_list:
                sc.fringe.push((state, direction, current_state, cost + cost_so_far),
                               cost + cost_so_far + heuristic(state, problem))

    # Action list construction
    action_list = []  # initialize the action list
    while True:
        if sc.getParent(sc.final_state)[0] is not None:
            action_list.append(sc.getParent(sc.final_state)[1])  # Append action in reverse order
            sc.final_state = sc.getParent(sc.final_state)[0]  # Update state according to its parent
        else:
            break
    return list(reversed(action_list))


def depthFirstSearch(problem):
    """ Search the deepest nodes in the search tree first. """
    # Variables initialization
    fringe = WrappedFringe(util.Stack())
    return genericSearch(problem, fringe)


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    fringe = WrappedFringe(util.Queue())
    return genericSearch(problem, fringe)


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    fringe = WrappedFringe(util.PriorityQueue())
    return genericSearch(problem, fringe)


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    fringe = WrappedFringe(util.PriorityQueue())
    return genericSearch(problem, fringe, heuristic)


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
