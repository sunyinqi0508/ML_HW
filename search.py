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

from os import terminal_size
from pacman import GameState
from game import AgentState, Directions, GameStateData
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

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    from game import Directions
    st = Directions.STOP
    from util import Stack
    pos = problem.getStartState()
    open = Stack()
    visited = {}

    open.push((pos, 0, Directions.STOP))
    found = False
    history = Stack()

    dep = -1
    while(not problem.isGoalState(pos) and not open.isEmpty()):
        pos, _dep, his = open.pop()
        if visited.get(pos) is not None and visited[pos]:
            continue
        visited[pos] = True
        
        while dep >= _dep:
            dep -= 1
            history.pop()
        dep = _dep
        if(his is not st):
            history.push(his)
        if(problem.isGoalState(pos)):
            found = True
            break
        else:
            succ = problem.getSuccessors(pos)
            for next in succ:
                successor, action, _ = next
                if(visited.get(successor) is None or visited[successor] == False):
                    visited[successor] = False
                    open.push((successor, dep+1, action))

    ret = []
    if(found):
        while(not history.isEmpty()):
            ret = [history.pop()] + ret
    return ret
    

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    from util import Queue
    pos = problem.getStartState()

    open = Queue()
    visited = {}

    open.push((pos, []))
    found = False
    history = Queue()

    while(not problem.isGoalState(pos) and not open.isEmpty()):
        pos, his = open.pop()
        if visited.get(pos) is not None and visited[pos]:
            continue
        visited[pos] = True
        if(problem.isGoalState(pos)):
            found = True
            history = his
            break
        else:
            succ = problem.getSuccessors(pos)
            for next in succ:
                successor, action, _ = next
                if visited.get(successor) is None or visited[successor] == False:
                    visited[successor] = False
                    open.push((successor, his+[action]))

    ret = []
    if(found):
        ret = history
    return ret
def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    from util import Queue
    from util import PriorityQueue
    pos = problem.getStartState()
    open = PriorityQueue()
    visited = {} 
    open.push((pos,0, []),0)
    found = False
    history = Queue()

    
    while(not problem.isGoalState(pos) and not open.isEmpty()):
        pos, cost, his = open.pop()
        if visited.get(pos) is not None and visited[pos]:
            continue
        visited[pos] = True
        if(problem.isGoalState(pos)):
            found = True
            history = his
            break
        else:
            succ = problem.getSuccessors(pos)
            for next in succ:
                successor, action, _cost = next
                if(visited.get(successor) is None or visited.get(successor)==False):
                    visited[successor] = False
                    _c = _cost + cost
                    open.push((successor, _c, his+[action]), _c)

    ret = []
    if(found):
        ret = history
    return ret

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
