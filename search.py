# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called 
by Pacman agents (in searchAgents.py).
"""

import util
import heapq, pprint

pp = pprint.PrettyPrinter(indent=2)

class SearchProblem:
  """
  This class outlines the structure of a search problem, but doesn't implement
  any of the methods (in object-oriented terminology: an abstract class).
  
  You do not need to change anything in this class, ever.
  """
  
  def getStartState(self):
     """
     Returns the start state for the search problem 
     """
     util.raiseNotDefined()
    
  def isGoalState(self, state):
     """
       state: Search state
    
     Returns True if and only if the state is a valid goal state
     """
     util.raiseNotDefined()

  def getSuccessors(self, state):
     """
       state: Search state
     
     For a given state, this should return a list of triples, 
     (successor, action, stepCost), where 'successor' is a 
     successor to the current state, 'action' is the action
     required to get there, and 'stepCost' is the incremental 
     cost of expanding to that successor
     """
     util.raiseNotDefined()

  def getCostOfActions(self, actions):
     """
      actions: A list of actions to take
 
     This method returns the total cost of a particular sequence of actions.  The sequence must
     be composed of legal moves
     """
     util.raiseNotDefined()
           

def tinyMazeSearch(problem):
  """
  Returns a sequence of moves that solves tinyMaze.  For any other
  maze, the sequence of moves will be incorrect, so only use this for tinyMaze
  """
  from game import Directions
  s = Directions.SOUTH
  w = Directions.WEST
  return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
  """
  Search the deepest nodes in the search tree first
  [2nd Edition: p 75, 3rd Edition: p 87]
  
  Your search algorithm needs to return a list of actions that reaches
  the goal.  Make sure to implement a graph search algorithm 
  [2nd Edition: Fig. 3.18, 3rd Edition: Fig 3.7].
  
  To get started, you might want to try some of these simple commands to
  understand the search problem that is being passed in:
  
  print "Start:", problem.getStartState()
  print "Is the start a goal?", problem.isGoalState(problem.getStartState())
  print "Start's successors:", problem.getSuccessors(problem.getStartState())
  """
  "*** YOUR CODE HERE ***"
  from game import Directions

  start_state = problem.getStartState()
  state_stack = util.Stack()

  visited_states = []
  parent_states = {}
  dir_to_node = {}
  goal_state = []

  visited_states.append(start_state)

  if problem.isGoalState(start_state): 
    return [Directions.STOP]

  for state in problem.getSuccessors(start_state):
    parent_states[state[0]] = start_state
    dir_to_node[state[0]] = state[1]
    state_stack.push(state)

  while not state_stack.isEmpty():
    current_node = state_stack.pop()
    visited_states.append(current_node[0])

    if problem.isGoalState(current_node[0]):
      goal_state = current_node[0]
      break

    for state in problem.getSuccessors(current_node[0]):
      if state[0] not in visited_states and state[0] not in parent_states.keys():
        state_stack.push(state)
        parent_states[state[0]] = current_node[0]
        dir_to_node[state[0]] = current_node[1]

  node_path = []
  while True:
    node_path.insert(0, goal_state)
    if parent_states[goal_state] != start_state:
      goal_state = parent_states[goal_state]
    else:
      break

  dir_path = []
  for node in node_path:
    dir_path.append(dir_to_node[node])

  return dir_path


def breadthFirstSearch(problem):
  """
  Search the shallowest nodes in the search tree first.
  [2nd Edition: p 73, 3rd Edition: p 82]
  """
  "*** YOUR CODE HERE ***"
  from game import Directions

  start_state = problem.getStartState()
  state_queue = util.Queue()

  visited_states = []
  parent_states = {}
  dir_to_node = {}
  goal_state = []

  visited_states.append(start_state)

  if problem.isGoalState(start_state): 
    return [Directions.STOP]

  for state in problem.getSuccessors(start_state):
    parent_states[state[0]] = start_state
    dir_to_node[state[0]] = state[1]
    state_queue.push(state)

  while not state_queue.isEmpty():
    current_node = state_queue.pop()
    visited_states.append(current_node[0])

    if problem.isGoalState(current_node[0]):
      goal_state = current_node[0]
      break

    for state in problem.getSuccessors(current_node[0]):
      if state[0] not in visited_states and state[0] not in parent_states.keys():
        state_queue.push(state)
        parent_states[state[0]] = current_node[0]
        dir_to_node[state[0]] = current_node[1]

  node_path = []
  while True:
    node_path.insert(0, goal_state)
    if parent_states[goal_state] != start_state:
      goal_state = parent_states[goal_state]
    else:
      break

  dir_path = []
  for node in node_path:
    dir_path.append(dir_to_node[node])

  return dir_path
      
def uniformCostSearch(problem):
  "Search the node of least total cost first. "
  "*** YOUR CODE HERE ***"

  start_state = problem.getStartState()
  first_nodes = problem.getSuccessors(start_state)
  
  pq = util.PriorityQueue()
  visited = set([])

  for n in first_nodes:
    pq.push([n], n[2])

  while pq.heap:
    path = pq.pop()
    if problem.isGoalState(path[-1][0]):
      return [p[1] for p in path]

    visited.add(path[-1][0])

    for successor in problem.getSuccessors(path[-1][0]):
      if successor[0] not in visited:
        existing_priority, indexof = priorityQueueContains(pq, successor[0])
        if not existing_priority:
          new_path = list(path)
          new_path.append(successor)
          pq.push(new_path, successor[2])
        elif successor[2] < existing_priority:
          pq.heap[indexof][0] = successor[2] # change the priority
          heapq.heapify(pq.heap)             # then reset the pqueue # this might be slowing things down some

def priorityQueueContains(pq, item):
  for i, pair in enumerate(pq.heap):
    if pair[1][-1][0] == item: # my priority queue contains paths.  pair[1][-1][0] get the last state of those lists.
      return (pair[0], i)                                      # --> (priority, item) 
  return (False, -1)                                                          # --> [(state, dir, cost), ...]


def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

def aStarSearch(problem, heuristic=nullHeuristic):
  "Search the node that has the lowest combined cost and heuristic first."
  "*** YOUR CODE HERE ***"
  
  state_pq = util.PriorityQueue()
  visited_states = []
  parent_states = {}
  dir_to_node = {}

  goal_state = []

  start_state = problem.getStartState()
  first_nodes = problem.getSuccessors(start_state)

  for n in first_nodes:
    parent_states[n[0]] = start_state
    dir_to_node[n[0]] = n[1]
    state_pq.push(n, problem.getCostOfActions([n[1]]) + heuristic(n[0], problem))

  while not state_pq.isEmpty():
    current_state = state_pq.pop()
    visited_states.append(current_state[0])

    if problem.isGoalState(current_state[0]):
      goal_state = current_state[0]
      break

    for successor in problem.getSuccessors(current_state[0]):
      if successor[0] not in visited_states and successor[0] not in parent_states.keys():
        parent_states[successor[0]] = current_state[0]
        dir_to_node[successor[0]] = successor[1]
        back_to_start = []
        dir_back_to_start = []

        parent = successor[0]
        while True:
          back_to_start.insert(0, parent)
          if parent_states[parent] != start_state:
            parent = parent_states[parent]
          else:
            break

        for state in back_to_start:
          dir_back_to_start.append(dir_to_node[state])

        state_pq.push(successor, problem.getCostOfActions(dir_back_to_start) + heuristic(successor[0], problem))

  node_path = []
  while True:
    node_path.insert(0, goal_state)
    if parent_states[goal_state] != start_state:
      goal_state = parent_states[goal_state]
    else:
      break

  dir_path = []
  for state in node_path:
    dir_path.append(dir_to_node[state])

  return dir_path
  
  
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
