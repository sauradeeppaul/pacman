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



'''
This code is for CSE 537 Fall 2018 by Priyanka Sangtani and Sauradeep Paul
'''

"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import sys
import util
import searchAgents

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
        print self, ":", actions
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

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

'''Generalized function that is called by all search functions'''
def genericSearch(problem, nodes, heuristic = nullHeuristic):
	# Initializing the start node with cost 0
	start = (problem.getStartState(), 0, [])
	pushToNodeList(nodes, start, 0)

	visited = []

	# Searching till the data structure is empty
	while not nodes.isEmpty():
		(currentNode, cost, path)=nodes.pop()

		# Since the graph is undirected we need to maintain a list of visited nodes to create a tree
		if currentNode not in visited:

			# Initializing path
			if path == None:
				path = []

			# Return goal state 
			if problem.isGoalState(currentNode):
				return path

			# Add current node to the list of visited nodes
			visited.append(currentNode)

			# Looping through non-visited children
			for childNode, direction, childCost in problem.getSuccessors(currentNode):
				if childNode not in visited:
					# Adding cost to previous path cost (g(n))
					newCost = cost + childCost

					# Calculating h(n) based on the heuristic
					h = newCost + heuristic(childNode, problem)

					# Adding new node to data structure
					newPath = [p for p in path]
					newPath.append(direction)
					newState = (childNode, newCost, newPath)
					pushToNodeList(nodes, newState, h)

'''Handling push for all data structures'''
def pushToNodeList(nodes, state, cost):
	if isinstance(nodes, util.PriorityQueue):
		nodes.push(state, cost)
	else:
		nodes.push(state)

'''DFS with a stack as the data structure to maintain child nodes'''
def depthFirstSearch(problem):
    # print "Start:", problem.getStartState()
    # print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    # print "Start's successors:", problem.getSuccessors(problem.getStartState())
    
    nodes=util.Stack()
    return genericSearch(problem, nodes)

'''BFS with a queue as the data structure to maintain child nodes'''
def breadthFirstSearch(problem):
    nodes=util.Queue()
    return genericSearch(problem, nodes)

'''UCS with a priority queue as the data structure to maintain child nodes'''
def uniformCostSearch(problem):
    nodes=util.PriorityQueue()
    return genericSearch(problem, nodes)

'''AStar with a priority queue as the data structure to maintain child nodes,
but also takes in heuristic type as an argument; nullHeuristic effective means
that AStar acts as UCS'''
def aStarSearch(problem, heuristic=nullHeuristic):
    nodes=util.PriorityQueue()
    return genericSearch(problem, nodes, heuristic)


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
