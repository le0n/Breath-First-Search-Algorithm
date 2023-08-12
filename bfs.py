import sys


class Node:
    def __init__(self, state, parent, action):
        self.state = state
        self.parent = parent
        self.action = action


class StackFrontier:
    def __init__(self):
        self.frontier = []

    def add(self, node):
        self.frontier.append(node)

    def contains_state(self, state):
        return any(node.state == state for node in self.frontier)

    def empty(self):
        return len(self.frontier) == 0

    def remove(self):
        if self.empty():
            raise Exception("empty frontier")
        else:
            node = self.frontier[:-1]
            self.frontier = self.frontier[:-1]
            return node


class QueueFrontier(StackFrontier):
    def remove(self):
        if self.empty():
            raise Exception("empty frontier")
        else:
            node = self.frontier[0]
            self.frontier = self.frontier[1:]
            return node


class Solver:
    """solve for optimal solution using BFS if one exist"""

    def __init__(self, intial, goal):
        self.initial = intial
        self.goal = goal
        self.solution = []

    def solve(self, initial=None, goal=None):
        number_explored = 0
        if initial is None or goal is None:
            initial = self.initial
            goal = self.goal

        start = Node(state=initial, parent=None, action=None)
        frontier = QueueFrontier()
        frontier.add(start)

        explored = set()

        while True:
            if frontier.empty():
                raise Exception("Frontier empty, no solution")

            # choose a node from frontier
            # using queue so remove from beginning of list
            node = frontier.remove()
            number_explored += 1

            # check for solution
            if node.state == goal:
                actions = []
                cells = []

                # follow parent nodes to find solution
                while node.parent is not None:
                    actions.append(node.action)
                    cells.append(node.state)
                    node = node.parent

                actions.reverse()
                cells.reverse()
                self.so
