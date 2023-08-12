import sys
import time


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
        self.solution = None
        self.explored = set()
        self.number_explored = 0

    def get_parity(self, state):
        inversion = 0
        state = state.replace("_", "")
        for i in range(len(state)):  # Skip the blank tile
            if i + 1 < len(state):
                for j in range(i + 1, len(state)):
                    if int(state[j]) < int(state[i]):
                        inversion += 1
        return inversion % 2

    def is_solvable(self, state1, state2):
        parity_1 = self.get_parity(state1)
        parity_2 = self.get_parity(state2)
        print("parity 1 is:", parity_1, "parity 2 is:", parity_2)
        return parity_1 == parity_2

    def neighbours(self, state):
        """given a state, return a list of valid moves and the corresponding new state"""
        result = []
        legalMoves = [("right", 1), ("left", -1), ("down", 3), ("up", -3)]
        sourceLocation = state.index("_")

        for name, move in legalMoves:
            destinationLocation = sourceLocation + move

            if destinationLocation >= len(state) or destinationLocation < 0:
                # move out of bounds
                continue
            else:
                # create a new state based on move
                newState = ""
                stateList = list(state)

                swap = stateList[sourceLocation + move]
                stateList[sourceLocation + move] = "_"
                stateList[sourceLocation] = swap
                result.append((name, newState.join(stateList)))
        return result

    def print_solution(self):
        if self.solution is None:
            print("solution is empty")
        else:
            print("actions for solution: ", self.solution[0])

    def solve(self, initial=None, goal=None):
        if initial is None or goal is None:
            initial = self.initial
            goal = self.goal

        if not self.is_solvable(initial, goal):
            raise Exception("Different parity, this is not solvable")

        start = Node(state=initial, parent=None, action=None)
        frontier = QueueFrontier()
        frontier.add(start)

        while True:
            if frontier.empty():
                raise Exception("Frontier empty, no solution")

            # choose a node from frontier
            # using queue so remove from beginning of list
            node = frontier.remove()
            self.number_explored += 1

            # check for solution
            if node.state == goal:
                actions = []
                cells = []

                # follow parent nodes to find solution
                while node.parent is not None:
                    actions.append(node.action)
                    cells.append(node.state)
                    node = node.parent

                # reverse actions
                actions.reverse()
                cells.reverse()
                self.solution = (actions, cells)
                return

            # if not solution, mark node as explored
            self.explored.add(node.state)

            for action, state in self.neighbours(node.state):
                if not frontier.contains_state(state) and state not in self.explored:
                    child = Node(state=state, parent=node, action=action)
                    frontier.add(child)


if len(sys.argv) != 3:
    sys.exit("Usage: python bfs.py initial goal")

m = Solver(sys.argv[1], sys.argv[2])
start_time = time.perf_counter()
print("Solving...")
m.solve()
end_time = time.perf_counter()
print("States Explored:", m.number_explored)
time_elapsed = round((end_time - start_time) * 1000, 5)
print("Time elasped:", time_elapsed, "milliseconds")
print("Solution:")
m.print_solution()
