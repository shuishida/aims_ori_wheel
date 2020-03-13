from scipy.spatial import KDTree
import numpy as np
import matplotlib.pyplot as plt


def min_cost(q, cost):
    min_node = None
    for node in q:
        if min_node == None:
            min_node = node
        elif cost[node] < cost[min_node]:
            min_node = node
    return min_node


class PRM(object):
    def __init__(self, points, line_collision, k):
        self.points = points
        self.nodes = [Node(point) for point in points]
        self.kd = KDTree(np.array(points))
        self.line_collision = line_collision
        for node in self.nodes:
            neighbors = self.get_k_nearest_nodes(node.point, k)
            for neighbor in neighbors:
                if not line_collision(node.point, neighbor.point):
                    node.add(neighbor)
        # self.plot()

    def plot(self):
        xs, ys = tuple(zip(*[node.point for node in self.nodes]))
        plt.scatter(xs, ys)
        for node in self.nodes:
            for neighbor, cost in node.neighbors:
                x1, y1 = node.point
                x2, y2 = neighbor.point
                plt.plot([x1, x2], [y1, y2])
        plt.show()

    def get_k_nearest_nodes(self, point, k):
        _, inds = self.kd.query(point, k=k)
        if isinstance(inds, int):
            inds = [inds]
        return [self.nodes[i] for i in inds]

    def plan(self, start_point, goal_point):
        start_node = Node(start_point)
        end_node = Node(goal_point)

        def find_nearest_node(node):
            for neighbor in self.get_k_nearest_nodes(node.point, 10):
                if not self.line_collision(node.point, neighbor.point):
                    return neighbor
            raise Exception("sample size needs to be increased")

        source_node = find_nearest_node(start_node)
        terminate_node = find_nearest_node(end_node)

        q = set()
        cost = {}
        parent = {}

        for v in self.nodes:  # initialization
            cost[v] = np.inf  # unknown distance from source to v
            parent[v] = None  # previous node in optimal path from source
            q.add(v)

        # distance from source to source
        cost[source_node] = 0
        parent[source_node] = start_node
        curr_node = None

        while q:
            # node with the least distance selected first
            curr_node = min_cost(q, cost)
            q.remove(curr_node)

            print("curr_node: %s" % cost[curr_node])

            if curr_node is terminate_node:
                print("found terminal node")
                break

            for neighbor, edge_cost in curr_node.neighbors:
                new_cost = cost[curr_node] + edge_cost
                if new_cost < cost[neighbor]:
                    # a shorter path to v has been found
                    cost[neighbor] = new_cost
                    parent[neighbor] = curr_node

        if curr_node is not terminate_node:
            raise Exception("Could not find path")

        print("Constructing path...")

        path = []
        while curr_node is not start_node:
            path.insert(0, curr_node)
            curr_node = parent[curr_node]

        path = [start_node] + path[1:-1] + [end_node]

        print(path)

        return [node.point for node in path]


class Node(object):
    def __init__(self, point):
        self.point = point
        self.neighbors = [] # tuple of (neighbor_node, traversal_cost)

    def __str__(self):
        return "node: " + str(self.point)

    def dist(self, other):
        x1, y1 = self.point
        x2, y2 = other.point
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

    def add(self, node):
        self.neighbors.append((node, self.dist(node)))
