# coding=utf-8
# Each step is 0.03M
import math
import time
from enum import Enum

from typing import List


class State(Enum):
    NOT_VALID = -1
    NOT_VISITED = 0
    PARTIALLY_VISITED = 1
    VISITED = 2


class Node:
    def __init__(self, x: int, y: int, is_wall: bool):
        self.quickest_path: List[Node] = []
        self.x: int = x
        self.y: int = y
        self.is_wall: bool = is_wall
        if is_wall:
            self.state: State = State.NOT_VALID
        else:
            self.state: State = State.NOT_VISITED

    def equals(self, node) -> bool:
        return self.x == node.x and self.y == node.y

    def __str__(self) -> str:
        return "{x: " + self.x.__str__() + ", y: " + self.y.__str__() + "}"

    def __repr__(self) -> str:
        return "{x: " + self.x.__str__() + ", y: " + self.y.__str__() + "}"


def dijkstras(init: Node, dest: Node) -> List[Node]:
    steps: int = 1
    nodes: dict[tuple[int, int], Node] = {(init.x, init.y): init}
    top_level_nodes: List[Node] = [init]
    found: bool = False
    highest_x_y: Node = init
    while not found:
        old_top_level_nodes: List[Node] = top_level_nodes.copy()
        top_level_nodes = []
        print("Executing step " + steps.__str__() + ", current length of top level "
                                                    "nodes is " + len(old_top_level_nodes).__str__() + " and "
                                                                                                       "highest XY Node is " + highest_x_y.__str__())
        for node in old_top_level_nodes:
            node.state = State.VISITED
            x_inc: int = -3
            y_inc: int = -3
            for x in range(9):
                new_node: Node = nodes.get((node.x + x_inc, node.y + y_inc))
                if new_node is not None:
                    if not new_node.is_wall:
                        if len(node.quickest_path) + 1 < len(new_node.quickest_path):
                            path = node.quickest_path.copy()
                            path.append(node)
                            new_node.quickest_path = path

                        # If the node has been visited, it means it has already been traversed up until that point.
                        # Ignore the point and continue.
                        if new_node.state == State.VISITED:
                            x_inc += 3
                            if x_inc == 6:
                                x_inc = 0
                                y_inc += 3
                                if y_inc == 6:
                                    break
                            continue

                        # Only append the new node if it is not already present.
                        try:
                            index: int = top_level_nodes.index(new_node)
                        except ValueError:
                            if new_node.x > highest_x_y.x and new_node.y > highest_x_y.y:
                                highest_x_y = new_node

                            top_level_nodes.append(new_node)
                        finally:
                            x_inc += 3
                            if x_inc == 6:
                                x_inc = 0
                                y_inc += 3
                                if y_inc == 6:
                                    break
                            continue

                    else:
                        # This is a wall, and we shall not check this space.
                        continue
                else:
                    new_node = Node(node.x + x_inc, node.y + y_inc, False)
                    new_node.state = State.PARTIALLY_VISITED
                    new_node.quickest_path = node.quickest_path.copy()
                    new_node.quickest_path.append(node)
                    nodes[(new_node.x, new_node.y)] = new_node
                    if new_node.equals(dest):
                        # As the first occurrence of the destination node will always be the quickest path,
                        # there is no need to check all possible paths, just return the full path.
                        #
                        # This is always the quickest path as the cost will be the same for all nodes we
                        # generate in this iteration.
                        path = new_node.quickest_path.copy()
                        path.append(node)
                        return path

                if new_node.x > highest_x_y.x and new_node.y > highest_x_y.y:
                    highest_x_y = new_node

                top_level_nodes.append(new_node)
                x_inc += 3
                if x_inc == 6:
                    x_inc = 0
                    y_inc += 3
                    if y_inc == 6:
                        break

        if len(top_level_nodes) == 0:
            # No path was found, returning.
            return []
        steps += 1


def convert_to_node(x: float, y: float, is_wall: bool) -> Node:
    return Node((round_to_nearest(math.floor(x * 100), 3)), (round_to_nearest(math.floor(y * 100), 3)), is_wall)


def convert_to_coordinates(node: Node) -> (float, float):
    return node.x / 100.0, node.y / 100.0


def round_to_nearest(x: int, a: int) -> int:
    return x + (x % a)

if __name__ == '__main__':
    print(dijkstras(Node(0, 0, False), Node(402, 201, False)).__str__())