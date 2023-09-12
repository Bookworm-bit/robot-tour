#!/usr/bin/env python3

from colorama import init, Fore, Back, Style

# import ev3dev2.motor as motor
import math
import heapq


class Robot:
    def __init__(self, left_motor_port, right_motor_port, wheel_diameter, wheel_distance, bot_length, bot_width, target_time):
        self.left_motor = motor.LargeMotor(left_motor_port)
        self.right_motor = motor.LargeMotor(right_motor_port)
        self.wheel_diameter = wheel_diameter
        self.wheel_distance = wheel_distance
        self.bot_length = bot_length
        self.bot_width = bot_width
        self.direction = "up"
        self.rotations = 0
        self.drive_movements = 0
        self.target_time = target_time

    def distance_to_rotations(self, distance):
        return distance / (math.pi * self.wheel_diameter)

    def rotate(self, direction):
        turn = 1 if direction == "right" else -1

        self.left_motor.on_for_degrees(
            self.distance_to_rotations(self.bot_width / 2) * 360 * turn, 50)
        self.right_motor.on_for_degrees(
            self.distance_to_rotations(self.bot_width / 2) * 360 * turn, 50)

        self.rotations += 1

    def move(self, direction):
        direction = 1 if direction == "forward" else -1

        self.left_motor.on_for_degrees(
            self.distance_to_rotations(250) * 360 * direction, 50)
        self.right_motor.on_for_degrees(
            self.distance_to_rotations(250) * 360 * direction, 50)

        self.drive_movements += 1

    def follow_path(self, graph):
        path = None
        self.drive_movements += len(path)

        for i in range(len(path) - 1):
            current_node = path[i]
            next_node = path[i + 1]

            y_diff = next_node[0] - current_node[0]
            x_diff = next_node[1] - current_node[1]

            if x_diff == 1:
                if self.direction == "left":
                    self.move("backward")
                elif self.direction == "right":
                    self.move("forward")
                elif self.direction == "up":
                    self.rotate("right")
                    self.move("forward")
                    self.direction = "right"
                elif self.direction == "down":
                    self.rotate("left")
                    self.move("forward")
                    self.direction = "right"
            elif x_diff == -1:
                if self.direction == "left":
                    self.move("forward")
                elif self.direction == "right":
                    self.move("backward")
                elif self.direction == "up":
                    self.rotate("left")
                    self.move("forward")
                    self.direction = "left"
                elif self.direction == "down":
                    self.rotate("right")
                    self.move("forward")
                    self.direction = "left"
            elif y_diff == 1:
                if self.direction == "left":
                    self.rotate("right")
                    self.move("forward")
                    self.direction = "up"
                elif self.direction == "right":
                    self.rotate("left")
                    self.move("forward")
                    self.direction = "up"
                elif self.direction == "up":
                    self.move("forward")
                elif self.direction == "down":
                    self.move("backward")
            elif y_diff == -1:
                if self.direction == "left":
                    self.rotate("left")
                    self.move("forward")
                    self.direction = "down"
                elif self.direction == "right":
                    self.rotate("right")
                    self.move("forward")
                    self.direction = "down"
                elif self.direction == "up":
                    self.move("backward")
                elif self.direction == "down":
                    self.move("forward")


WALL = math.inf
GATE = "G"
DIRECTIONS = {
    "left": (0, -1), "right": (0, 1), "up": (-1, 0), "down": (1, 0)
}


def set_start_point(x, y):
    global START_POINT
    graph[x][y] = "S"
    START_POINT = (y, x)


def set_target_point(x, y):
    global TARGET_POINT
    graph[x][y] = "T"
    TARGET_POINT = (y, x)


def set_gates(points):
    global GATE, GATES
    for point in points:
        graph[point[0]][point[1]] = GATE
    GATES = points


def create_wall(origin_point, direction):
    global DIRECTIONS

    for i in range(3):
        graph[origin_point[0] + DIRECTIONS[direction][0] *
              i][origin_point[1] + DIRECTIONS[direction][1] * i] = WALL


def is_valid_move(x, y):
    return 0 <= x < 9 and 0 <= y < 9 and graph[y][x] != WALL


def print_colored_grid(grid):
    for row in grid:
        for value in row:
            if value == 0:
                print(Fore.WHITE + Back.WHITE + '██', end='')
            elif value == math.inf:
                print(Fore.RED + Back.RED + '██', end='')
            elif value == 'G':
                print(Fore.GREEN + Back.GREEN + '██', end='')
            elif value == 'S':
                print(Fore.BLUE + Back.BLUE + 'S█', end='')
            elif value == 'T':
                print(Fore.BLUE + Back.BLUE + 'T█', end='')
            elif value == 'X':
                print(Fore.BLACK + Back.BLACK + 'X█', end='')
        print(Back.RESET)


def get_neighbors(node):
    x, y = node
    neighbors = [
        (x + 1, y),
        (x - 1, y),
        (x, y + 1),
        (x, y - 1),
    ]
    return [(x, y) for x, y in neighbors if is_valid_move(x, y)]


def weighted_sum(path, target, desired_length):
    gate_weight = 15  # Adjust this weight as needed
    gate_reward = 0

    for node in path:
        if node in gates:
            gate_reward += gate_weight

    length_penalty = len(path) - desired_length
    length_penalty *= 1 if length_penalty >=0 else 2

    combined_cost = -gate_reward + length_penalty

    return combined_cost

def backtrack_search(current_node, path, desired_length, visited_gates):
    if len(path) > desired_length:
        return None

    if current_node == TARGET_POINT:
        return path

    path.append(current_node)

    valid_neighbors = get_neighbors(current_node)

    # Sort neighbors based on weighted sum
    valid_neighbors.sort(key=lambda neighbor: weighted_sum(
        path, TARGET_POINT, desired_length))

    for neighbor in valid_neighbors:
        if neighbor not in path:
            result = backtrack_search(
                neighbor, path.copy(), desired_length, visited_gates)
            if result:
                return result

    return None


def find_best_path(target_time):
    visited_gates = set()
    visited_gates.add(START_POINT)  # Start from the START_POINT
    return backtrack_search(START_POINT, [], target_time, visited_gates)


gates = [(5, 1), (1, 5), (5, 7)]

if __name__ == "__main__":
    init(autoreset=True)

    graph = [[0 for _ in range(9)] for _ in range(9)]

    # INITIALIZATION
    set_start_point(8, 3)
    set_target_point(1, 3)

    set_gates([(5, 1), (1, 5), (5, 7)])

    create_wall((4, 0), "right")
    create_wall((2, 2), "right")
    create_wall((6, 2), "right")
    create_wall((4, 4), "right")
    create_wall((6, 6), "right")
    create_wall((0, 4), "down")
    create_wall((0, 6), "down")
    create_wall((4, 4), "down")

    TARGET_TIME = 20
    # END INITIALIZATION

    path = find_best_path(TARGET_TIME)

    if path:
        print("Found path:")
        for node in path:
            graph[node[1]][node[0]] = "X"
    else:
        print("No path found.")

    print_colored_grid(graph)
    print("Length of path:", len(path))
    print(path)

    # bot = Robot("outA", "outB", 43, 100, 161, 141, TARGET_TIME)

    # search(graph)
