#!/usr/bin/env python3

import ev3dev2.motor as motor
import math
import heapq

class Robot:
    def __init__(self, left_motor_port, right_motor_port, wheel_diameter, target_time):
        self.tank_drive = motor.MoveTank(left_motor_port, right_motor_port)
        self.wheel_diameter = wheel_diameter
        self.direction = "up"
        self.target_time = int(target_time / (250 / (math.pi * wheel_diameter * (8 / 3))))
        self.path = None

    def distance_to_rotations(self, distance):
        return distance / (math.pi * self.wheel_diameter)

    def rotate(self, direction):
        turn = 1 if direction == "right" else -1

        self.direction = 
        self.tank_drive.reset()
        self.tank_drive.on_for_rotations(100 * turn, 0, 1.5, brake=True, block=True)
        self.tank_drive.reset()
        self.tank_drive.on_for_rotations(0, -100 * turn, 1.5, brake=True, block=True)

    def move(self):
        square = self.distance_to_rotations(250)
        self.tank_drive.on_for_rotations(100, 100, square, brake=True, block=True)

    def follow_path(self):
        move = self.distance_to_rotations(125)
        self.tank_drive.on_for_rotations(100, 100, move, brake=True, block=True)

        for i in range(len(self.path) - 1):
            current_node = self.path[i]
            next_node = self.path[i + 1]

            absolute_direction_to_turn = None
            for side in DIRECTIONS.keys():
                if current_node[0] + DIRECTIONS[side][0] == next_node[0] and current_node[1] + DIRECTIONS[side][1] == next_node[1]:
                    absolute_direction_to_turn = side
            
            directions = ["up", "right", "down", "left"]
            if self.direction == absolute_direction_to_turn:
                self.move()
            else:
                if abs(self.d)


    def weighted_sum(self, path):
        desired_length = self.target_time

        visited_gates = [gate for gate in GATES if gate in path]

        gate_reward = 1.5 * len(visited_gates)

        length_penalty = max(2 * (desired_length - len(path)), 0)

        combined_cost = -gate_reward + length_penalty + len(path)
        return combined_cost

    def search(self, start, target):
        open_list = []
        heapq.heappush(open_list, (0, start, [start]))

        while open_list:
            _, current, path = heapq.heappop(open_list)

            if current == target:
                self.path = path

                return self.follow_path()

            if len(path) >= self.target_time * 1.3:
                continue

            for neighbor in get_neighbors(current):
                if neighbor not in path:
                    new_path = path + [neighbor]
                    priority = self.weighted_sum(new_path)

                    heapq.heappush(open_list, (priority, neighbor, new_path))
        return None


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


def get_neighbors(node):
    x, y = node
    neighbors = [
        (x + 1, y),
        (x - 1, y),
        (x, y + 1),
        (x, y - 1),
    ]
    return [(x, y) for x, y in neighbors if is_valid_move(x, y)]


def manhattan_distance(point1, point2):
    return abs(point1[0] - point2[0]) + abs(point1[1] - point2[1])


if __name__ == "__main__":
    graph = [[0 for _ in range(9)] for _ in range(9)]

    # INITIALIZATION
    set_start_point(8, 6)
    set_target_point(1, 4)

    set_gates([(6, 3), (3, 7)])

    create_wall((6, 0), "right")
    create_wall((6, 2), "up")
    create_wall((4, 2), "right")
    create_wall((8, 4), "up")
    create_wall((6, 4), "right")
    create_wall((2, 4), "right")
    create_wall((2, 6), "right")
    create_wall((0, 2), "down")

    TARGET_TIME = 60

    bot = Robot("outA", "outB", 43, TARGET_TIME - 5)
    bot.search(START_POINT, TARGET_POINT)
