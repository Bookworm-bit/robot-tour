#!/usr/bin/env python3

from ev3dev2.motor import MoveTank

class Robot:
    def __init__(self, lmp, rmp, wheel_diameter, width, target_time, path):
        self.tank_drive = MoveTank(lmp, rmp)
        self.wheel_diameter = wheel_diameter
        self.width = width
        self.target_time = target_time

    def distance_to_rotations(self, distance):
        return distance / (math.pi * self.wheel_diameter)

    def rotate_ccw(self):
        self.tank_drive.reset()
        self.tank_drive.on_for_rotations(-50, 50, distance_to_rotations(self.width * math.pi / 4), brake=True, block=True)

    def rotate_cw(self):
        self.tank_drive.reset()
        self.tank_drive.on_for_rotations(50, -50, distance_to_rotations(self.width * math.pi / 4), brake=True, block=True)

    def rotate_180(self):
        self.tank_drive.reset()
        self.tank_drive.on_for_rotations(50, -50, distance_to_rotations(self.width * math.pi / 2), brake=True, block=True)

    def move(self):
        square = self.distance_to_rotations(250 - width / 2)
        self.tank_drive.on_for_rotations(50, 50, square, brake=True, block=True)

    def follow(self):
        for i in range(len(path)):
            if path[i] == "ccw":
                self.rotate_ccw()
            elif path[i] == "cw":
                self.rotate_cw()
            elif path[i] == "180":
                self.rotate_180()
            else:
                self.move()

            if path[i] != "move":
                self.tank_drive.on_for_rotations(50, 50, self.distance_to_rotations(width / 2), brake=True, block=True)

if __name__ == "__main__":
    TARGET_TIME = 60

    instructions = [
        "ccw",
        "cw",
        "move"
    ]
    
    bot = Robot("outA", "outB", 43, TARGET_TIME - 5)
    
