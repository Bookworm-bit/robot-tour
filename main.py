#!/usr/bin/env micropython

from ev3dev2.motor import MoveTank
from math import pi

class Robot:
    def __init__(self, lmp, rmp, width, target_time, path):
        self.tank_drive = MoveTank(lmp, rmp)
        self.width = width

        total = 0
        for elem in path:
            if elem == "cw" or elem == "ccw":
                total += self.distance_to_rotations(self.width * pi / 4 + width)
            elif elem == "180":
                total += self.distance_to_rotations(self.width * pi / 2 + width)
            else:
                total += self.distance_to_rotations(250)
        
        self.speed = min(max(-100, total * 60 / target_time), 100) # rotations per minute
        self.path = path

    def distance_to_rotations(self, distance):
        return distance / (140)

    def rotate_ccw(self):
        self.tank_drive.reset()
        self.tank_drive.on_for_rotations(-self.speed, self.speed, self.distance_to_rotations(self.width*(pi/4 + pi/12.25)), brake=True, block=True)

    def rotate_cw(self):
        self.tank_drive.reset()
        self.tank_drive.on_for_rotations(self.speed, -self.speed, self.distance_to_rotations(self.width*(pi/4 + pi/12.25)), brake=True, block=True)

    def rotate_180(self):
        self.tank_drive.reset()
        self.tank_drive.on_for_rotations(self.speed, -self.speed, self.distance_to_rotations(self.width*(pi/2 + pi/6.5)), brake=True, block=True)

    def move(self):
        self.tank_drive.reset()
        square = self.distance_to_rotations(250)
        self.tank_drive.on_for_rotations(self.speed, self.speed, square, brake=True, block=True)

    def follow(self):
        for i in range(len(self.path)):
            if self.path[i] != "move":
                self.tank_drive.reset()
                self.tank_drive.on_for_rotations(-self.speed, -self.speed, self.distance_to_rotations(self.width / 2), brake=True, block=True)

            if self.path[i] == "ccw":
                self.rotate_ccw()
            elif self.path[i] == "cw":
                self.rotate_cw()
            elif self.path[i] == "180":
                self.rotate_180()
            else:
                self.move()
            
            if self.path[i] != "move":
                self.tank_drive.reset()
                self.tank_drive.on_for_rotations(self.speed, self.speed, self.distance_to_rotations(self.width / 2), brake=True, block=True)


if __name__ == "__main__":
    TARGET_TIME = 15

    instructions = [
        "180",
        "move",
        "cw",
        "move",
        "ccw",
        "move"
    ]

    bot = Robot("outA", "outB", 160, TARGET_TIME - 3, instructions)
    bot.follow()
    
