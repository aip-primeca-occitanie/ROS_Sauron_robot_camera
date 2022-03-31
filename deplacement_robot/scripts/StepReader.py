#!/usr/bin/env python

import re

class StepReader:
    
    def __init__(self, file_path):
        f = open(file_path, 'r')
        data = f.read()
        f.close()

        self.cylinders_args = []
        self.cylinders = []
        self.other = {}

        self.read_data(data)

    def create_cylinders(self):
        for args in self.cylinders_args:
            s = args.split(",")
            rayon = float(s[2])

            Axis2P3D = self.other[s[1]]
            s1 = Axis2P3D.split(",")

            coord = self.other[s1[1]]
            c = re.split("\(|\)|,",coord)
            x = float(c[2])
            y = float(c[3])
            z = float(c[4])

            direction = self.other[s1[2]]
            d = re.split("\(|\)|,", direction)
            dx = float(d[2])
            dy = float(d[3])
            dz = float(d[4])

            cylender = Cylender(rayon, (x,y,z), (dx,dy,dz))

            self.cylinders.append(cylender)

    def getCylinders(self) :
        return self.cylinders
    
    def read_data(self, data):
        lines = data.splitlines()

        for l in lines:
            self.read_line(l)
            
        self.create_cylinders()

    def read_line(self, l):
        s = l.split("=")

        if len(s) == 1:
            return

        id = s[0]
        s = re.split("\(|\)", s[1])

        if s[0] == 'CYLINDRICAL_SURFACE':
            self.cylinders_args.append(s[1])
        else :
            self.other[id] = ','.join(s[1:-1])

class Cylender:
    def __init__(self, rayon, position ,direction):
        self.rayon = rayon
        self.position = position
        self.direction = direction

if __name__ == "__main__":
    s = StepReader("/home/alexandre/Plaque_1.stp")