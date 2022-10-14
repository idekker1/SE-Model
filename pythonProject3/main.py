import numpy as np

class drone:
    def __init__(self, st, bl, grid, start, boat):
        self.scout_time = st
        self.battery_life = bl
        self.work_area = grid
        self.start_position = start
        self.boat_position = boat

    def total_time(self):
        pass


# For the first version it is assumed that n and dn are both multiples of 2.
class area:
    def __init__(self, n, dn):
        self.grid = np.zeros([n, n])
        self.n = n
        self.drone_number = dn
        self.segments = {}

    def segment(self):
        [k, remainder] = divmod(self.n, self.drone_number)

        mid_y = round(self.n / 2)

    def divide(self):
        count = 0
        result = []
        for i in self.segments:
            boat_pos = str([self.n/2, self.n/2])
            print(self.segments[i])
            #result[count] = drone(1, 100, self.segments[i], i, boat_pos)
            count += 1

        return result


x = area(4, 2)
x.segment()
temp = x.divide()
print(temp)

