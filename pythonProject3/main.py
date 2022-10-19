import math
import numpy as np
import enum


class DroneSpecs(enum.Enum):
    speed = 30  # km/h
    max_range = 200  # in blocks


class ConversionsUnits(enum.Enum):
    km_per_block = 10/256
    travel_time_block = 1/DroneSpecs.speed.value
    research_time_block = 15


class Drone:
    def __init__(self, st, bl, grid, start, boat):
        self.scout_time = st
        self.battery_life = bl
        self.work_area = grid
        self.start_position = start
        self.boat_position = boat

        self.total_travel = 0
        self.search_distance = 0
        self.travel_distance = 0
        self.empty = False

    def get_to_start(self):
        self.travel_distance = abs(self.start_position[0] - self.boat_position[0])

        # Check if drone can reach and return the starting point
        if self.battery_life - 2 * self.travel_distance < 0:
            self.empty = True
            return

        # update travel distance
        self.total_travel = self.travel_distance

    def scout_area(self):
        x = self.work_area.shape
        size = x[0] * x[1]

        if self.battery_life - size - self.travel_distance < 0:
            self.empty = True
            return

        self.total_travel = self.total_travel + size

    def return_to_base(self):
        self.total_travel = self.total_travel + self.travel_distance

    def full_scout(self):
        self.get_to_start()
        if self.empty:
            self.total_travel = -1
            return

        self.scout_area()
        if self.empty:
            self.total_travel = -2
            return

        self.return_to_base()


# For the first version it is assumed that n and dn are both multiples of 2.
class Area:
    def __init__(self, n, dn):
        self.grid = np.zeros([n, n])
        self.n = n
        self.drone_number = dn
        self.segments = {}

    def segment(self):
        [stroke_size, remainder] = divmod(self.n, self.drone_number)

        if (remainder != 0):
            exit("n not divisible by number of drones")

        count = 0
        while count < self.drone_number:
            mat = np.zeros([self.n, stroke_size])
            coordinate = (count * stroke_size, int(math.ceil(self.n / 2)))
            self.segments[coordinate] = mat
            count += 1

    def divide(self):
        boat_pos = (int(self.n / 2), int(self.n / 2))
        result = []
        for i in self.segments:
            result.append(Drone(1, DroneSpecs.max_range.value, self.segments[i], i, boat_pos))

        return result


class Conversions:
    def blocks_to_distance(self, blocks):
        return blocks * ConversionsUnits.km_per_block.value

    def travelBlocks_to_time(self, blocks):
        return blocks * ConversionsUnits.travel_time_block.value

    def researchBlocks_to_time(self, blocks):
        return blocks * ConversionsUnits.research_time_block.value


class RunModel:
    def __init__(self):
        self.conv = Conversions()

    def log(self, count, j):
        x = j.work_area.shape
        size = x[0] * x[1]
        dist = abs(j.start_position[0] - j.boat_position[0])

        print("\ndrone %s" % count)
        print("battery: %s" % j.battery_life, " ,grid size: %s" % size, " ,Travel distance: %s" % (2 * dist))
        if j.total_travel == -1:
            print("Could not reach target area")
        elif j.total_travel == -2:
            print("Work area is too big\n")
        else:
            print("Area fully scouted, total traveled distance: %s" % j.total_travel, "\n")
    def run_sim(self, area_size, iterations, log):
        print("start of simulation")
        print("Size of area in blocks: %s x %s" % (area_size, area_size))
        print("Size in km: %s x %s" % (self.conv.blocks_to_distance(area_size),self.conv.blocks_to_distance(area_size)))


        # Number of tests that are executed
        for i in range(iterations):
            print("\nExperiment %s"% i)

            # generate 2^i drones
            area = Area(area_size, 2 ** i)
            area.segment()
            drones = area.divide()

            # set simulation parameters
            d = drones[0]
            failed = False
            count = 0

            for j in drones:
                j.full_scout()
                # Check if every drone was able to scout its area
                if j.total_travel < 0:
                    failed = True
                    continue
                # Check which drone covered the most ground
                if j.total_travel > d.total_travel:
                    d = j

                if log:
                    self.log(count, j)
                count += 1  # Counter for the number of drones

            if not failed:
                time = self.conv.researchBlocks_to_time(d.search_distance) + 2 * self.conv.travelBlocks_to_time(d.travel_distance)
                print("Time to cover whole area using %s drones: %s"%(2 ** i, time))

            else:
                print("Simulation failed")


x = RunModel()
x.run_sim(16, 4, True)
