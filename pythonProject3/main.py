import math
import numpy as np
import enum

# Simulation paramers
# Grid_size: specifies the number of blocks in which the grid is divided
# area_in_km: specifies the actual size of the grid in km2
# iterations: specifies the number of iterations in the simulation. Note: 2^iterations <= grid_size
# logging: enables or disables logging mode
grid_size = 256
area_in_km = 2.5
iterations = 6
logging = False


# Current specs based on the DJI Mavic 3 drone
class DroneSpecs(enum.Enum):
    speed = 45  # km/h
    max_range = 100  # in km
    max_flight_time = 120  # in min


class ConversionsUnits(enum.Enum):
    # Calculation factor based on ratio of set parameters.
    # Units: km / blocks
    # Note: (km / blocks) * blocks -> km
    km_per_block = area_in_km / grid_size
    
    # Formula to calculate the time it takes to travel one block.
    # Units: 1/m * m/s -> s
    travel_time_block = 1 / (km_per_block * 1000) * (DroneSpecs.speed.value / 3.6)  # Time in seconds
    
    # Assumption is that it takes 1 sec to scout a single block
    research_time_block = 1/10  # time in seconds
    
    # Conversion factor to convert blocks to actual distance
    # Units: 1 / (km / blocks) = block / km
    # note: (blocks / km) * km -> blocks 
    km_to_blocks = (1 / km_per_block)


class Drone:
    def __init__(self, grid, start, boat):
        # Calculate the max distance the drone can fly based on specs
        # Unit: blocks
        self.max_distance = DroneSpecs.max_range.value * ConversionsUnits.km_to_blocks.value
        
        # Specs give the max flight time in minutes which is used here
        # Unit: minutes
        # Note: if not specified make it MAX_VALUE otherwise the simulation fails.
        self.max_time = DroneSpecs.max_flight_time.value
        self.work_area = grid
        self.start_position = start
        self.boat_position = boat
        
        # Keep track of travel distance, search distance and the total traveled distance
        # Units: Blocks
        self.total_travel = 0
        self.search_distance = 0
        self.travel_distance = 0

        # Variable to store the calculated travel time
        # Unit: seconds
        self.travel_time = 0

        # Flag which indicates that the drone is unable to cover the assigned area.
        self.empty = False

    def get_to_start(self):
        # calculate the distance to the assigned starting point. 
        self.travel_distance = abs(self.start_position[0] - self.boat_position[0])

        # Check if drone is able to fly to the starting point and then return.
        if self.max_distance - 2 * self.travel_distance < 0:
            self.empty = True
            return

        # update travel distance
        self.total_travel = self.travel_distance

    def scout_area(self):
        # Calculate the size of the searching area. 
        x = self.work_area.shape
        self.search_distance = x[0] * x[1]

        # Check if drone can cover the area and then return back to the base.
        if self.max_distance - self.search_distance - self.travel_distance < 0:
            self.empty = True
            return

        # Update the total traveled distance.
        self.total_travel = self.total_travel + self.search_distance

    def return_to_base(self):
        # Update the total travel distance
        self.total_travel = self.total_travel + self.travel_distance

    def calc_travel_time(self):
        conv = Conversions()

        # Calculate the total travel time. This is done by calculating the search time and travel time seperately.
        search_t = conv.researchBlocks_to_time(self.search_distance)
        travel_t = 2 * conv.travelBlocks_to_time(self.travel_distance)
        self.travel_time = search_t + travel_t

    # Function that checks if the total travel time is within the limits of the drone.
    def enough_flight_time(self):
        if self.travel_time / 60 > self.max_time:
            return False

        return True

    # Function which runs the full cycle including break checks. This function also sets error codes.
    # Error code -1: Drone can not reach the starting point.
    # Error code -2: The work area is too big for the drone to fully cover.
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
        self.calc_travel_time()


# For the first version it is assumed that n and dn are both multiples of 2.
class Area:
    def __init__(self, n, dn):
        self.grid = np.zeros([n, n])
        self.n = n
        self.drone_number = dn
        self.segments = {}

    # Divides the area in multiple areas which are stored in a dictionary together with the start coordinate for every
    # area.
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

    # Actual work division by creating a drone for every area.
    # This function returns a list containing all the created drones.
    def divide(self):
        boat_pos = (int(self.n / 2), int(self.n / 2))
        result = []
        for i in self.segments:
            result.append(Drone(self.segments[i], i, boat_pos))

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

    # Log function which can be optionally set. When set the function puts out the information of all the created
    # drones. This might be useful for determining drone specs.
    def log(self, count, j):
        x = j.work_area.shape
        size = x[0] * x[1]
        dist = abs(j.start_position[0] - j.boat_position[0])

        print("\ndrone %s" % count)
        print("battery: %s" % j.max_distance, " ,grid size: %s" % size, " ,Travel distance: %s" % (2 * dist))
        if j.total_travel == -1:
            print("Could not reach target area")
        elif j.total_travel == -2:
            print("Work area is too big\n")
        elif not j.enough_flight_time():
            print("Battery life not sufficient")
        else:
            print("Area fully scouted, total traveled distance: %s" % j.total_travel)
            print("Total time: %s" % (j.travel_time), "\n")

    def run_sim(self, area_size, iterations, log):
        print("start of simulation")
        print("Size of area in blocks: %s x %s" % (area_size, area_size))
        print(
            "Size in km: %s x %s" % (self.conv.blocks_to_distance(area_size), self.conv.blocks_to_distance(area_size)))

        # run multiple tests, for every run the number of drones is multiplied by 2
        for i in range(iterations):
            print("\nExperiment %s" % (i + 1))

            # generate 2^i drones and divide the whole grid in work areas.
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
                # Check if drone has enough battery life
                if not j.enough_flight_time():
                    failed = True

                # Check which drone covered the most ground
                if j.travel_time > d.travel_time:
                    d = j

                if log:
                    self.log(count, j)
                count += 1  # Counter for the number of drones

            if not failed:
                print("Time to cover whole area using %s drones: %s = %s minutes" % (2 ** i, d.travel_time, d.travel_time/60))

            else:
                print("Simulation failed")


x = RunModel()
# This function runs the simulation
x.run_sim(grid_size, iterations, logging)
