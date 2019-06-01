import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
import numpy.linalg as LA

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local
from planning_utils_advance import create_grid_and_edges,a_star,heuristic,closest_point,prune_path
import networkx as nx


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection, waypoints=[]):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = waypoints
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # registering all our callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        if len(self.waypoints) > 0:
            self.send_waypoints()
            time.sleep(1)
        else:

            filename = 'colliders.csv'
            # Reading in the data skipping the first two lines.
            # reading lat0, lon0 from colliders into floating point values
            f = open(filename, "r")
            temp = f.read().split('\n')
            lat,lon = temp[0].split(",")

            lat0 = float(lat.strip('lat0'))
            lon0 = float(lon.strip(' lon0 '))

            f.close()

            print(lat0,lon0)

            # setting home position to (lon0, lat0, 0)
            self.set_home_position(lon0, lat0, 0.0)

            # retrieving current global position
            global_position = (self._longitude,self._latitude,self._altitude)

            # converting to current local position using global_to_local()
            local_position = global_to_local(global_position, self.global_home)

            print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                             self.local_position))

            self.landing_transition()
            self.disarming_transition()
            self.manual_transition()

            # Reading in obstacle map
            data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

            # Defining a grid for a particular altitude and safety margin around obstacles
            #grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

            drone_altitude = 5
            safety_distance = 3

            # This is now the routine using Voronoi
            grid, edges, north_offset, east_offset  = create_grid_and_edges(data, drone_altitude, safety_distance)

            grid_start = (-north_offset + int(self.local_position[0]), -east_offset + int(self.local_position[1]))

            # Setting goal as some arbitrary position on the grid
            # adapting to set goal as latitude / longitude position and convert
            goal_local = global_to_local ([-122.395914,37.795267,0],self.global_home)
            #goal_local = global_to_local ([-122.398993,37.792522,0],self.global_home)

            grid_goal = ((-north_offset + int(goal_local[0])),(-east_offset + int(goal_local[1])))
            #grid_goal = (-north_offset + 10, -east_offset + 10)

            G = nx.Graph()
            for e in edges:
                p1 = e[0]
                p2 = e[1]
                dist = LA.norm(np.array(p2) - np.array(p1))
                G.add_edge(p1, p2, weight=dist)



            # Running A* to find a path from start to goal
            print('Local Start and Goal: ', grid_start, grid_goal)
            start_ne_g = closest_point(G, grid_start)
            goal_ne_g = closest_point(G, grid_goal)

            path_, cost = a_star(G, heuristic, start_ne_g, goal_ne_g)
            path = prune_path(grid, path_)

            # Converting path to waypoints
            waypoints = [[int(p[0] + north_offset), int(p[1] + east_offset), TARGET_ALTITUDE, 0] for p in path]

            print(waypoints)

            # Reconnecting to drone and send calculated waypoints
            conn = MavlinkConnection('tcp:{0}:{1}'.format('127.0.0.1', 5760),
                                             timeout=600)
            drone = MotionPlanning(conn, waypoints=waypoints)
            time.sleep(1)
            drone.start()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        while self.in_mission:
            pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=600)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
