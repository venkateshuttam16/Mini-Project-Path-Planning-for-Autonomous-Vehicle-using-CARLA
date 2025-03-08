

import random
import glob
import os
import sys
import time
import math
import numpy as np

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla
import heapq

# Define the Waypoint class with a comparison function
class Waypoint:
    def __init__(self, waypoint, cost):
        self.waypoint = waypoint
        self.cost = cost

    def __lt__(self, other):
        return self.cost < other.cost

# Define the heuristic function for A*
def distance_heuristic(start, end):
    return start.transform.location.distance(end.transform.location)

# Define the A* search algorithm with lane change logic
def find_shortest_path_with_lane_change(start, end, world, vehicle):
    open_list = []
    closed_list = set()

    # Initialize the start waypoint
    start_cost = distance_heuristic(start, end)
    start_waypoint = Waypoint(start, start_cost)
    heapq.heappush(open_list, start_waypoint)

    # Keep searching until the end waypoint is found or the open list is empty
    while open_list:
        # Pop the waypoint with the lowest cost from the open list
        current = heapq.heappop(open_list)

        # If the current waypoint is the end waypoint, we've found the shortest path
        if current.waypoint.transform.location.distance(end.transform.location) < 1.0:
            path = [current.waypoint]
            while current.waypoint != start_waypoint.waypoint:
                current = current.parent
                path.append(current.waypoint)
            path.reverse()
            return path

        # Add the current waypoint to the closed list
        closed_list.add(current.waypoint)

        # Check all the neighbors of the current waypoint
        for neighbor in current.waypoint.next(1.0):
            # Check if the neighbor waypoint is blocked by any dynamic obstacles
            if is_waypoint_blocked(neighbor, world, vehicle):
                # Try to find an adjacent main lane if available
                main_lane_waypoint = find_main_lane(neighbor, world, vehicle)
                if main_lane_waypoint:
                    neighbor = main_lane_waypoint
                else:
                    continue

            # Check if the neighbor waypoint has a traffic light
            traffic_lights = world.get_actors().filter('traffic.traffic_light')
            for traffic_light in traffic_lights:
                traffic_light_location = traffic_light.get_location()
                if traffic_light_location.distance(neighbor.transform.location) < 5.0:
                    if traffic_light.get_state() != carla.TrafficLightState.Green:
                        # If the traffic light is not green, wait until it turns green
                        continue

            neighbor_cost = current.cost + neighbor.transform.location.distance(current.waypoint.transform.location)
            neighbor_waypoint = Waypoint(neighbor, neighbor_cost)

            # If the neighbor waypoint is already on the closed list, skip it
            if neighbor in closed_list:
                continue

            # If the neighbor waypoint is not on the open list, add it
            if neighbor_waypoint not in open_list:
                neighbor_waypoint.parent = current
                neighbor_waypoint.cost += distance_heuristic(neighbor, end)
                heapq.heappush(open_list, neighbor_waypoint)

            # If the neighbor waypoint is already on the open list, update its cost if necessary
            else:
                for wp in open_list:
                    if wp.waypoint == neighbor:
                        if neighbor_waypoint.cost < wp.cost:
                            wp.cost = neighbor_waypoint.cost
                            wp.parent = current
                            heapq.heapify(open_list)
                        break

    # If the open list is empty and we haven't found the end waypoint, the path doesn't exist
    return None

def is_waypoint_blocked(waypoint, world, vehicle):
    # Check if the waypoint is blocked by any dynamic obstacles (e.g., vehicles)
    for actor in world.get_actors().filter('vehicle.*'):
        if actor != vehicle:  # Exclude the controlled vehicle
            actor_location = actor.get_location()
            if waypoint.transform.location.distance(actor_location) < 3.0:
                return True
    return False

def find_main_lane(waypoint, world, vehicle):
    # Find the main adjacent lane if available
    right_lane = waypoint.get_right_lane()
    left_lane = waypoint.get_left_lane()

    if right_lane and right_lane.lane_type == carla.LaneType.Driving and not is_waypoint_blocked(right_lane, world, vehicle):
        return right_lane
    elif left_lane and left_lane.lane_type == carla.LaneType.Driving and not is_waypoint_blocked(left_lane, world, vehicle):
        return left_lane
    else:
        return None

def draw_marker(location, color):
    world.debug.draw_point(location, size=0.2, color=color, life_time=6000.0)

def is_traffic_light_red_in_lane(waypoint, world):
    # Check if there is a traffic light and it is red, and if it is in the same lane as the waypoint
    traffic_lights = world.get_actors().filter('traffic.traffic_light')
    for traffic_light in traffic_lights:
        traffic_light_location = traffic_light.get_location()
        if traffic_light_location.distance(waypoint.transform.location) < 7.0:
            if traffic_light.get_state() == carla.TrafficLightState.Red:
                # Check if the traffic light is in the same lane as the waypoint
                waypoint_lane_id = waypoint.lane_id
                traffic_light_waypoint = map.get_waypoint(traffic_light_location)
                traffic_light_lane_id = traffic_light_waypoint.lane_id
                if waypoint_lane_id == traffic_light_lane_id:
                    return True
    return False
def change_lane_smoothly(vehicle, target_waypoint):
    max_angle_diff = 45.0  # Adjust this angle as needed for smoother lane changes
    max_steer_angle = 0.5  # Adjust this steering angle limit
    current_transform = vehicle.get_transform()
    current_rotation = current_transform.rotation.yaw
    target_rotation = math.degrees(math.atan2(target_waypoint.transform.location.y - current_transform.location.y,
                                              target_waypoint.transform.location.x - current_transform.location.x))

    angle_diff = (target_rotation - current_rotation + 180) % 360 - 180
    if abs(angle_diff) > max_angle_diff:
        steer = np.clip(angle_diff, -max_steer_angle, max_steer_angle)
        vehicle.apply_control(carla.VehicleControl(steer=steer))
# Define a function to draw arrows indicating lane changes
def draw_arrow(start_location, end_location, color):
    world.debug.draw_arrow(start_location, end_location, thickness=0.1, arrow_size=0.5, color=color, life_time=2.0)


# Connect to the CARLA server and retrieve the world object
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()

# Get the map and retrieve the transform objects for the start and end locations
map = world.get_map()
start_location = carla.Transform(carla.Location(x=51.0, y=0, z=2.0), carla.Rotation(yaw=0.0))
end_location = carla.Transform(carla.Location(x=94.3, y=109.5, z=2.0), carla.Rotation(yaw=180.0))

# Spawn two waypoint markers at the start and end locations
start_waypoint = map.get_waypoint(start_location.location)
end_waypoint = map.get_waypoint(end_location.location)
start_marker = world.debug.draw_string(start_location.location, 'Start Location', draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=600.0)
end_marker = world.debug.draw_string(end_location.location, 'End Location', draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=600.0)

# Print out the start and end locations for verification
print('Start Location: ({}, {}, {})'.format(start_location.location.x, start_location.location.y, start_location.location.z))
print('End Location: ({}, {}, {})'.format(end_location.location.x, end_location.location.y, end_location.location.z))

# Get the car's blueprint and spawn it at the start location
# Get the blueprint for Audi e-tron
# Get the blueprint for Audi e-tron
# Get the blueprint for Mini Cooper S (replace 'mini' with the correct keyword in your CARLA environment)
blueprint_library = world.get_blueprint_library()

# ... (rest of your code)

# Get the blueprint for Mini Cooper S (replace 'mini' with the correct keyword in your CARLA environment)
mini_bp = blueprint_library.filter('vehicle.mini.cooper_s_2021')[0]

# Set the spawn location for the car
spawn_point = start_location

# Spawn the Mini Cooper S at the specified location
vehicle = world.spawn_actor(mini_bp, spawn_point)


# Get the blueprint of the static car we want to spawn
blueprint_library = world.get_blueprint_library()
car1_bp = blueprint_library.filter('vehicle.*')[0]

# Set the spawn location for the car
spawn_point1 = carla.Transform(carla.Location(x=2.0, y=-56.8, z=0), carla.Rotation())

# Spawn the car at the specified location
car1 = world.spawn_actor(car1_bp, spawn_point1)

# Set the car to be static (i.e. not movable)
car1.set_simulate_physics(False)

# Get the blueprint of the static car we want to spawn
blueprint_library = world.get_blueprint_library()
car2_bp = blueprint_library.filter('vehicle.*')[0]

# Set the spawn location for the car
spawn_point2 = carla.Transform(carla.Location(x=36.2, y=-60.9, z=0), carla.Rotation())

# Spawn the car at the specified location
car2 = world.spawn_actor(car2_bp, spawn_point2)

# Set the car to be static (i.e. not movable)
car2.set_simulate_physics(False)

### Modify the event-driven path generation loop
path = []
current_lane = start_waypoint.lane_type
original_lane = current_lane
current_waypoint = start_waypoint

while True:
    current_location = vehicle.get_location()
    current_waypoint = map.get_waypoint(current_location)

    new_path = find_shortest_path_with_lane_change(current_waypoint, end_waypoint, world, vehicle)

    if new_path is not None:
        path = new_path

    world.debug.draw_string(current_location, '', draw_shadow=False, color=carla.Color(0, 0, 0), life_time=0.1)

    if path:
        for i, waypoint in enumerate(path):
            if i < len(path) - 1:
                if is_waypoint_blocked(path[i], world, vehicle):
                    # Red for blocked path
                    world.debug.draw_line(path[i].transform.location, path[i + 1].transform.location, thickness=0.1, color=carla.Color(r=0, g=0, b=180), life_time=2.0)
                elif path[i] == current_waypoint:
                    # Yellow for lane change
                    world.debug.draw_line(path[i].transform.location, path[i + 1].transform.location, thickness=0.1, color=carla.Color(r=180, g=180, b=0), life_time=2.0)
                    draw_arrow(path[i].transform.location, path[i + 1].transform.location, carla.Color(r=255, g=255, b=0))  # Arrow for lane change visualization
                else:
                    # Green for main path
                    world.debug.draw_line(path[i].transform.location, path[i + 1].transform.location, thickness=0.1, color=carla.Color(r=0, g=180, b=0), life_time=2.0)

            if is_traffic_light_red_in_lane(waypoint, world):
                print("Traffic light is red. Waiting...")
                while is_traffic_light_red_in_lane(waypoint, world):
                    time.sleep(1.0)

            collision_check_distance = 10.0
            collision_check_location = current_location + current_waypoint.transform.get_forward_vector() * collision_check_distance
            collision_check_waypoint = map.get_waypoint(collision_check_location)

            if is_waypoint_blocked(collision_check_waypoint, world, vehicle):
                main_lane_waypoint = find_main_lane(current_waypoint, world, vehicle)
                if main_lane_waypoint:
                    change_lane_smoothly(vehicle, main_lane_waypoint)
                    current_waypoint = main_lane_waypoint
                    draw_marker(main_lane_waypoint.transform.location, carla.Color(r=180, g=180, b=0))  # Yellow for lane change visualization
                    time.sleep(1.0)

            vehicle.set_transform(waypoint.transform)

            if current_location.distance(end_location.location) < 1.0:
                vehicle.destroy()
                print("Car has reached the end location and has been destroyed!")
                sys.exit()

            time.sleep(0.1)
