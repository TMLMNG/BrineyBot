from map_controller import MapHandler as mph
import geopandas as gpd
from shapely.geometry import Polygon, Point
import matplotlib.pyplot as plt
from math import cos, sin,  pi
from slam_handler import SlamHandler as slamh
import numpy as np
from time import sleep
from path_handler import calculate_trajectory_from_control_path, produce_control_path, perform_dijkstra
from statistics import mean
from gps_controller import GPS_Handler
SIM_LOOP = 500
show_animation = True
r = 40075000

def get_interior_points(bound_x, bound_y, polygon_series, num_points):
    interior_points = []
    traverse_x = abs(min(bound_x) -max(bound_x))/20
    traverse_y = abs(min(bound_y) - max(bound_y))/20
    x = traverse_x+min(bound_x)
    y = traverse_y + min(bound_y)
    latMid = mean(bound_x)
    while x <= max(bound_x):
        while y <= max(bound_y):
            if polygon_series.contains(Point(y, x)).bool():
                #found_y = (r * cos(latMid) * y)#/100
                #found_x = (r * x)#/100
                #interior_points.append([found_y, found_x])
                interior_points.append([y, x])
            y += traverse_y
        y = traverse_y + min(bound_y)
        x += traverse_x
    output_points = np.array(interior_points)
    return output_points
def normalize(array):
    min_val = min(array[:, 1])
    max_val = max(array[:, 1])
    return_y = [((input_y - min_val) / (max_val - min_val)) for input_y in array[:, 1]]
    min_val = min(array[:, 0])
    max_val = max(array[:, 0])
    return_x = [(input_x - min_val) / (max_val - min_val) for input_x in array[:, 0]]
    return np.array([return_x, return_y])


def make_meters_map():
    l = 12.0
    w = 23.0
    longs = [0, l/2, l, l, l, l, l, l-(l*(1/3)*(1/2)), l-(l*1/3), l-(l*1/3), l/2-2, 1, 0, 0]
    lats = [0, 0, 0, -w/4, -w/2, -w*3/4, -w, -w, -w, -w*3/4, -w/2, -w/2+.5, -w/4, -w/8]
    return longs, lats
test_map = mph()
test_slam = slamh()
#test_gps_controller = GPS_Handler()
#bounds_lats = [42.333999, 42.333950, 42.333964, 42.333984, 42.333987, 42.333976, 42.333952, 42.333972,
#        42.334071, 42.334037, 42.334013]
bounds_lats = [42.333999, 42.333950, 42.333964, 42.333984, 42.333987, 42.333976, 42.333952, 42.333972,
        42.334071, 42.334037, 42.334013]
bounds_longs = [-71.535223, -71.535205, -71.535184, -71.535098, -71.535021, -71.534969, -71.534936, -71.534858,
         -71.534898, -71.535003, -71.535123]

bounds_longs, bounds_lats = make_meters_map()
point_arr = []
count = 0
for coords in zip(bounds_longs, bounds_lats):
    point_arr.append(Point(coords[0], coords[1]))
    test_slam.update_landmarks([coords])
    if count == 3:
        break
polygon = Polygon([[p.x, p.y] for p in point_arr])
pseries = gpd.GeoSeries(polygon)

test_map.map = gpd.GeoSeries(polygon)
boundary_points = test_map.get_map_boundary()
waypoints = test_map.get_interior_points( )
obstacles = []
latMid = mean(bounds_lats)

# for point in boundary_points:
#     # x = (r * cos(latMid) * point.geometry.x)#/100
#     # y = (r * point.geometry.y)#/100
#     obstacles.append([point.geometry.x, point.geometry.y])
obstacles = np.array(obstacles)
#test_map.get_obstacles_numpy_array()
# test_map.plot_map()

# Check moving through map
longs = [longitude for longitude in map(lambda x: x / 100000000.0, range(-7153492900, -7153494300, -123))]
lats = [latitude for latitude in map(lambda x: x / 100000000.0, range(4233398400, 4233399800, 75))]
orientations = [0, 0, 0, pi / 12, pi / 12, pi / 12, (pi / 12) * 2, (pi / 12) * 2, (pi / 12) * 2, (pi / 12) * 3,
                (pi / 12) * 3, (pi / 12) * 3]
#test_map.update_spray_to_map(longs, lats, orientations)
#test_map.plot_map()
bot_prop = {}
bot_prop['current_speed'] = 10.0 / 3.6  # current speed [m/s]
bot_prop['lat_pose']= 2.0  # current lateral position [m]
bot_prop['lat_speed'] = 0.0  # current lateral speed [m/s]
bot_prop['lat_accel'] = 0.0  # current lateral acceleration [m/s]
bot_prop['course_pos'] = 0.0  # current course position
#illegal_steps = test_map.check_eligible_move(zip(longs, lats))
#test_slam.first_reading([longs[0], lats[0]])
# r = 40075000
# latMid = mean(bounds_lats)
# x = [r*cos(latMid)*long for long in longs]
# y = [r*lat for lat in lats]
#tx, ty, tyaw, tc, csp = produce_control_path([x, y[0:len(x)]])
#waypoints = normalize(waypoints)
#obstacles = normalize(obstacles)
next_travel_way_points = [[[.5, -1], [11.5, -1]], [[11.5, -1], [11.5, -22.5]], [[11.5, -22.5], [11.5, -22.5]],
                          [[10.5, -22.5], [10.5, -2]], [[10.5, -2], [.5, -2]], [[.5, -2], [.5, -3]], [[.5, -3], [9.5, -3]],
                          [[9.5, -3], [9.5, -22.5]], [[9.5, -22.5], [8.5, -22.5]], [[8.5, -22.5], [8.5, -4]],
                          [[8.5, -4], [.5, -4]], [[.5, -4], [.5, -5]], [[.5, -5], [7.5, -5]], [[7.5, -5], [7.5, -15.5]],
                          [[7.5, -15.5], [1.35, -10]], [[1.35, -10], [1, -6]],
                          [[1, -6], [6.5, -6]], [[6.5, -6], [6.5, -13]], [[6.5, -13], [1.5, -9]], [[1.5, -9], [5.5, -9]],
                          [[5.5, -9], [5.5, -11]], [[5.5, -11], [1.5, -7]], [[1.5, -7], [6, -7]], [[6, -7], [6, -8]], [[6, -8], [1.5, -8]]]

for i in range(0, len(next_travel_way_points)):
    velocity = 1
    # new_gps = np.zeros((3, 1))
    # new_gps[0, 0] = longs[i]
    # new_gps[1, 0] = lats[i]
    # new_gps[2, 0] = orientations[i]
    #When we have bot on
    #test_gps_controller.update_gps()
    #next_travel_way_point = test_map.neareast_unvisted_waypoint(test_gps_controller.current_readout)
    next_travel_way_point = next_travel_way_points[i]
    #next_travel_way_point = test_map.neareast_unvisted_waypoint(current_point)
    rx, ry = perform_dijkstra(next_travel_way_point, boundary_points)#test_map.bot_dim)
    plt.cla()
    plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])

    plt.plot(boundary_points[:, 0], boundary_points[:, 1], 'xk', label='Obstacles/Bounds')
    plt.plot(next_travel_way_point[0][0], next_travel_way_point[0][1], '*c', label='Bot')
    plt.plot(rx, ry, "-r", label='Path To Next Waypoint')
    plt.title('Bot in Motion')
    plt.grid(True)
    traverse_count = 0
    for coord in zip(reversed(rx), reversed(ry)):
        if i == 9:
            if traverse_count == 5:
                plt.plot(coord[0], coord[1], '*c', label='Bot')
                plt.plot(coord[0], coord[1]+.5, 'xk')
                plt.title('Obstacle Detected!')
                plt.pause(5)
                next_travel_way_point = [coord, next_travel_way_points[i][1]]
                boundary_points = np.concatenate((boundary_points, np.array([np.array([[coord[0]], [coord[1]+.5]])])))
                rx, ry = perform_dijkstra(next_travel_way_point, boundary_points)
                plt.cla()
                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])

                plt.plot(boundary_points[:, 0], boundary_points[:, 1], 'xk', label='Obstacles/Bounds')
                plt.plot(next_travel_way_point[0][0], next_travel_way_point[0][1], '*c', label='Bot')
                plt.plot(rx, ry, "-r", label='Path To Next Waypoint')
                plt.title('Bot in Motion')
                plt.grid(True)
                for coord in zip(reversed(rx), reversed(ry)):
                    plt.plot(coord[0], coord[1], '*c', label='Bot')
                    plt.pause(0.01)
                break
            traverse_count+=1
        else:
            plt.plot(coord[0], coord[1], '*c', label='Bot')
            plt.pause(0.01)
    test_map.update_spray_to_map(rx, ry, [0] * len(rx))
    current_point = [next_travel_way_point[1][0], next_travel_way_point[1][1]]

next_travel_way_point = [current_point, next_travel_way_points[0][0]]
rx, ry = perform_dijkstra(next_travel_way_point, boundary_points)#test_map.bot_dim)
plt.cla()
plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
plt.title('Heading Home')
plt.plot(boundary_points[:, 0], boundary_points[:, 1], 'xk', label='Obstacles/Bounds')
plt.plot(next_travel_way_point[0][0], next_travel_way_point[0][1], '*c', label='Bot')
plt.plot(rx, ry, "-r", label='Path To Next Waypoint')
plt.grid(True)
for coord in zip(reversed(rx), reversed(ry)):
    plt.plot(coord[0], coord[1], '*c', label='Bot')
    plt.pause(0.0001)
plt.title('Complete!')
test_map.plot_map()

input('')
#test_slam.plot_graph()