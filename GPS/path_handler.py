from PythonRobotics.PathPlanning.Dijkstra.dijkstra import *
from PythonRobotics.PathPlanning.FrenetOptimalTrajectory.frenet_optimal_trajectory import *


def new_coordinate_plane(bot_path, obstacles):
    no_decimal_coord_multiplier = 100000000
    zero_point_x =no_decimal_coord_multiplier * min(obstacles[:, 0])[0]
    zero_point_y =no_decimal_coord_multiplier *  min(obstacles[:, 1])[0]
    sx = round(bot_path[0][0]*no_decimal_coord_multiplier - zero_point_x)
    sy = round(bot_path[0][1]*no_decimal_coord_multiplier - zero_point_y)
    gx = round(bot_path[1][0] * no_decimal_coord_multiplier - zero_point_x)
    gy = round(bot_path[1][1] * no_decimal_coord_multiplier - zero_point_y)
    output_obs_x = [round(val[0][0]*no_decimal_coord_multiplier - zero_point_x) for val in obstacles]
    output_obs_y = [round(val[0][1]*no_decimal_coord_multiplier - zero_point_y) for val in obstacles]
    return sx, sy, gx, gy, output_obs_x, output_obs_y


def perform_dijkstra(bot_path, obstacles):
    # start and goal position
    #sx, sy, gx, gy, x, y = new_coordinate_plane(bot_path, obstacles)
    sx = round(bot_path[0][0]*1000)  # [mm]
    sy = round(bot_path[0][1]*1000)# [mm]
    gx = round(bot_path[1][0]*1000)# [mm]
    gy = round(bot_path[1][1]*1000)  # [mm]
    print(sx, sy, gx, gy)
    grid_size = .5*1000  # [mm]
    robot_radius = .5*1000  # [mm]
    x = [round(val[0][0]*1000) for val in obstacles]
    y = [round(val[1][0]*1000) for val in obstacles]
    trashcan_x = [11.7, 11.9, 11.9, 11.7, 4, 4.2, 4.2, 4]
    trashcan_y = [-17, -17, -17.5, -17.5, -4, -4, -4.2, -4.2]
    for obstacle in zip(trashcan_x, trashcan_y):
        x.append(obstacle[0]*1000)
        y.append(obstacle[1]*1000)
    plt.plot(x, y, 'xk')
    dijkstra = Dijkstra(x, y, grid_size, robot_radius)
    rx, ry = dijkstra.planning(sx, sy, gx, gy)
    rx = [val/1000 for val in rx]
    ry = [val/1000 for val in ry]
    return rx, ry
# get cub spline path from
def produce_control_path(waypoints):
    waypoints = np.array(waypoints)
    x = waypoints[:, 0]
    y = waypoints[:, 1]
    tx, ty, tyaw, tc, csp = generate_target_course(x, y)
    return tx, ty, tyaw, tc, csp
# Optimal Trajectory in a Frenet Frame
def calculate_trajectory_from_control_path(control_path, obstacles, bot_prop, csp):

    # initial state
    c_speed = bot_prop['current_speed']  # current speed [m/s]
    c_d = bot_prop['lat_pose']  # current lateral position [m]
    c_d_d = bot_prop['lat_speed']  # current lateral speed [m/s]
    c_d_dd = bot_prop['lat_accel']  # current lateral acceleration [m/s]
    s0 = bot_prop['course_pos']  # current course position
    path = frenet_optimal_planning(
        csp, s0, c_speed, c_d, c_d_d, c_d_dd, obstacles)

    s0 = path.s[1]
    c_d = path.d[1]
    c_d_d = path.d_d[1]
    c_d_dd = path.d_dd[1]
    c_speed = path.s_d[1]

    output_bot_prop = {}
    output_bot_prop['current_speed'] = c_speed  # current speed [m/s]
    output_bot_prop['lat_pose'] = c_d # current lateral position [m]
    output_bot_prop['lat_speed'] = c_d_d  # current lateral speed [m/s]
    output_bot_prop['lat_accel'] = c_d_dd  # current lateral acceleration [m/s]
    output_bot_prop['course_pos'] = s0  # current course position
    return False, output_bot_prop, path
def main():
    x = [-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0]
    y = [0.7, -6, 5, 6.5, 0.0, 5.0, -2.0]
    ds = 0.1  # , csp[m] distance of each intepolated points
    x, y, yaw, c = produce_control_path([x, y], ds)

if __name__ == '__main__':
    main()