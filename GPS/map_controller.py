import geopandas as gpd
from shapely.geometry import Polygon, Point, MultiPolygon
import matplotlib.pyplot as plt
from math import cos, sin,  pi, sqrt
import numpy as np
from statistics import mean


class MapHandler:
    def __init__(self, map_path='', bot_dim=''):
        self.map_path = map_path
        if self.map_path:
            self.map = gpd.read_file(map_path)
        self.visited_areas = []
        self.obstacle_polygons = None
        self.last_spray_pos = None
        self.origin = []
        self.boundary_resolution = 500
        self.boundary = []
        if not bot_dim:
            self.bot_dim = {'height': 10, 'radius': .5}

    def have_we_sprayed(self, lat, long):
        spray_point = Point(lat, long)
        for area_to_check in self.visited_areas:
            if area_to_check.contains(spray_point):
                return True
        return False

    def get_obstacles_numpy_array(self):
        #return bounds
        # return obstacles
        # return visited areas
        pass

    def convert_lat_long_x_y(self, longs, lats):
        r = 40075000
        latMid = mean(self.bounds_lats)
        x = [r * cos(latMid) * long for long in longs]
        y = [r * lat for lat in lats]
        return x, y

    def update_spray_to_map(self, longs, lats, orientations):
        if not self.origin:
            self.origin = (longs[0], lats[0])
        new_areas_lat_longs = []
        if len(lats) < len(longs):
            longs = longs[0:len(lats)]
        elif len(lats) > len(longs):
            lats = lats[0:len(longs)]
        for bot_pos in zip(*[longs, lats, orientations]):
            R = 6378137
            dLat = self.bot_dim['radius']  # radius / R
            dLon = self.bot_dim['radius']  # radius / (R * cos(pi * center_point[0] / 180))
            center_point = (bot_pos[0], bot_pos[1])
            new_areas_lat_longs.append(self.rotate((center_point[0]+dLon, center_point[1]+dLat), bot_pos[2]))
            #new_areas_lat_longs.append(self.rotate((center_point[0]+dLon, center_point[1]), bot_pos[2]))
            new_areas_lat_longs.append(self.rotate((center_point[0]+dLon, center_point[1]-dLat), bot_pos[2]))
            #new_areas_lat_longs.append(self.rotate((center_point[0], center_point[1]-dLat), bot_pos[2]))
            new_areas_lat_longs.append(self.rotate((center_point[0]-dLon, center_point[1]-dLat), bot_pos[2]))
            #new_areas_lat_longs.append(self.rotate((center_point[0]-dLon, center_point[1]), bot_pos[2]))
            new_areas_lat_longs.append(self.rotate((center_point[0]-dLon, center_point[1]+dLat), bot_pos[2]))
            #new_areas_lat_longs.append(self.rotate((center_point[0], center_point[1]+dLat), bot_pos[2]))
        self.visited_areas.append(Polygon(new_areas_lat_longs))

    def rotate(self, point, angle):
        """
        Rotate a point counterclockwise by a given angle around a given origin.

        The angle should be given in radians.
        """
        ox, oy = self.origin
        px, py = point
        qx = ox + cos(angle) * (px - ox) - sin(angle) * (py - oy)
        qy = oy + sin(angle) * (px - ox) + cos(angle) * (py - oy)
        return qx, qy

    def load_map(self):
        pass
    def check_eligible_move(self, moves):
        illegal_steps = []
        for step in moves:
            step_point = Point(step)
            if not self.map.contains(step_point)[0]: #return illegal steps
                #add a cost for dikstra
                illegal_steps.append((step, 9999))
            for visit_spot in self.visited_areas:
                if visit_spot.contains(step):
                    illegal_steps.append((step, 5))
            #if self.obstacles.contains():
            #if step is obstacle report false
    def report_obstacle(self):
        pass
    def get_map_stats(self):
        pass
    def plot_map(self):
        # plot spray/non-spray
        # plot obstacles perm and reported

        if self.visited_areas:
            visited_polys = gpd.GeoSeries(MultiPolygon(self.visited_areas))
            visited_gf = gpd.GeoDataFrame({'geometry': visited_polys, 'visited_gf': [1]})
            full_map_gf = gpd.GeoDataFrame({'geometry': self.map, 'full_map_gf': [1]})
            frame_map = gpd.overlay(full_map_gf, visited_gf, how='difference')
            come_together = gpd.overlay(full_map_gf, visited_gf, how='intersection')
            percent_covered = come_together.area/full_map_gf.area*100
            ax = frame_map.plot(cmap='tab10')
            visited_polys.plot(ax=ax, facecolor='red', edgecolor='r')
            #self.map.plot(ax=ax, facecolor='blue', edgecolor='k')
        else:
            percent_covered = 0
            self.map.plot()
        #obstacles_gf = gpd.GeoDataFrame({'geometry': self.obstacles, 'obstacles_gf': [1]})
        #self.obstacles.plot(ax=ax, facecolor='green', edgecolor='k')
        plt.title('Spray Coverage %d' % percent_covered)
        plt.show()
        plt.pause(2)

    def get_map_boundary(self):
        boundary = [self.map.exterior.interpolate(i / float(self.boundary_resolution - 1), normalized=True) for i in range(self.boundary_resolution)]
        for point in boundary:
            # x = (r * cos(latMid) * point.geometry.x)#/100
            # y = (r * point.geometry.y)#/100
            self.boundary.append([point.geometry.x, point.geometry.y])
        self.boundary = np.array(self.boundary)
        return self.boundary

    def get_interior_points(self):
        interior_points = []

        traverse_x = abs(min(self.boundary[:, 0]) - max(self.boundary[:, 0])) / 50#self.boundary_resolution
        traverse_y = abs(min(self.boundary[:, 1]) - max(self.boundary[:, 1])) / 50#self.boundary_resolution
        x = traverse_x + min(self.boundary[:, 0])
        y = traverse_y + min(self.boundary[:, 1])
        #latMid = mean(bound_x)
        while x <= max(self.boundary[:, 0]):
            while y <= max(self.boundary[:, 1]):
                if self.map.contains(Point(x, y)).bool():
                    # found_y = (r * cos(latMid) * y)#/100
                    # found_x = (r * x)#/100
                    # interior_points.append([found_y, found_x])
                    interior_points.append([x[0], y[0]])
                y += traverse_y
            y = traverse_y + min(self.boundary[:, 1])
            x += traverse_x
        self.waypoints = np.array(interior_points)
        return self.waypoints

    def neareast_unvisted_waypoint(self, current_position):
        minDist = 9999999999999999999
        for i in range(0, self.waypoints.shape[0]-1):
                p = self.waypoints[i]
                abs_distance = sqrt((p[0]-current_position[0]) ** 2 + (p[1]-current_position[1]) ** 2)
                if abs_distance < minDist and not self.have_we_sprayed(p[0], p[1]):
                    minDist = abs_distance
                    closestPoint = [p[0], p[1]]
                    min_index = i
        self.waypoints[min_index][0] = 999999999999999999
        self.waypoints[min_index][1] = 999999999999999999
        return [current_position, closestPoint]

if __name__ == '__main__':
    #polygon_geom = Polygon([(42.333951, -71.535205), (42.333995, -71.535221), (42.334062, -71.534897),
    #                        (48.853033, -71.534865), (42.333972, -71.534947),
    #                        (42.333950, -71.534989), (42.333951, -71.535205)])
    # unittest
    test_map = MapHandler()
    lats = [42.333999, 42.333950, 42.333964, 42.333984, 42.333987, 42.333976, 42.333952, 42.333972,
            42.334071, 42.334037, 42.334013]
    longs =[-71.535223, -71.535205, -71.535184,-71.535098, -71.535021, -71.534969, -71.534936, -71.534858,
            -71.534898, -71.535003, -71.535123]
    point_arr = []
    for coords in zip(longs, lats):
        point_arr.append(Point(coords[0], coords[1]))
    polygon = Polygon([[p.x, p.y] for p in point_arr])
    test_map.map = gpd.GeoSeries(polygon)
    #test_map.plot_map()

    # Check moving through map
    longs = [longitude for longitude in map(lambda x: x / 100000000.0, range(-7153492900, -7153494300, -123))]
    lats = [latitude for latitude in map(lambda x: x / 100000000.0, range(4233398400, 4233399800, 75))]
    orientations = [0, 0, 0, pi/12, pi/12, pi/12, (pi/12)*2, (pi/12)*2,(pi/12)*2, (pi/12)*3, (pi/12)*3, (pi/12)*3]
    #lats = range(-71.534929, -71.53494300, -.00000123)
    #longs = range(42.333984, 42.333998,   .00000075)
    illegal_steps = test_map.check_eligible_move(zip(longs, lats))
    print(illegal_steps)
    for center_point in zip(longs, lats):
        bot_radius = .0000005
        spray_flag = test_map.have_we_sprayed(center_point[0], center_point[1])
    test_map.update_spray_to_map(longs, lats, orientations)

    test_map.plot_map()
    #check what an illegimate move gets reported

    #check obstacle