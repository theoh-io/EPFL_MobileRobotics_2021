#!pip install pyvisgraph
#!pip install geopandas
#!pip install matplotlib

import pyvisgraph as vg
import math
import geopandas as gpd
from geopandas import GeoSeries
from shapely.geometry import Polygon, Point, LineString

# Set of random polygons TO BE CHANGED 
p1 = Polygon([(1, 1), (2, 1), (2, 2), (1, 2)])
p2 = Polygon([(3, 0), (4, 0), (5, 2), (5, 4), (3, 3)])
p3 = Polygon([(6, 6), (5, 5), (6, 5)])

# Geometric graph of the obstacles as given by the vision analysis
g = GeoSeries([p1, p2, p3])
print(g)
g.plot()

# Geometric graph of the obstacles with the margin
margin=0.1
g=g.buffer(margin,join_style=2) 
print(g)
g.plot()

# Visibility graph created from the geometric graph
polygons = []
for poly in g:
    print(poly)
    x, y = poly.exterior.coords.xy
    polygon_vg = []
    for i in range(len(x)):
        polygon_vg.append(vg.Point(x[i],y[i]))
    polygons.append(polygon_vg)

visgraph = vg.VisGraph()
visgraph.build(polygons)
visgraph.save('graph_test')

print("\n Visibility graph input: ", polygons)

# Path planning with visibility graph, start and goal
start_point = vg.Point(0.0, 0.0)  
end_point = vg.Point(6.0, 6.0)
shortest_path = visgraph.shortest_path(start_point, end_point)
print(shortest_path)

# Path distance
path_distance = 0
prev_point = shortest_path[0]
for point in shortest_path[1:]:
    path_distance += math.sqrt(pow(point.x - prev_point.x, 2)+pow(point.y - prev_point.y, 2))
    prev_point = point
print('Shortest path distance: {}'.format(path_distance))

# Geometric global path of the path found with visibility graph
path = []
for i in range(len(shortest_path)):
    path.append(Point(shortest_path[i].x,shortest_path[i].y))

path = GeoSeries(path)

print(path)
path.plot()

# Loading visibility graph from previous graph
visgraph2 = vg.VisGraph()
visgraph2.load('graph_test')

# Path planning with visibility graph, start and goal
start_point = vg.Point(0.0, 0.0)   # Start
end_point = vg.Point(10.0, 10.0) # Goal
shortest = visgraph2.shortest_path(start_point, end_point)
print(shortest)