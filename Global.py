import pyvisgraph as vg
import math
import geopandas as gpd
from geopandas import GeoSeries
from shapely.geometry import Polygon, Point, LineString

#Constant margin to add to obstacle
margin=40

def plot_geometric_data(g):
    g.plot('Reds') 

def obstacles_to_polygons(list_obstacles):
    # Convert this polygone list into a geometric set of polygons
    list_polygons = []
    for obstacle in list_obstacles:
        list_polygons.append(Polygon(obstacle))
    
    # Create Geometric graph of the obstacles as given by the vision analysis
    g = GeoSeries(list_polygons)
    return g


def polygons_add_margin(g):
    # Geometric graph of the obstacles with the margin
    g=g.buffer(margin,join_style=2) 
    return g


def polygons_to_VisibilityGraph(g):
    # Visibility graph created from the geometric graph
    polygons = []
    for poly in g:
        x, y = poly.exterior.coords.xy
        polygon_vg = []
        for i in range(len(x)):
            polygon_vg.append(vg.Point(x[i],y[i]))
        polygons.append(polygon_vg)

    visgraph = vg.VisGraph()
    visgraph.build(polygons)
    
    return visgraph

def save_VisibilityGraph(visgraph,visgraph_name):
    visgraph.save(visgraph_name)

def point_to_VisibilityGraph(point):
    point = vg.Point(point[0],point[1])
    return point

    
def VisibilityGraph_shortest_path(visgraph, start_point, end_point):
    # Path planning with visibility graph, start and goal
    start_point=point_to_VisibilityGraph(start_point)
    end_point=point_to_VisibilityGraph(end_point)
    shortest_path = visgraph.shortest_path(start_point, end_point)
    
    return shortest_path

def path_distance(shortest_path):
    # Path distance
    path_distance = 0
    prev_point = shortest_path[0]
    for point in shortest_path[1:]:
        path_distance += math.sqrt(pow(point.x - prev_point.x, 2)+pow(point.y - prev_point.y, 2))
        prev_point = point 
    return path_distance

def ShortestPath_to_geometric(shortest_path):
    # Geometric global path of the path found with visibility graph
    path = []
    for i in range(len(shortest_path)):
        path.append(Point(shortest_path[i].x,shortest_path[i].y))
    path = GeoSeries(path)
    return path

def load_VisibilityGraph(visgraph_name):
    # Loading visibility graph from previous graph
    visgraph2 = vg.VisGraph()
    visgraph2.load(visgraph_name)
    
def geometric_path_to_vector(path):
    check_points=[]
    for i in range(len(path)):
        check_points.append([path.x[i],path.y[i]])
    return check_points

def global_pathplanning(start_point,end_point,list_obstacles):
    
    #convert polygone list into a geometric set and add margin
    g_without_margin = obstacles_to_polygons(list_obstacles)
    g = polygons_add_margin(g_without_margin)
    
    #Visibility graph shortest path algorithm
    visgraph = polygons_to_VisibilityGraph(g)
    shortest_path = VisibilityGraph_shortest_path(visgraph, start_point, end_point)
    distance = path_distance(shortest_path)
    
    #convert shortest path into a geometric data
    path = ShortestPath_to_geometric(shortest_path)
    
    #plot the geometric data set with both the path and the obstacles
    g = g.geometry.append(g_without_margin.geometry)
    g = g.geometry.append(path.geometry)
    plot_geometric_data(g)
   
    #convert the geometric path into a vector for the control
    path=geometric_path_to_vector(path)
    
    return path

