import os
import yaml

import math
import numpy as np
import random
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

def generate_colors(vertices):
    colors = {}
    
    for vertex in vertices:
        r = random.random()
        g = random.random()
        b = random.random()
        color = (r, g, b)
        
        colors[vertex["name"]] = color
        
    return colors

def show_vertices(vertices, colors):
    bmax = -np.Infinity
    bscaler = 2.0
        
    for vertex in vertices:        
        col = colors[vertex["name"]]
        
        current = vertex["current"]
        
        tail = (current[0], current[1])
        unit_vec = (math.cos(current[2]), math.sin(current[2]))
        head = (tail[0] + unit_vec[0], tail[1] + unit_vec[1])
        
        if (abs(current[0]) > bmax):
            bmax = abs(current[0])
        if (abs(current[1] > bmax)):
            bmax = abs(current[1])

        prop = dict(color=col,arrowstyle="-|>")
        
        plt.annotate("", xy=head, xytext=tail, arrowprops=prop)
        plt.scatter(current[0], current[1], c=[col])
    
    bmax *= bscaler
    plt.xlim([-bmax, bmax])
    plt.ylim([-bmax, bmax])
    
def show_edges(vertices, adj_priority_list, colors):
    for adj_priority in adj_priority_list:
        col = colors[adj_priority["higher"]]
        high_current = getCurrentPose(adj_priority["higher"], vertices)
        for low in adj_priority["lower"]:
            low_current = getCurrentPose(low, vertices)
            plt.plot([high_current[0], low_current[0]], [high_current[1], low_current[1]], '-', c=col)
        
def getCurrentPose(name, vertices):
    for vertex in vertices:
        if vertex["name"] == name:
            return vertex["current"]

if __name__ == '__main__': 
    with open(os.path.join(os.getcwd(), "result", "dynamic_graph.yaml")) as dynamic_graph:
        dynamic_graph = yaml.load(dynamic_graph, Loader=yaml.Loader)

    colors = generate_colors(dynamic_graph["vertices"])
    show_edges(dynamic_graph["vertices"], dynamic_graph["adjacent_priority_list"], colors)
    show_vertices(dynamic_graph["vertices"], colors)

    plt.gca().set_aspect('equal')
    plt.show()