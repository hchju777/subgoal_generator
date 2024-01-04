import os
import yaml

import math
import numpy as np
import random
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.collections import PatchCollection

def generate_colors(agents):
    colors = {}
    
    for agent in agents:
        r = random.random()
        g = random.random()
        b = random.random()
        color = (r, g, b)
        
        colors[agent["info"]["name"]] = color
        
    return colors

def show_agents(agents, colors):
    bmax = -np.Infinity
    bscaler = 2.0
    
    for agent in agents:
        col = colors[agent["info"]["name"]]
        secondary_col = (1 - col[0], 1 - col[1], 1 - col[2])
        pose = agent["info"]["pose"]
        goal = agent["info"]["goal"]
        radius = agent["info"]["radius"]
        
        v = [abs(pose[0]), abs(pose[1]), abs(goal[0]), abs(goal[1]), bmax]
        bmax = max(v)
        
        plt.plot([pose[0], goal[0]], [pose[1], goal[1]], '--', c=col, lw=3.0)
        plt.scatter(goal[0], goal[1], c=[col])
        
        draw_agent = plt.Circle((pose[0], pose[1]), radius, color=col, fill=True)
        plt.gcf().gca().add_artist(draw_agent)
        
        tail = (pose[0], pose[1])
        unit_vec = (math.cos(pose[2]), math.sin(pose[2]))
        head = (tail[0] + radius * unit_vec[0], tail[1] + radius * unit_vec[1])
        
        plt.plot([tail[0], head[0]], [tail[1], head[1]], '-', c=secondary_col, lw=2.0)
        
    bmax *= bscaler
    plt.xlim([-bmax, bmax])
    plt.ylim([-bmax, bmax])
    
def show_VOCones(agents, colors):
    for agent in agents:
        name = agent["info"]["name"]
        show_VOCone(agent, colors[name])
    
def show_VOCone(agent, color):
    colors = [color]
    
    if "vo" in agent:
        for cone in agent["vo"]:
            point = cone["point"]        
            radius = cone["radius"]
            left_direction = cone["left_direction"]
            right_direction = cone["right_direction"]
                    
            if radius == "inf":
                radius = 10000
        
            left_rad = math.atan2(left_direction[1], left_direction[0])
            right_rad = math.atan2(right_direction[1], right_direction[0])
            left_deg = left_rad * 180 / math.pi
            right_deg = right_rad * 180 / math.pi
        
            patches = []
        
            wedge = mpatches.Wedge((point[0], point[1]), radius, right_deg, left_deg, ec="none")
            patches.append(wedge)

            collection = PatchCollection(patches, facecolors=colors, alpha=0.3)
            plt.gca().add_collection(collection)
        
def getAgent(name, agents):
    for agent in agents:
        if agent["info"]["name"] == name:
            return agent

if __name__ == '__main__': 
    with open(os.path.join(os.getcwd(), "result", "velocity_obstacle.yaml")) as velocity_obstacle:
        agents = yaml.load(velocity_obstacle, Loader=yaml.Loader)
    
    target = "robot3"
    
    colors = generate_colors(agents)
    # show_VOCone(getAgent(target, agents), colors[target])
    
    show_VOCones(agents, colors)
    show_agents(agents, colors)

    plt.gca().set_aspect('equal')
    plt.show()