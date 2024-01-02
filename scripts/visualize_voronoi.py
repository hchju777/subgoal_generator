import matplotlib.pyplot as plt

import os
import yaml

if __name__ == '__main__': 
    with open(os.path.join(os.getcwd(), "result", "voronoi.yaml")) as voronoi_diagram:
        voronoi_diagram = yaml.load(voronoi_diagram, Loader=yaml.Loader)

    for cell in voronoi_diagram:
        for edge in cell["polygon"]:
            point_from = edge["from"]
            point_to = edge["to"]
            plt.scatter(point_from[0], point_from[1], c='blue')
            plt.scatter(point_to[0], point_to[1], c='blue')
            
            plt.plot([point_from[0], point_to[0]], [point_from[1], point_to[1]], c='blue')
            
        point = cell["point"]
        plt.scatter(point[0], point[1], c='red')

    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()