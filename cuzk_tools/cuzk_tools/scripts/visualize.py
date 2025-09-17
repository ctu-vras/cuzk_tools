import geopandas as gpd
import numpy as np
import os
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

from descartes import PolygonPatch
from shapely.geometry import MultiPolygon

os.environ["SHAPE_RESTORE_SHX"] = "YES"


def line_visualize(shp_file_path):
    gdf = gpd.read_file(shp_file_path)

    linestring = gdf['geometry'][0]
    data = np.array(linestring.coords)

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    ax.scatter(data[:,0], data[:,1], data[:,2], marker='o')

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    plt.show()


def plot_polygons(shp_file_path):
    gdf = gpd.read_file(shp_file_path)

    polygons = gdf['geometry']

    multi_polygon = MultiPolygon(polygons)

    # Create a figure and axes
    fig, ax = plt.subplots()

    # Create a PolygonPatch for the MultiPolygon and add it to the axes
    patch = PolygonPatch(multi_polygon, facecolor='#6699cc', edgecolor='#6699cc', alpha=0.7)
    ax.add_patch(patch)

    # Set axis limits based on the extent of all polygons
    minx, miny, maxx, maxy = polygons[0].bounds
    for polygon in polygons:
        minx = min(minx, polygon.bounds[0])
        miny = min(miny, polygon.bounds[1])
        maxx = max(maxx, polygon.bounds[2])
        maxy = max(maxy, polygon.bounds[3])

    ax.set_xlim(minx - 1, maxx + 1)
    ax.set_ylim(miny - 1, maxy + 1)

    # Set labels for x and y axes
    ax.set_xlabel('Longitude')
    ax.set_ylabel('Latitude')

    # Set the aspect ratio to be equal
    ax.set_aspect('equal', 'box')

    # Show the plot
    plt.show()


def plot_image(path):
    # Load the PNG image using matplotlib's imread function
    image = mpimg.imread(path)

    # Create a figure and axis
    fig, ax = plt.subplots()

    # Display the image using imshow
    ax.imshow(image)

    # Hide the axis ticks and labels
    ax.axis('off')

    # Show the image
    plt.show()