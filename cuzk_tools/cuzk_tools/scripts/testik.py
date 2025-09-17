# import pylas

path = "/home/herolada/.ros/cache/cuzk_tools/elevation/PRAH72.laz"

import laspy


# Open the file
with laspy.open(path) as laz_file:
    header = laz_file.header
    print("Point format:", header.point_format)
    print("Number of points:", header.point_count)

    # Read all points
    points = laz_file.read().points

# Access point data
print("X coordinates:", points['X'][:10])  # First 10 X values
print("Y coordinates:", points['Y'][:10])
print("Z coordinates:", points['Z'][:10])
