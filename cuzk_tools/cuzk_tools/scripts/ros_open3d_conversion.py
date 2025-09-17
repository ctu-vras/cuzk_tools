"""
    TAKEN FROM https://github.com/felixchenfy/open3d_ros_pointcloud_conversion
    ALTERED VERY SLIGHTLY TO WORK WITH CURRENT VERSIONS OF PACKAGES
"""

'''
This script contains 2 functions for converting cloud format between Open3D and ROS:   
* convertCloudFromOpen3dToRos  
* convertCloudFromRosToOpen3d
where the ROS format refers to "sensor_msgs/PointCloud2.msg" type.

This script also contains a test case, which does such a thing:  
(1) Read a open3d_cloud from .pcd file by Open3D.
(2) Convert it to ros_cloud.
(3) Publish ros_cloud to topic.
(4) Subscribe the ros_cloud from the same topic.
(5) Convert ros_cloud back to open3d_cloud.
(6) Display it.  
You can test this script's function by rosrun this script.

'''

import open3d
import numpy as np
from ctypes import * # convert float to uint32

from pymeshfix import MeshFix
from pymeshfix._meshfix import PyTMesh
from pymeshfix.examples import planar_mesh
import pyvista as pv

import rospy
import ros_numpy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

# The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
    [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

# Bit operations
BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8
convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
)
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)

# Convert the datatype of point cloud from Open3D to ROS PointCloud2 (XYZRGB only)
def convertCloudFromOpen3dToRos(open3d_cloud, frame_id="odom"):
    # Set "header"
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    # Set "fields" and "cloud_data"
    points=np.asarray(open3d_cloud.points)
    if not open3d_cloud.colors: # XYZ only
        fields=FIELDS_XYZ
        cloud_data=points
    else: # XYZ + RGB
        fields=FIELDS_XYZRGB
        # -- Change rgb color from "three float" to "one 24-byte int"
        # 0x00FFFFFF is white, 0x00000000 is black.
        colors = np.floor(np.asarray(open3d_cloud.colors)*255) # nx3 matrix
        colors = colors[:,0] * BIT_MOVE_16 +colors[:,1] * BIT_MOVE_8 + colors[:,2]  
        cloud_data=np.c_[points, colors]
    
    # create ros_cloud
    cloud_data_formatted = np.empty(len(cloud_data), dtype=[('x', np.float32),('y', np.float32),('z', np.float32),('rgb', int)])
    cloud_data_formatted['x'] = cloud_data[:,0].astype(np.float32)
    cloud_data_formatted['y'] = cloud_data[:,1].astype(np.float32)
    cloud_data_formatted['z'] = cloud_data[:,2].astype(np.float32)
    cloud_data_formatted['rgb'] = cloud_data[:,3].astype(np.float32)

    return pc2.create_cloud(header, fields, cloud_data_formatted)

def convertCloudFromRosToOpen3d(ros_cloud):
    
    # Get cloud data from ros_cloud
    field_names=[field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))
    #arr = ros_numpy.numpify(ros_cloud)

    # Check empty
    open3d_cloud = open3d.geometry.PointCloud()
    if len(cloud_data)==0:
        print("Converting an empty cloud")
        return None

    # Set open3d_cloud
    if "rgb" in field_names:
        IDX_RGB_IN_FIELD=3 # x, y, z, rgb
        
        # Get xyz
        xyz = [(x,y,z) for x,y,z,rgb in cloud_data ] # (why cannot put this line below rgb?)

        # Get rgb
        # Check whether int or float
        if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
            rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
        else:
            rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

        # combine
        open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))
        open3d_cloud.colors = open3d.utility.Vector3dVector(np.array(rgb)/255.0)
    else:
        xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
        open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))

    # return
    return open3d_cloud,cloud_data



def ball_pivoting_reconstruction(pcd, radii=None, hole_filling_power=50):
    """Given a 3D point cloud, get unstructured mesh using ball pivoting algorithm
    
    Based on this stack overflow code snippet:
    https://stackoverflow.com/questions/56965268/how-do-i-convert-a-3d-point-cloud-ply-into-a-mesh-with-faces-and-vertices
    
    Parameters
    ----------
    xyz: [n_points, 3]
        input point cloud, numpy array
    radii: [n_radii]
        list of radiuses to use for the ball pivoting algorithm
    
    Returns
    -------
    mesh: trimesh Mesh object with verts, faces and normals of the mesh
    
    """
    
    pcd.estimate_normals()
    pcd.orient_normals_to_align_with_direction(np.array([0.,0.,1.]))

    #up_normals = np.asarray(pcd.normals)
    #down_bool = up_normals[:,2] < 0.
    #up_normals[down_bool,2] *= -1

    #pcd.normals = up_normals

    # heuristic to estimate the radius of a rolling ball
    if radii is None:
        distances = pcd.compute_nearest_neighbor_distance()
        avg_dist = np.mean(distances)
        radius = 5.#1.5 * avg_dist   
        radii = [radius, radius * 2]
    
    mesh = open3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        pcd, open3d.utility.DoubleVector(radii))        # takes a long time for large number of points (300000 p. ... 20-30 s)
    
    vertex_colors = np.asarray(mesh.vertex_colors)
    triangles = np.asarray(mesh.triangles)

    #colors = vertex_colors[triangles]
    #colors = np.mean(colors, axis=1)


    #vclean, fclean = pymeshfix.clean_from_arrays(np.asarray(mesh.vertices), np.asarray(mesh.triangles))

    v,f = np.asarray(mesh.vertices), np.asarray(mesh.triangles)

    meshfix = MeshFix(v,f)

    #triangles = np.empty((f.shape[0], 4), dtype=f.dtype)
    #triangles[:, -3:] = f
    #triangles[:, 0] = 3

    #orig_mesh = pv.PolyData(v, triangles)

    #holes = meshfix.extract_holes()

    # Render the mesh and outline the holes
    #plotter = pv.Plotter()
    #plotter.add_mesh(orig_mesh, color=True)
    #plotter.add_mesh(holes, color="r", line_width=5)
    #plotter.enable_eye_dome_lighting()  # helps depth perception
    #_ = plotter.show()


    ###############################################################################
    # This example uses the lower level C interface to the TMesh object.
    mfix = PyTMesh(False)  # False removes extra verbose output
    mfix.load_array(v,f)

    # Fills all the holes having at at most 'nbe' boundary edges. If
    # 'refine' is true, adds inner vertices to reproduce the sampling
    # density of the surroundings. Returns number of holes patched.  If
    # 'nbe' is 0 (default), all the holes are patched.
    mfix.fill_small_boundaries(nbe = hole_filling_power, refine=True)

    ###############################################################################
    # Convert back to a pyvista mesh
    vert, faces = mfix.return_arrays()
    #triangles = np.empty((faces.shape[0], 4), dtype=faces.dtype)
    #triangles[:, -3:] = faces
    #triangles[:, 0] = 3

    #mesh = pv.PolyData(vert, triangles)

    ################################################################################
    # Plot the repaired mesh along with the original holes
    # Note: It seems there is a limit to the repair algorithm whereby some
    # of the holes that include only a single point are not filled. These
    # boundary holes are not detected by VTK's hole repair algorithm
    # either.

    #plotter = pv.Plotter()
    #plotter.add_mesh(mesh, color=True)
    #plotter.add_mesh(holes, color="r", line_width=5)
    #plotter.enable_eye_dome_lighting()  # helps depth perception    _ = plotter.show()
        
    #if fill_holes:
    #    mesh = trimesh.Trimesh(np.asarray(mesh.vertices), np.asarray(mesh.triangles), vertex_normals=np.asarray(mesh.vertex_normals))
    #    mesh.fill_holes()

    # try to fix normals with Trimesh
    #mesh.fix_normals()     # TAKES TOO LONG AND DOES NOT DO MUCH
    
    # save mesh:
    # mesh.export('../logs/mesh.obj')

    #colors = vertex_colors[triangles]
    #colors = np.mean(colors, axis=1)
    
    return vert, faces



# -- Example of usage
if __name__ == "__main__":
    rospy.init_node('test_pc_conversion_between_Open3D_and_ROS', anonymous=True)
    
    # -- Read point cloud from file
    import os
    PYTHON_FILE_PATH=os.path.join(os.path.dirname(__file__))+"/"
    if 0: # test XYZ point cloud format
        filename=PYTHON_FILE_PATH+"test_cloud_XYZ_noRGB.pcd"
    else: # test XYZRGB point cloud format
        filename=PYTHON_FILE_PATH+"test_cloud_XYZRGB.pcd"

    open3d_cloud = open3d.io.read_point_cloud(filename)
    rospy.loginfo("Loading cloud from file by open3d.read_point_cloud: ")
    print(open3d_cloud)
    print("")

    #open3d_cloud.estimate_normals()
    #pcd2mesh(open3d_cloud)

    #mesh = ball_pivoting_reconstruction(open3d_cloud)
    #open3d.visualization.draw_geometries([open3d_cloud, mesh])

    """ # -- Set publisher
    topic_name="kinect2/qhd/points"
    pub = rospy.Publisher(topic_name, PointCloud2, queue_size=1)
    
    # -- Set subscriber
    global received_ros_cloud
    received_ros_cloud = None
    def callback(ros_cloud):
        global received_ros_cloud
        received_ros_cloud=ros_cloud
        rospy.loginfo("-- Received ROS PointCloud2 message.")
    rospy.Subscriber(topic_name, PointCloud2, callback)      
    
    # -- Convert open3d_cloud to ros_cloud, and publish. Until the subscribe receives it.
    while received_ros_cloud is None and not rospy.is_shutdown():
        rospy.loginfo("-- Not receiving ROS PointCloud2 message yet ...")

        if 1: # Use the cloud from file
            rospy.loginfo("Converting cloud from Open3d to ROS PointCloud2 ...")
            ros_cloud = convertCloudFromOpen3dToRos(open3d_cloud)

        else: # Use the cloud with 3 points generated below
            rospy.loginfo("Converting a 3-point cloud into ROS PointCloud2 ...")
            TEST_CLOUD_POINTS = [
                [1.0, 0.0, 0.0, 0xff0000],
                [0.0, 1.0, 0.0, 0x00ff00],
                [0.0, 0.0, 1.0, 0x0000ff],
            ]
            ros_cloud = pc2.create_cloud(
                Header(frame_id="odom"), FIELDS_XYZ , TEST_CLOUD_POINTS)

        # publish cloud
        pub.publish(ros_cloud)
        rospy.loginfo("Conversion and publish success ...\n")
        rospy.sleep(1)
        
    # -- After subscribing the ros cloud, convert it back to open3d, and draw
    received_open3d_cloud = convertCloudFromRosToOpen3d(received_ros_cloud)
    print(received_open3d_cloud)

    # write to file
    output_filename=PYTHON_FILE_PATH+"conversion_result.pcd"
    open3d.io.write_point_cloud(output_filename, received_open3d_cloud)
    rospy.loginfo("-- Write result point cloud to: "+output_filename)

    # draw
    open3d.visualization.draw_geometries([received_open3d_cloud])
    rospy.loginfo("-- Finish display. The program is terminating ...\n") """