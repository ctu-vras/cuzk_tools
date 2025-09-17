#!/usr/bin/env python

"""
Subscribe to fused localization odometry (gps_odom) and get the elevation at that position. Publish the elevation as odometry.

As always works only in Czechia.
"""

import rclpy
import json
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.time import Time
from tf_transformations import euler_from_quaternion
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

import random
import math
import numpy as np
import os
from scipy.spatial import cKDTree
from scipy.interpolate import LinearNDInterpolator, NearestNDInterpolator

from cuzk_tools.scripts.dmr5g import Dmr5gParser, get_utm_to_sjtsk_trans
# from cuzk_tools.scripts.elevation_class import Elevation,UnsupportedFrameError

class ElevationOdomNode(Node):

    def __init__(self):

        super().__init__('elevation_odom')

        # ----------- ELEVATION STUFF -----------
        self.cache_dir = os.environ['HOME'] + "/.ros/cache/cuzk_tools/elevation/"

        os.makedirs(self.cache_dir, exist_ok=True)
            
        self.elev_data_parser = Dmr5gParser(self.cache_dir, self.get_logger)

        self.tile_data_dict = dict()
        self.tree_dict = dict()

        # ----------- ROS STUFF -----------
        self.declare_parameter('followed_tf', 'base_link')
        self.declare_parameter('reference_tf', 'utm')
        self.declare_parameter('utm_zone', '33N')
        
        self.followed_tf = self.get_parameter('followed_tf').get_parameter_value().string_value
        self.reference_tf = self.get_parameter('reference_tf').get_parameter_value().string_value
        self.utm_zone = self.get_parameter('utm_zone').get_parameter_value().string_value

        self.odom_sub = self.create_subscription(
            Odometry,
            "odom",
            self.odom_callback,
            10)
        
        self.elev_odom_pub = self.create_publisher(
            Odometry,
            "odom_elev",
            10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def odom_callback(self, msg):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.reference_tf,  
                msg.header.frame_id,                
                Time()
            )
        except Exception as e:
            self.get_logger().error(f'(elevation_odom) Could not get transform to `utm`: {e}')
            return
        
        # Convert quaternion to list format for tf
        quat_list = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]

        # Convert quaternion to Euler angles (roll, pitch, yaw) in radians
        _, _, yaw = euler_from_quaternion(quat_list)
        
        utm_coords = (trans.transform.translation.x + np.cos(yaw) * msg.pose.pose.position.x - np.sin(yaw) * msg.pose.pose.position.y,
                      trans.transform.translation.y + np.sin(yaw) * msg.pose.pose.position.x + np.cos(yaw) * msg.pose.pose.position.y)


        sjtsk_coords = get_utm_to_sjtsk_trans(self.utm_zone[2], self.utm_zone[:2]).transform(utm_coords[0],utm_coords[1])
        # self.get_logger().info(f"(elevation_odom) UTM coords: {utm_coords}")
        elevation = self.sjtsk2elevation(sjtsk_coords)
        # self.get_logger().info(f"(elevation_odom) elevation: {elevation}")
        if math.isnan(elevation):
            self.get_logger().error("(elevation_odom) Elevation is `nan`. Not publishing.")
            return

        msg_out = Odometry()
        msg_out.header = msg.header
        msg_out.pose.pose.position.z = elevation.item()
        msg_out.pose.covariance[14] = 1

        self.elev_odom_pub.publish(msg_out)

    def update_utm_zone(self, msg):
        self.utm_zone = msg.data

    def is_file_in_dir(self, dir_path, fn):    
        file_path = os.path.join(dir_path, fn)
        
        return os.path.exists(file_path)

    def get_data(self, point_sjtsk, radius=None):
        ids = self.elev_data_parser.get_tile_ids(point_sjtsk, 0 if radius is None else radius)

        sjtsk_data = np.zeros(0,dtype=[
                    ('x', np.float64),
                    ('y', np.float64),
                    ('z', np.float64)])

        for id in ids:
            tile_code = self.elev_data_parser.get_tile_code(id)
            fn = tile_code + ".laz"

            if not self.is_file_in_dir(self.cache_dir, fn):
                self.get_logger().info("Tile {} not in cache. Downloading...".format(tile_code))
                self.elev_data_parser.download_tile(id)
            
            else:
                # If the update_dates file does not exist, create it first.
                if not os.path.exists(self.cache_dir + "update_dates.json"):
                    with open(self.cache_dir + "update_dates.json", 'w') as upd_f:
                        pass

                with open(self.cache_dir + "update_dates.json", "r") as f:
                    tile_cache_date_dict = json.load(f)

                    if not tile_code in tile_cache_date_dict:
                        self.get_logger().info("Tile {} update date missing. Downloading...".format(tile_code))
                        self.elev_data_parser.download_tile(id)
                    
                    else:
                        tile_cache_date = tile_cache_date_dict[tile_code]
                        tile_update_date = self.elev_data_parser.get_tile_update_date(id)
                        
                        if tile_cache_date != tile_update_date:
                            self.get_logger().info("Tile {} needs an update. Downloading...".format(tile_code))
                            self.elev_data_parser.download_tile(id)

            try:
                tile_data = self.elev_data_parser.get_tile_data(fn)
            except Exception as e:
                self.get_logger().warn("Cannot open tile file (likely because it hasn't been downloaded). Error: " + str(e))
            else:
                sjtsk_data = np.concatenate((sjtsk_data,tile_data))

        if radius is not None and sjtsk_data is not None:
            dists = ((sjtsk_data['x'] - point_sjtsk[0])**2 + (sjtsk_data['y'] - point_sjtsk[1])**2)**(1/2)
            sjtsk_data = sjtsk_data[dists <= radius]
        else:
            pass
        
        if len(sjtsk_data) == 0:
            self.get_logger().warn("Failed to obtain any data.")
            
        return sjtsk_data

    def sjtsk2elevation(self, sjtsk_coords):
        # Get tile containing robot position.
        tile_id = self.elev_data_parser.get_tile_ids(sjtsk_coords, 0)[0]

        # If tile is new, open it, create cKDTree (x,y only), save it.
        if tile_id in self.tile_data_dict and tile_id in self.tree_dict:
            pass
        else:
            tile_data = self.get_data(sjtsk_coords,None)

            if len(tile_data) == 0:
                self.get_logger().error("(elevation_odom) No data available.")
                return float('nan')
            
            self.tile_data_dict[tile_id] = tile_data.view((np.float64,len(tile_data.dtype.names)))

            self.tree_dict[tile_id] = cKDTree(self.tile_data_dict[tile_id][:,:2])

        # Use (saved) cKDTree to find nearest point.
        _,inds = self.tree_dict[tile_id].query(sjtsk_coords, k=10)
        nearest_points = self.tile_data_dict[tile_id][inds,:]

        # self.get_logger().info(f"(elevation_odom) nearest points: {nearest_points}")

        # Interpolate to obtain elevation.
        elevation = self.interpolate_elevation(sjtsk_coords, nearest_points)

        # Create response with x,y,z. 
        return elevation
    
    def interpolate_elevation(self, point, near_points):
        try:
            interpolator = LinearNDInterpolator(near_points[:,:2], near_points[:,2])
        except:
            self.get_logger().warn("Using nearest interpolation instead of linear due to error.")
            interpolator = NearestNDInterpolator(near_points[:,:2], near_points[:,2])

        elevation = interpolator(point)

        if np.isnan(elevation):
            self.get_logger().warn("Using nearest interpolation instead of linear due to nan.")
            interpolator = NearestNDInterpolator(near_points[:,:2], near_points[:,2])
            elevation = interpolator(point)
        
        return elevation


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ElevationOdomNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()






# class ElevationTF(Elevation):
#     def __init__(self, reference_tf, default_utm_zone, default_followed_tf):
#         rospy.init_node('pose_elevation_tf')

#         self.reference_tf = reference_tf
#         self.followed_tf = default_followed_tf

#         self.cache_dir = os.environ['HOME'] + "/.ros/cache/cuzk_tools/elevation/"

#         if not os.path.exists(self.cache_dir):
#             os.mkdir(self.cache_dir)
            
#         self.elev_data_parser = Dmr5gParser(self.cache_dir)

#         self.utm_zone = default_utm_zone
#         self.utm_zone_sub = rospy.Subscriber("utm_zone", String, self.update_utm_zone, queue_size=10)

#         self.utm_local_trans = None
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

#         self.tf_broadcaster = tf2_ros.TransformBroadcaster()

#         rospy.Service('set_elevation_tf', ElevationSetTF, self.handle_set_tf)
        
#         self.tree_dict = dict()
#         self.tile_data_dict = dict()

#     def handle_set_tf(self, req):
#         self.followed_tf = req.frame.data
#         return ElevationSetTFResponse()     # empty response

#     def sjtsk2elevation(self, sjtsk_coords):
#         # Get tile containing robot position.
#         tile_id = self.elev_data_parser.get_tile_ids(sjtsk_coords, 0)[0]

#         # If tile is new, open it, create cKDTree (x,y only), save it.
#         if tile_id in self.tile_data_dict and tile_id in self.tree_dict:
#             pass
#         else:
#             tile_data = self.get_data(sjtsk_coords,None)

#             if len(tile_data) == 0:
#                 rospy.logwarn("No data available. Defaulting to elevation = 0.")
#                 return 0.
            
#             self.tile_data_dict[tile_id] = tile_data.view((np.float64,len(tile_data.dtype.names)))

#             self.tree_dict[tile_id] = cKDTree(self.tile_data_dict[tile_id][:,:2])

#         # Use (saved) cKDTree to find nearest point.
#         _,inds = self.tree_dict[tile_id].query(sjtsk_coords, k=10)
#         nearest_points = self.tile_data_dict[tile_id][inds,:]

#         # Interpolate to obtain elevation.
#         elevation = self.interpolate_elevation(sjtsk_coords, nearest_points)

#         # Create response with x,y,z. 
#         return elevation
    
#     def interpolate_elevation(self, point, near_points):
#         try:
#             interpolator = LinearNDInterpolator(near_points[:,:2], near_points[:,2])
#         except:
#             rospy.logwarn("Using nearest interpolation instead of linear due to error.")
#             interpolator = NearestNDInterpolator(near_points[:,:2], near_points[:,2])

#         elevation = interpolator(point)

#         if np.isnan(elevation):
#             rospy.logwarn("Using nearest interpolation instead of linear due to nan.")
#             interpolator = NearestNDInterpolator(near_points[:,:2], near_points[:,2])
#             elevation = interpolator(point)
        
#         return elevation
    
# if __name__ == "__main__":
#     reference_tf = "utm" #rospy.get_param('/pose_elevation_tf/reference_tf', None)
#     default_utm_zone = rospy.get_param('/pose_elevation_tf/default_utm_zone', None)
#     default_followed_tf = rospy.get_param('/pose_elevation_tf/default_followed_tf', None)
#     node = ElevationTF(reference_tf, default_utm_zone, default_followed_tf)

#     rate = rospy.Rate(1)
    
#     while not rospy.is_shutdown():
#         try:
#             trans = node.tf_buffer.lookup_transform(node.reference_tf, node.followed_tf, rospy.Time())
#         except:
#             rospy.logwarn_throttle(5,"Waiting for transformation between {} and {}.".format(node.followed_tf, node.reference_tf))
#             rate.sleep()
#             continue        
        
#         #trans.transform.translation.x += random.uniform(-100,100)
#         #trans.transform.translation.y += random.uniform(-100,100)
        
#         utm_coords = [trans.transform.translation.x, trans.transform.translation.y] 
        
#         if node.utm_zone is None:
#             rospy.logwarn_throttle(5,"utm_zone has not been set.")
#             rospy.sleep(rate)
#             continue

#         sjtsk_coords = get_utm_to_sjtsk_trans(node.utm_zone[2], node.utm_zone[:2]).transform(utm_coords[0],utm_coords[1])
#         elevation = node.sjtsk2elevation(sjtsk_coords)

#         msg = TransformStamped()
#         trans_ele = trans.transform
#         trans_ele.translation.z = elevation

#         msg.header = node.get_header(node.reference_tf)
#         msg.child_frame_id = node.followed_tf + "_elev"
#         msg.transform = trans_ele

#         node.tf_broadcaster.sendTransform(msg)

#         rate.sleep()