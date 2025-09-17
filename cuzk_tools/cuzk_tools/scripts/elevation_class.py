# coding=utf8
"""
The source code for the 'elevation' node.

The node provides two services and some related publishers:
    1. elevation_publish - the request contains a point in WGS, a radius in meters
        and the frames in which we want to publish (boolean flag for each of the four frames).
        The service then publishes the elevation data of the area indicated by the request as PointCloud2.

    2. elevation_get - the request is the same. The service then returns the elevation data as a response.

The open data source is 'ZABAGED Výškopis DMR5G' of the ČÚZK. Only works for the Czechia area.
"""

import rospy
import ros_numpy
import std_msgs
import tf2_ros

from copy import deepcopy

from cuzk_tools_interfaces.msg import OrtoImage
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String, Float64
from visualization_msgs.msg import Marker

from cuzk_tools_interfaces.srv import ElevationPublish, ElevationGet, ElevationPublishResponse, ElevationGetResponse

import json
import numpy as np
import os

from cuzk_tools.scripts.dmr5g import Dmr5gParser, WGS_TO_SJTSK, SJTSK_TO_WGS, get_sjtsk_to_utm_trans, get_utm_to_sjtsk_trans
from cuzk_tools.scripts.orto import get_img
from cuzk_tools.scripts.img2rgb import img2rgb

class UnsupportedFrameError(Exception):
    pass

class Elevation:
    def __init__(self, default_utm_zone):
        rospy.init_node('elevation')

        self.cache_dir = os.environ['HOME'] + "/.ros/cache/cuzk_tools/elevation/"

        if not os.path.exists(self.cache_dir):
            os.makedirs(self.cache_dir, exist_ok=True)

        self.elev_data_parser = Dmr5gParser(self.cache_dir)

        self.sjtsk_frame = "sjtsk"
        self.utm_frame = "utm"
        self.utm_local_frame = "utm_local"
        self.wgs_frame = "wgs"

        self.utm_zone = default_utm_zone
        self.utm_zone_sub = rospy.Subscriber("utm_zone", String, self.update_utm_zone, queue_size=10)

        self.utm_local_trans = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.elev_sjtsk_pub = rospy.Publisher('elevation_sjtsk', PointCloud2, queue_size=10, latch=True)
        self.elev_utm_pub = rospy.Publisher('elevation_utm', PointCloud2, queue_size=10, latch=True)
        self.elev_utm_local_pub = rospy.Publisher('elevation_utm_local', PointCloud2, queue_size=10, latch=True)
        self.elev_wgs_pub = rospy.Publisher('elevation_wgs', PointCloud2, queue_size=10, latch=True)
        
        self.orto_img_pub = rospy.Publisher('elevation_img', OrtoImage, queue_size=10, latch=True)

        rospy.Service('elevation_publish', ElevationPublish, self.handle_elevation_publish)
        rospy.Service('elevation_get', ElevationGet, self.handle_elevation_get)

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
                rospy.loginfo("Tile {} not in cache. Downloading...".format(tile_code))
                self.elev_data_parser.download_tile(id)
            
            else:
                # If the update_dates file does not exist, create it first.
                if not os.path.exists(self.cache_dir + "update_dates.json"):
                    with open(self.cache_dir + "update_dates.json", 'w') as upd_f:
                        pass

                with open(self.cache_dir + "update_dates.json", "r") as f:
                    tile_cache_date_dict = json.load(f)

                    if not tile_code in tile_cache_date_dict:
                        rospy.loginfo("Tile {} update date missing. Downloading...".format(tile_code))
                        self.elev_data_parser.download_tile(id)
                    
                    else:
                        tile_cache_date = tile_cache_date_dict[tile_code]
                        tile_update_date = self.elev_data_parser.get_tile_update_date(id)
                        
                        if tile_cache_date != tile_update_date:
                            rospy.loginfo("Tile {} needs an update. Downloading...".format(tile_code))
                            self.elev_data_parser.download_tile(id)

            try:
                tile_data = self.elev_data_parser.get_tile_data(fn)
            except Exception as e:
                rospy.logwarn("Cannot open tile file (likely because it hasn't been downloaded). Error: " + str(e))
            else:
                sjtsk_data = np.concatenate((sjtsk_data,tile_data))

        if radius is not None and sjtsk_data is not None:
            dists = ((sjtsk_data['x'] - point_sjtsk[0])**2 + (sjtsk_data['y'] - point_sjtsk[1])**2)**(1/2)
            sjtsk_data = sjtsk_data[dists <= radius]
        else:
            pass
        
        if len(sjtsk_data) == 0:
            rospy.logwarn("Failed to obtain any data.")
            
        return sjtsk_data
    
    def get_bg_img(self,point,radius):
        coords = [int(np.floor(point[0]-radius)),
                int(np.floor(point[1]-radius)),
                int(np.ceil(point[0]+radius)),
                int(np.ceil(point[1]+radius))]
        
        img_path = get_img(coords, self.cache_dir)
        return img_path,coords
    
    def coord_transform_data(self, data, transformer, dtype=np.float32):
        if len(data) >= 1:
            wgs_data = transformer.transform(data['x'], data['y'])

            wgs_arr = np.zeros(len(data), dtype=[
            ('x', dtype),
            ('y', dtype),
            ('z', dtype)])
        
            wgs_arr['x'] = wgs_data[0]
            wgs_arr['y'] = wgs_data[1]
            wgs_arr['z'] = data['z']
    
            return wgs_arr
        else:
            return data
        
    def change_arr_type(self,arr,dtype):
        new_arr = np.empty(len(arr), dtype=[('x', dtype),('y', dtype),('z', dtype)])
        new_arr['x'] = arr['x'].astype(dtype)
        new_arr['y'] = arr['y'].astype(dtype)
        new_arr['z'] = arr['z'].astype(dtype)
        return new_arr

    def add_rgb(self,arr,dtype,rgb=None):
        new_arr = np.empty(len(arr), dtype=[('x', dtype),('y', dtype),('z', dtype),('rgb', int)])
        new_arr['x'] = arr['x'].astype(dtype)
        new_arr['y'] = arr['y'].astype(dtype)
        new_arr['z'] = arr['z'].astype(dtype)
        new_arr['rgb'] = rgb
        return new_arr
    
    def get_header(self,frame):
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame
        return header
    

    def point2sjtsk(self,point,frame):
        sjtsk_coords = None

        if frame == "sjtsk":
            sjtsk_coords = point

        elif frame == "utm":
            if self.utm_zone is None:
                raise ValueError("point2sjtsk: utm_zone has not been set.")
            sjtsk_coords = self.coord_transform_data(point, get_utm_to_sjtsk_trans(self.utm_zone[2], self.utm_zone[:2]), dtype=np.float64)

        elif frame == "utm_local":
            try:
                self.utm_local_trans = self.tf_buffer.lookup_transform(self.utm_frame, self.utm_local_frame, rospy.Time())
            except:
                rospy.logwarn("point2sjtsk: Cannot obtain transform (utm, utm_local).")
                if self.utm_local_trans is not None:
                    rospy.logwarn("point2sjtsk: Using last known transform.")
                else:
                    rospy.logwarn("point2sjtsk: Returning None.")
                    return None
                
            point[0] += self.utm_local_trans.transform.translation.x
            point[1] += self.utm_local_trans.transform.translation.y

            if self.utm_zone is None:
                raise ValueError("utm_zone has not been set.")
            
            sjtsk_coords = self.coord_transform_data(point, get_utm_to_sjtsk_trans(self.utm_zone[2], self.utm_zone[:2]), dtype=np.float64)

        elif frame == "wgs":
            sjtsk_coords = self.coord_transform_data(point, WGS_TO_SJTSK, dtype=np.float64)

        else:
            raise UnsupportedFrameError("Frame {} is not one of ('sjtsk','utm','utm_local','wgs').".format(frame))
        
        return sjtsk_coords
    
    def handle_elevation_publish(self, req):
        radius = req.radius.data
        point = [req.point.x, req.point.y]
        point_sjtsk = WGS_TO_SJTSK.transform(point[1],point[0])

        sjtsk_bool = req.sjtsk.data
        utm_bool = req.utm.data
        utm_local_bool = req.utm_local.data
        wgs_bool = req.wgs.data

        sjtsk_data = self.get_data(point_sjtsk,radius)

        n_points = len(sjtsk_data)

        img_path,bounds = self.get_bg_img(point_sjtsk,radius)

        img_path_msg = OrtoImage()
        header = self.get_header(self.sjtsk_frame)
        img_path_msg.header = header
        img_path_msg.path = String(img_path)
        img_path_msg.bounds = [Float64(p) for p in bounds]

        self.orto_img_pub.publish(img_path_msg)

        rgb_bool = True
        if img_path is None:
            rgb_bool = False

        if rgb_bool:
            tl_bl_br = np.array([[bounds[0], bounds[3]],
                                 [bounds[0], bounds[1]],
                                 [bounds[2], bounds[1]]])
            rgb_arr = img2rgb(img_path, tl_bl_br, sjtsk_data.view((np.float64,len(sjtsk_data.dtype.names))))
            rgba_arr = np.hstack((rgb_arr, 255*np.ones((n_points,1)))).astype('uint32')
            rgb = (rgba_arr[:, 3] << 24) | (rgba_arr[:, 0]<< 16) | (rgba_arr[:, 1] << 8) | rgba_arr[:, 2]

            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
            ]
        else:
            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            ]

        if sjtsk_bool:
            pcd_data = deepcopy(sjtsk_data)

            pcd_data['x'] = sjtsk_data['x']
            pcd_data['y'] = sjtsk_data['y']
            pcd_data['z'] = sjtsk_data['z']

            if rgb_bool:
                points = self.add_rgb(pcd_data,np.float32,rgb)
            else:
                points = self.change_arr_type(pcd_data,np.float32)
            
            pcd = pc2.create_cloud(self.get_header(self.sjtsk_frame), fields, points)
            self.elev_sjtsk_pub.publish(pcd)                 

        if utm_bool or utm_local_bool:
            if self.utm_zone is None:
                raise ValueError("utm_zone has not been set.")
            
            utm_data = self.coord_transform_data(sjtsk_data, get_sjtsk_to_utm_trans(self.utm_zone[2], self.utm_zone[:2]), dtype=np.float64)

            if utm_bool:

                if rgb_bool:
                    points = self.add_rgb(utm_data,np.float32,rgb)
                else:
                    points = self.change_arr_type(utm_data,np.float32)
                
                pcd = pc2.create_cloud(self.get_header(self.utm_frame), fields, points)
                self.elev_utm_pub.publish(pcd) 

            if utm_local_bool:
                try:
                    self.utm_local_trans = self.tf_buffer.lookup_transform(self.utm_frame, self.utm_local_frame, rospy.Time())
                    x_offset = self.utm_local_trans.transform.translation.x
                    y_offset = self.utm_local_trans.transform.translation.y
                except:
                    rospy.logwarn("utm_local: Cannot obtain transform (utm, utm_local).")
                    if self.utm_local_trans is not None:
                        x_offset = self.utm_local_trans.transform.translation.x
                        y_offset = self.utm_local_trans.transform.translation.y
                        rospy.logwarn("utm_local: Using last known transform.")

                if self.utm_local_trans is not None:
                    utm_local_data = np.copy(utm_data)
                    utm_local_data['x'] -=  x_offset
                    utm_local_data['y'] -=  y_offset

                    if rgb_bool:
                        points = self.add_rgb(utm_local_data,np.float32,rgb)
                    else:
                        points = self.change_arr_type(utm_local_data,np.float32)
                    
                    pcd = pc2.create_cloud(self.get_header(self.utm_local_frame), fields, points)
                    self.elev_utm_local_pub.publish(pcd) 
                        

                else:
                    rospy.logwarn("utm_local: Publishing empty pointcloud.")

        if wgs_bool:
            wgs_data = self.coord_transform_data(sjtsk_data, SJTSK_TO_WGS)

            if rgb_bool:
                points = self.add_rgb(wgs_data,np.float32,rgb)
            else:
                points = self.change_arr_type(wgs_data,np.float32)
            
            pcd = pc2.create_cloud(self.get_header(self.wgs_frame), fields, points)
            self.elev_wgs_pub.publish(pcd)                

        # Return empty-ish response.
        response = ElevationPublishResponse()
        return response
    
    def handle_elevation_get(self, req):
        radius = req.radius.data
        point = [req.point.x, req.point.y]
        point_sjtsk = WGS_TO_SJTSK.transform(point[1],point[0])

        sjtsk_bool = req.sjtsk.data
        utm_bool = req.utm.data
        utm_local_bool = req.utm_local.data
        wgs_bool = req.wgs.data

        sjtsk_data = self.get_data(point_sjtsk,radius)

        
            
        n_points = len(sjtsk_data)

        sjtsk_msg = PointCloud2()
        utm_msg = PointCloud2()
        utm_local_msg = PointCloud2()
        wgs_msg = PointCloud2()

        if sjtsk_bool:
            points = self.change_arr_type(sjtsk_data,np.float32)

            sjtsk_msg = ros_numpy.msgify(PointCloud2, points)
            sjtsk_msg.header = self.get_header(self.sjtsk_frame)

        if utm_bool or utm_local_bool:
            if self.utm_zone is None:
                raise ValueError("utm_zone has not been set.")
            
            utm_data = self.coord_transform_data(sjtsk_data, get_sjtsk_to_utm_trans(self.utm_zone[2], self.utm_zone[:2]), dtype=np.float64)

            if utm_bool:
                points = self.change_arr_type(utm_data,np.float32)
                
                utm_msg = ros_numpy.msgify(PointCloud2, points)
                utm_msg.header = self.get_header(self.utm_frame)

            if utm_local_bool:
                trans_exists = False
                try:
                    self.utm_local_trans = self.tf_buffer.lookup_transform(self.utm_frame, self.utm_local_frame, rospy.Time())
                    x_offset = self.utm_local_trans.transform.translation.x
                    y_offset = self.utm_local_trans.transform.translation.y
                    trans_exists= True
                except:
                    rospy.logwarn_once("Cannot obtain transform (utm, utm_local). Publishing empty pointcloud.")

                if trans_exists:
                    utm_local_data = np.copy(utm_data)
                    utm_local_data['x'] -=  x_offset
                    utm_local_data['y'] -=  y_offset

                    points = self.change_arr_type(utm_local_data,np.float32)

                    utm_local_msg = ros_numpy.msgify(PointCloud2, points)
                    utm_local_msg.header = self.get_header(self.utm_local_frame)

        if wgs_bool:
            wgs_data = self.coord_transform_data(sjtsk_data, SJTSK_TO_WGS)

            wgs_msg = ros_numpy.msgify(PointCloud2, wgs_data)
            wgs_msg.header = self.get_header(self.wgs_frame)

        response = ElevationGetResponse()

        response.elevation_sjtsk = sjtsk_msg
        response.elevation_utm = utm_msg
        response.elevation_utm_local = utm_local_msg
        response.elevation_wgs = wgs_msg

        return response
