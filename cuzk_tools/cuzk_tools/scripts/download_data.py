import os
import numpy as np

from cuzk_tools.scripts.dmr5g import Dmr5gParser, WGS_TO_SJTSK, SJTSK_TO_WGS, get_sjtsk_to_utm_trans, get_utm_to_sjtsk_trans
from cuzk_tools.scripts.elevation_class import UnsupportedFrameError


class Dmr5gDownloader():
    def __init__(self):
        self.cache_dir = os.environ['HOME'] + "/.ros/cache/cuzk_tools/elevation/"

        if not os.path.exists(self.cache_dir):
            os.mkdir(self.cache_dir)

        self.elev_data_parser = Dmr5gParser(self.cache_dir)

    def download_radius(self, point, radius, frame, utm_zone=None):
        point_sjtsk = self.point2sjtsk(point,frame,utm_zone)

        ids = self.elev_data_parser.get_tile_ids(point_sjtsk, 0 if radius is None else radius)

        for id in ids:
            tile_code = self.elev_data_parser.get_tile_code(id)
            fn = tile_code + ".laz"

            if not self.is_file_in_dir(self.cache_dir, fn):
                self.elev_data_parser.download_tile(id)
                print("Downloading tile {}.".format(tile_code))
            else:
                print("Tile {} already in cache.".format(tile_code))


    def download_rectangle(self, x_min,y_min,x_max,y_max, frame, utm_zone=None):
        tl_sjtsk = np.array(self.point2sjtsk([x_min,y_max],frame,utm_zone))
        tr_sjtsk = np.array(self.point2sjtsk([x_max,y_max],frame,utm_zone))
        bl_sjtsk = np.array(self.point2sjtsk([x_min,y_min],frame,utm_zone))
        br_sjtsk = np.array(self.point2sjtsk([x_max,y_min],frame,utm_zone))

        ids = self.elev_data_parser.get_tile_ids_rect(tl_sjtsk,tr_sjtsk,bl_sjtsk,br_sjtsk)

        for id in ids:
            tile_code = self.elev_data_parser.get_tile_code(id)
            fn = tile_code + ".laz"

            if not self.is_file_in_dir(self.cache_dir, fn):
                self.elev_data_parser.download_tile(id)
                print("Downloading tile {}.".format(tile_code))
            else:
                print("Tile {} already in cache.".format(tile_code))

    def is_file_in_dir(self, dir_path, fn):    
        file_path = os.path.join(dir_path, fn)
        
        return os.path.exists(file_path)
    
    def point2sjtsk(self,point,frame, utm_zone=None):
        sjtsk_coords = None

        if frame == "sjtsk":
            sjtsk_coords = point

        elif frame == "utm":
            if utm_zone is None:
                raise ValueError("point2sjtsk: utm_zone has not been set.")

            transformer = get_utm_to_sjtsk_trans(utm_zone[2], utm_zone[:2])
            sjtsk_coords = transformer.transform(point[0], point[1])
            
            """ elif frame == "utm_local":
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
            """
        
        elif frame == "wgs":
            sjtsk_coords = WGS_TO_SJTSK.transform(point[1], point[0])

        else:
            raise UnsupportedFrameError("Frame {} is not one of ('sjtsk','utm',,'wgs').".format(frame))
        
        return sjtsk_coords
    

if __name__ == "__main__":
    point = [14.,50.]
    radius = 1000
    xmin = 383875.47
    xmax = 383875.47
    ymin = 5500363.10
    ymax = 5511363.10
    frame = "utm"
    utm_zone = "33N"
    downloader = Dmr5gDownloader()
    #downloader.download_radius(point,radius,frame)
    downloader.download_rectangle(xmin,ymin,xmax,ymax,frame,utm_zone)