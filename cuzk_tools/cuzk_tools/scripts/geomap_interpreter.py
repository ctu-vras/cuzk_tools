import geographic_msgs
from shapely.geometry import Point,MultiLineString,MultiPolygon,Polygon,LineString

def geomap_callback(msg: geographic_msgs.msg.GeographicMap):
    """ Parse a GeographicMap ROS msg into a python dictionary. Specifically designed to work with
    the cuzk_tools package, so might not be directly usable with other sources of this msg.
    """
    geo_objects = dict()
    geo_objects["point"] = []
    geo_objects["multiline_string"] = []
    geo_objects["multi_polygon"] = []

    point_ids = {}  
    for wid in range(len(msg.points)):
        point_ids[msg.points[wid].id.uuid] = wid

    for geo_object in msg.features:
        object_points = []
        
        for prop in geo_object.props:
            if prop.key == "category":
                category = prop.value
            elif prop.key == "geom_type":
                geom_type = prop.value
            else:
                pass
    
        for component_id in geo_object.components:
            point_coords = msg.points[point_ids[component_id.uuid]].position
            object_points.append([point_coords.latitude, point_coords.longitude, point_coords.altitude])

        geom_object = dict()
        geom_object["geom_type"] = geom_type
        geom_object["category"] = category

        if geom_type == "Point":
            coords = Point(object_points[0])
            geom_object["coordinates"] = coords
            geo_objects["point"].append(geom_object)

        # In cuzk data it is called MultiLineString, BUT it is always a single line string. We repeat this pattern here.
        elif geom_type == 'MultiLineString':
            ls = LineString(object_points)
            coords = MultiLineString([ls])
            geom_object["coordinates"] = coords
            geo_objects["multiline_string"].append(geom_object)

        # In cuzk data it is called MultiPolygon, BUT it is always a single polygon. We repeat this pattern here.
        elif geom_type == 'MultiPolygon':
            poly = Polygon(object_points)
            coords = MultiPolygon([poly])
            geom_object["coordinates"] = coords
            geo_objects["multi_polygon"].append(geom_object)

        else:
            raise TypeError("Expected types are MultiLineString, MultiPolygon and Point. Got {} instead.".format(geom_type))

    return geo_objects