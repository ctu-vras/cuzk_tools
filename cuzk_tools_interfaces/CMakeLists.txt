cmake_minimum_required(VERSION 3.8)
project(cuzk_tools_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
# find_package(jsk_recognition_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LineString.msg"
  "msg/LineStringArray.msg"
  "msg/OrtoImage.msg"
  "msg/PointArray.msg"
  "msg/Polygons.msg"
  "msg/Topography.msg"
  "msg/TopographyCategory.msg"
  "srv/ElevationGet.srv"
  "srv/ElevationPublish.srv"
  "srv/ElevationSetTF.srv"
  "srv/GeoPointElevationGet.srv"
  "srv/GetTopography.srv"
  "srv/MarkerGet.srv"
  "srv/PointElevationGet.srv"
  DEPENDENCIES std_msgs sensor_msgs geographic_msgs geometry_msgs #jsk_recognition_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

ament_package()
