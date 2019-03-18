cmake_minimum_required(VERSION 2.8.3)
project(path_gen_srv)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  cv_bridge
  sensor_msgs
  std_msgs
  image_transport
  dynamic_reconfigure
)


## Generate services in the 'srv' folder
 add_service_files(
   FILES
   path_gen_srv.srv
#   Service1.srv
#   Service2.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs
   nav_msgs
 )


## Generate dynamic reconfigure parameters in the 'cfg' folder
 generate_dynamic_reconfigure_options(
   cfg/path_srv.cfg
#   cfg/DynReconf2.cfg
 )

catkin_package(

)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)


find_package(OpenCV REQUIRED)
catkin_package(
  CATKIN_DEPENDS 
  cv_bridge 
  image_transport
  roscpp sensor_msgs 
  std_msgs
)


#add_executable(test_srv src/test_srv.cpp)
#target_link_libraries(test_srv ${catkin_LIBRARIES})
#add_dependencies(test_srv path_gen_gencpp)


add_executable(path_gen_srv src/path_gen_service.cpp)
target_link_libraries(path_gen_srv ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(path_gen_srv path_gen_gencpp ${PROJECT_NAME}_gencfg)


add_executable(vehicle1 src/vehicle1.cpp)
target_link_libraries(vehicle1 ${catkin_LIBRARIES})
add_dependencies(vehicle1 path_gen_gencpp)

add_executable(vehicle2 src/vehicle2.cpp)
target_link_libraries(vehicle2 ${catkin_LIBRARIES})
add_dependencies(vehicle2 path_gen_gencpp)

add_executable(vehicle3 src/vehicle3.cpp)
target_link_libraries(vehicle3 ${catkin_LIBRARIES})
add_dependencies(vehicle3 path_gen_gencpp)