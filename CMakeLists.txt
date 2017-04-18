cmake_minimum_required(VERSION 2.8.3)
project(velodyne_odo)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  pcl_ros
  roscpp
  sensor_msgs
  tf
  angles
  pcl_conversions
)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(VTK)

if(VTK_FOUND)
    add_definitions( -DSHOW_VIEWER)
endif(VTK_FOUND)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES velodyne_odo
  CATKIN_DEPENDS tf
#  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(include/velodyne_odo
                    ${Boost_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIR})

add_library(PCregistration
            src/PCRegistration.cpp
              src/CloudNormalizer.cpp
            )



## Declare a C++ executable
add_executable(velodyne_odo_node_mt
                src/velodyne_odo_node_mt.cpp
                src/VelodyneOdoMT.cpp
              )

add_executable(velodyne_odo_node
                src/velodyne_odo_node.cpp
                src/VelodyneOdo.cpp
              )

add_executable( sph
              src/SphereArrTest.cpp
              src/PCRegistration.cpp
              src/CloudNormalizer.cpp
              #src/normalize_cloud.cpp
              )



## Specify libraries to link a library or executable target against
target_link_libraries(sph
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${VTK_LIBRARIES})


target_link_libraries( velodyne_odo_node
                        ${catkin_LIBRARIES}
                           ${PCL_LIBRARIES}
                           PCregistration)

target_link_libraries( velodyne_odo_node_mt
                        ${catkin_LIBRARIES}
                           ${PCL_LIBRARIES}
                           PCregistration)

if(VTK_FOUND)
    add_executable(vodo_viewer
                test/viewer.cpp
                test/cloud_viewer.cpp
                test/viewer_main.cpp
              )
    target_link_libraries( vodo_viewer
       ${catkin_LIBRARIES}
       ${PCL_LIBRARIES}
       ${VTK_LIBRARIES}
       )
endif(VTK_FOUND)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS velodyne_odo velodyne_odo_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_velodyne_odo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
