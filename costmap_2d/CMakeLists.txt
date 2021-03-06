cmake_minimum_required(VERSION 2.8.3)
project(costmap_2d)

find_package(catkin REQUIRED
        COMPONENTS
            dynamic_reconfigure
            geometry_msgs
            map_msgs
            message_filters
            message_generation
            nav_msgs
            pluginlib
            roscpp
            sensor_msgs
            std_msgs
            tf
            voxel_grid
        )

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_definitions(${EIGEN_DEFINITIONS})

# messages
add_message_files(
    DIRECTORY msg
    FILES
    VoxelGrid.msg
)

generate_messages(
    DEPENDENCIES
        std_msgs
        geometry_msgs
)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/Costmap2D.cfg
  cfg/ObstaclePlugin.cfg
  cfg/GenericPlugin.cfg
  cfg/InflationPlugin.cfg
)

catkin_package(
    INCLUDE_DIRS
        include
        ${EIGEN_INCLUDE_DIRS}
    LIBRARIES costmap_2d
    CATKIN_DEPENDS
        dynamic_reconfigure
        geometry_msgs
        map_msgs
        message_filters
        message_runtime
        nav_msgs
        pluginlib
        roscpp
        sensor_msgs
        std_msgs
        tf
        voxel_grid
    DEPENDS
        Eigen
        Boost
)

add_library(costmap_2d
  src/array_parser.cpp
  src/costmap_2d.cpp
  src/layer.cpp
  src/layered_costmap.cpp
  src/costmap_2d_ros.cpp
  src/costmap_2d_publisher.cpp
  src/costmap_math.cpp
  src/footprint.cpp
)
add_dependencies(costmap_2d geometry_msgs_gencpp)
target_link_libraries(costmap_2d
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_library(layers
  plugins/footprint_layer.cpp
  plugins/inflation_layer.cpp
  plugins/obstacle_layer.cpp
  plugins/static_layer.cpp
)
target_link_libraries(layers
  costmap_2d
)

add_dependencies(costmap_2d costmap_2d_gencfg)
add_dependencies(layers costmap_2d_gencfg)
add_dependencies(costmap_2d costmap_2d_gencpp)

add_dependencies(costmap_2d_markers visualization_msgs_gencpp)
target_link_libraries(costmap_2d_markers
    costmap_2d
    )

add_executable(costmap_2d_node src/costmap_2d_node.cpp)
target_link_libraries(costmap_2d_node
    costmap_2d
    )

install( TARGETS
    costmap_2d_markers
    costmap_2d_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
    costmap_2d
    layers
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES costmap_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
