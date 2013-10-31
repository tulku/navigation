cmake_minimum_required(VERSION 2.8.3)
project(robot_pose_ekf)

# bfl (Bayesian Filtering Library) is a third party package that uses pkg-config
find_package(PkgConfig)
pkg_check_modules(BFL REQUIRED orocos-bfl)

# People building this branch of nav within groovy will need this version:
include_directories(${BFL_INCLUDE_DIRS})
# People building this branch of nav within hydro will need this version:
include_directories(${BFL_INCLUDE_DIRS}/bfl)

# when no one is building this branch in groovy anymore, the .../bfl
# version should be removed and the
# #include <filter/extendedkalmanfilter.h>
# line in src/odom_estimation.h should become
# #include <bfl/filter/extendedkalmanfilter.h>

message("BFL include dirs:" ${BFL_INCLUDE_DIRS})
message("BFL library dirs:" ${BFL_LIBRARY_DIRS})
link_directories(${BFL_LIBRARY_DIRS})

find_package(catkin REQUIRED
        COMPONENTS
            roscpp
            tf
            nav_msgs
            std_msgs
            geometry_msgs
            sensor_msgs
            message_generation
        )

find_package(Boost REQUIRED COMPONENTS thread)

# services
add_service_files(
    DIRECTORY srv
    FILES
    GetStatus.srv
)

generate_messages(
    DEPENDENCIES
        std_msgs
)

catkin_package(
    CATKIN_DEPENDS
        roscpp
)

include_directories(
    "include"
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    )

add_executable(robot_pose_ekf 
                       src/odom_estimation.cpp 
                       src/nonlinearanalyticconditionalgaussianodo.cpp 
                       src/odom_estimation_node.cpp)
target_link_libraries(robot_pose_ekf
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${BFL_LIBRARIES}
    )
add_dependencies(robot_pose_ekf robot_pose_ekf_gencpp)

install(
    TARGETS
        robot_pose_ekf
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
