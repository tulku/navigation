cmake_minimum_required(VERSION 2.8.3)
project(fake_localization)

find_package(catkin REQUIRED
        COMPONENTS
            angles
            roscpp
            rosconsole
            nav_msgs
            geometry_msgs
            tf
            message_filters
        )
find_package(Boost REQUIRED COMPONENTS signals)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/fake_localization.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${PROJECT_NAME} nav_msgs_gencpp)
add_dependencies(${PROJECT_NAME} geometry_msgs_gencpp)

add_executable(${PROJECT_NAME}_node src/fake_localization_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_node nav_msgs_gencpp)
add_dependencies(${PROJECT_NAME}_node geometry_msgs_gencpp)

install(
    TARGETS
      ${PROJECT_NAME}
      ${PROJECT_NAME}_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

