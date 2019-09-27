cmake_minimum_required(VERSION 2.8.3)
project(kvaser)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        visualization_msgs
        autoware_can_msgs
        )

catkin_package(
        CATKIN_DEPENDS std_msgs visualization_msgs autoware_can_msgs
)

###########
## Build ##
###########

include_directories(
        ${catkin_INCLUDE_DIRS}
)

if (EXISTS "/usr/include/canlib.h")
    add_executable(can_listener nodes/can_listener/can_listener.cpp)
    target_include_directories(can_listener PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(can_listener ${catkin_LIBRARIES})
    target_link_libraries(can_listener canlib)
    add_dependencies(can_listener ${catkin_EXPORTED_TARGETS})
else ()
    message("'canlib' is not installed. 'can_listener' is not built.")
endif ()

add_executable(can_Mobileye_Decoder nodes/can_Mobileye_Decoder/can_Mobileye_Decoder.cpp)
target_include_directories(can_Mobileye_Decoder PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(can_Mobileye_Decoder ${catkin_LIBRARIES})
add_dependencies(can_Mobileye_Decoder ${catkin_EXPORTED_TARGETS})

add_executable(can_converter nodes/can_converter/can_converter.cpp)
target_include_directories(can_converter PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(can_converter ${catkin_LIBRARIES})
add_dependencies(can_converter ${catkin_EXPORTED_TARGETS})

add_executable(can_draw nodes/can_draw/can_draw.cpp)
target_include_directories(can_draw PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(can_draw ${catkin_LIBRARIES})
add_dependencies(can_draw ${catkin_EXPORTED_TARGETS})

add_executable(can_tx_to_raw nodes/can_tx_to_raw/can_tx_to_raw.cpp)
target_include_directories(can_tx_to_raw PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(can_tx_to_raw ${catkin_LIBRARIES})
add_dependencies(can_tx_to_raw ${catkin_EXPORTED_TARGETS})

add_executable(GPS_tude_to_XY nodes/GPS_tude_to_XY/GPS_tude_to_XY.cpp)
target_include_directories(GPS_tude_to_XY PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(GPS_tude_to_XY ${catkin_LIBRARIES})
add_dependencies(GPS_tude_to_XY ${catkin_EXPORTED_TARGETS})





