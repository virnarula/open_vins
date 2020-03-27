cmake_minimum_required(VERSION 2.8.8)

# Project name
project(ov_core)

# Include our cmake files
#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/)

# Find catkin (the ROS build system)
# find_package(catkin REQUIRED COMPONENTS roscpp rosbag tf std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs cv_bridge)

# Include libraries
find_package(Eigen3 REQUIRED)
find_package(OpenCV 3.4 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)

# display message to user
message(STATUS "EIGEN VERSION: " ${EIGEN3_VERSION})
message(STATUS "OPENCV VERSION: " ${OpenCV_VERSION})
message(STATUS "BOOST VERSION: " ${Boost_VERSION})

# Describe catkin project
# catkin_package(
#         CATKIN_DEPENDS roscpp rosbag tf std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs cv_bridge
#         INCLUDE_DIRS src
#         LIBRARIES ov_core_lib
# )

# Try to compile with c++11
# http://stackoverflow.com/a/25836953
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Enable compile optimizations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Enable debug flags (use if you want to debug in gdb)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -Wmaybe-uninitialized")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -pthread -lboost_date_time")

# Include our header files
include_directories(
        src
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
)

##################################################
# Make the core library
##################################################
add_library(ov_core_lib STATIC
        src/init/InertialInitializer.cpp
        src/track/TrackBase.cpp
        src/track/TrackAruco.cpp
        src/track/TrackDescriptor.cpp
        src/track/TrackKLT.cpp
        src/types/Landmark.cpp
        src/feat/Feature.cpp
        src/feat/FeatureInitializer.cpp
)


# install(TARGETS ov_core_lib DESTINATION lib)
# install(FILES src/track/TrackAruco.h src/track/TrackKLT.h src/track/TrackDescriptor.h src/track/TrackBase.h src/types/Type.h src/types/JPLQuat.h src/types/IMU.h src/types/Landmark.h src/types/PoseJPL.h src/types/Vec.h  DESTINATION include)

target_include_directories(ov_core_lib PUBLIC src)

# target_link_libraries(ov_core_lib ${thirdparty_libraries})

# install(TARGETS ov_core_lib EXPORT OVCoreConfig
#     ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
#     LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
#     RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows
# install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# # This makes the project importable from the install directory
# # Put config file in per-project dir (name MUST match), can also
# # just go into 'cmake'.
# install(EXPORT OVCoreConfig DESTINATION share/OVCore/cmake)

# # This makes the project importable from the build directory
# export(TARGETS ov_core_lib FILE OVCoreConfig.cmake)
