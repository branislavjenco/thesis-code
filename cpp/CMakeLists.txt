cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cpp_utils)

set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.5 REQUIRED)

find_package(Boost 1.53.0 REQUIRED COMPONENTS filesystem program_options)

include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(get_plane_distances src/get_plane_distances.cpp)
add_executable(set_laser_id_as_label src/set_laser_id_as_label.cpp)
add_executable(get_centroids src/get_centroids.cpp)

target_link_libraries(get_plane_distances ${PCL_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(set_laser_id_as_label ${PCL_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(get_centroids ${PCL_LIBRARIES} ${Boost_LIBRARIES})
