cmake_minimum_required(VERSION 2.8.3)
project(Sem2Vec)

set(CMAKE_CXX_STANDARD 14)

find_package(Boost)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
add_definitions(${Boost_DEFINITIONS})

find_package(PCL)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

include_directories(core/include
                    common/include
                    modules/cloud_io/include
                    modules/cloud_cluster/include)

add_library(lib_cloud_io
            modules/cloud_io/src/cloud_io.cpp)

add_library(lib_dbscan
            modules/cloud_cluster/src/dbscan.cpp)

list(INSERT _VECTORMAP_LIBRARIES_ 0 ${PCL_LIBRARIES})
list(INSERT _VECTORMAP_LIBRARIES_ 0 ${Boost_LIBRARIES})
list(INSERT _VECTORMAP_LIBRARIES_ 0 ${OpenCV_LIBRARIES})
list(INSERT _VECTORMAP_LIBRARIES_ 0 glog)
list(INSERT _VECTORMAP_LIBRARIES_ 0 lib_cloud_io)
list(INSERT _VECTORMAP_LIBRARIES_ 0 lib_dbscan)

add_executable(main main.cpp)
target_link_libraries(main ${_VECTORMAP_LIBRARIES_})