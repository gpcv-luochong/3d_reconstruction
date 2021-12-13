cmake_minimum_required(VERSION 2.6)
project(point2mesh)

find_package(PCL 1.12 REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(point2mesh point2mesh.cpp)

target_link_libraries(point2mesh ${OpenCV_LIBS})

target_link_libraries (point2mesh ${PCL_LIBRARIES})

install(TARGETS point2mesh RUNTIME DESTINATION bin)

