cmake_minimum_required(VERSION 2.6)
project(sgbm)

find_package(PCL 1.12 REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(sgbm sgbm.cpp)

target_link_libraries(sgbm ${OpenCV_LIBS})

target_link_libraries (sgbm ${PCL_LIBRARIES})

install(TARGETS sgbm RUNTIME DESTINATION bin)

