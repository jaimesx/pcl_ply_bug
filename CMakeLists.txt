cmake_minimum_required (VERSION 3.5)
project(pcl_io_test)  

find_package(PCL REQUIRED)

#
# build interface: commandline
# 
add_library(interface_commandline INTERFACE)
include_directories(interface_commandline INTERFACE ${PCL_INCLUDE_DIRS})
link_directories(interface_commandline INTERFACE ${PCL_LIBRARY_DIRS})
target_link_libraries(interface_commandline INTERFACE ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${BOOST_LIBRARIES})
target_compile_definitions(interface_commandline INTERFACE ${PCL_DEFINITIONS})  

add_executable(pcl_ply_test pcl_ply_test.cpp)
target_link_libraries(pcl_ply_test interface_commandline)
