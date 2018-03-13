cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cloud_viewer)
find_package( PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} "/usr/include/boost/" "/usr/include/pcl-1.7/" "/usr/include/eigen3/" "/usr/include/opencv2/" "/usr/include/opencv" "/usr/include/vtk-6.2/")
#link_directories("/usr/local/boost_1_59_0/lib" ${PCL_LIBRARY_DIRS} "/usr/lib/x86_64-linux-gnu/"  )
link_directories( "/usr/lib/x86_64-linux-gnu/"  )
add_definitions(${PCL_DEFINITIONS})
add_executable (cloud_viewer cloud_viewer.cpp)
target_link_libraries (cloud_viewer pcl_visualization  ${PCL_LIBRARIES} opencv_core opencv_imgproc opencv_highgui)

