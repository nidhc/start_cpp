cmake_minimum_required(VERSION 3.20)
project(start)

find_package(OpenCV REQUIRED)
find_package(PCL 1.12 REQUIRED)
find_package(VTK REQUIRED)
find_package(realsense2 REQUIRED)

add_definitions("-fpermissive")
set(CMAKE_CXX_STANDARD 14)
set(MAIN_FILES main.cpp)


add_executable(start ${MAIN_FILES} )


###添加opencv pcl vtk的依赖###
include(${VTK_USE_FILE})
include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


###添加机械臂依赖，其中inc/ inc/include/ lib/中储存机械臂依赖的包###
include_directories(
        ${PROJECT_SOURCE_DIR}/inc
        ${PROJECT_SOURCE_DIR}/inc/include
)
link_directories(
        ${PROJECT_SOURCE_DIR}/lib)


###添加mech相机依赖###
include(FindPkgConfig)
pkg_check_modules(PKGS REQUIRED MechEyeApi)
include_directories(${PKGS_INCLUDE_DIRS})
link_directories(${PKGS_LIBRARY_DIRS})


target_link_libraries (start
        ${PCL_LIBRARIES}
        ${VTK_LIBRARIES}
        ${OpenCV_LIBS}
        ${realsense2_LIBRARY}
        MechEyeApi
        RCI_Client.a
        xMateModel.a
        orocos-kdl.a
        glog.a)

