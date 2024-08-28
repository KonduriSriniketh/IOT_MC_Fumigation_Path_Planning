########### 版本要求
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 14)

########### 项目名称
project(astar)

########### 添加系统库
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        geometry_msgs
        nav_msgs
        tf
        )

catkin_package()

########### 添加头文件
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

########### 编译可执行文件
add_executable(${PROJECT_NAME} src/main.cpp src/Astar.cpp src/OccMapTransform.cpp)
add_executable(astar_path_distances src/path_distances.cpp src/Astar.cpp src/OccMapTransform.cpp)
add_executable(combined_path_publisher src/combined_path_publisher.cpp src/Astar.cpp src/OccMapTransform.cpp)
add_executable(optimised_out_path src/optimised_out_path.cpp src/Astar.cpp src/OccMapTransform.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(astar_path_distances ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(combined_path_publisher ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(optimised_out_path ${catkin_LIBRARIES} ${OpenCV_LIBS})
