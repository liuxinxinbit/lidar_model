cmake_minimum_required(VERSION 2.8.3)
project(lidar_model)


find_package(catkin REQUIRED COMPONENTS
  roscpp 
  rospy 
  cv_bridge 
  image_transport 
  std_msgs 
  genmsg 
  message_generation
)
find_package(PCL REQUIRED)
find_package(VTK REQUIRED)
find_package(OpenCV REQUIRED)

add_message_files(FILES 
  lidar_process_data.msg 
  boatpose.msg
  xyz_msg.msg
  stereo_vision_msg.msg
  obstacal_msg.msg
)

generate_messages(DEPENDENCIES 
  std_msgs
)
catkin_package(

)


add_executable(lidar_model_generate src/lidar_model_generate.cpp)
target_link_libraries(lidar_model_generate ${catkin_LIBRARIES})

add_executable(target_tracking_show src/target_tracking_show.cpp)
target_link_libraries(target_tracking_show ${catkin_LIBRARIES})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
)


