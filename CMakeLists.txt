cmake_minimum_required(VERSION 2.8.3)
project(flight_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  common_srv
)

add_message_files(
  FILES
  PID_param.msg
  MRFT_param.msg
  SM_param.msg
  Waypoint.msg
  Info.msg
)

add_service_files(
  FILES
  Arm.srv
  Update_Controller_PID.srv
  Update_Controller_MRFT.srv
  Update_Controller_SM.srv
  Reset_Controller.srv
  Update_Pose_Reference.srv
  Update_X_Reference.srv
  Update_Y_Reference.srv
  Update_Z_Reference.srv
  Update_Yaw_Reference.srv
  SwitchBlock.srv
  Restricted_Norm_Settings.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs message_runtime common_srv
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/Navio2
  ${CMAKE_CURRENT_SOURCE_DIR}/include/Navio2/Common
  ${catkin_INCLUDE_DIRS}
)

FILE(GLOB SOURCE_FILES *.hpp *.cpp *.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Navio2_cpp/*.cpp
        )

add_executable(${PROJECT_NAME} flight_controller_node/flight_controller_node.cpp ${SOURCE_FILES})
add_executable(providers_node providers_node/providers_node.cpp ${SOURCE_FILES})
add_executable(waypoint_reference_node waypoint_reference_node/waypoint_reference_node.cpp ${SOURCE_FILES})
add_executable(global2inertial_node global2inertial_node/global2inertial_node.cpp ${SOURCE_FILES})
add_executable(example_flight_controller_node example_node/example_node.cpp ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(providers_node ${catkin_LIBRARIES})
target_link_libraries(waypoint_reference_node ${catkin_LIBRARIES})
target_link_libraries(global2inertial_node ${catkin_LIBRARIES})
target_link_libraries(example_flight_controller_node ${catkin_LIBRARIES})

add_dependencies(flight_controller flight_controller_generate_messages_cpp)
add_dependencies(providers_node flight_controller_generate_messages_cpp)
add_dependencies(waypoint_reference_node flight_controller_generate_messages_cpp)
add_dependencies(global2inertial_node flight_controller_generate_messages_cpp)
add_dependencies(example_flight_controller_node flight_controller_generate_messages_cpp)
