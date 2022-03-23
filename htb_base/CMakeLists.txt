cmake_minimum_required(VERSION 3.0.2)
project(htb_base)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  roscpp
  roslaunch
  hardware_interface
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  	controller_manager 
  	roscpp 
  	hardware_interface
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/htb_base.cpp
  )

add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)
