cmake_minimum_required(VERSION 3.5)
project(simulation)
set(CMAKE_VERBOSE_MAKEFILE ON)
# Set CMAKE_BUILD_TYPE
set(CMAKE_BUILD_TYPE Release)
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(ALIAS_ROS2_TF2)
  add_definitions(-Dtf2=ros2_tf2)
  add_definitions(-Dtf2_ros=ros2_tf2_ros)
  add_definitions(-Dmessage_filters=ros2_message_filters)
endif()
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package( rclcpp REQUIRED)
find_package( std_msgs REQUIRED)
include_directories("include/simulation/matlab/toolbox/coder/rtiostream/src")
include_directories("include/simulation/matlab/toolbox/coder/rtiostream/src/utils")
include_directories("include/simulation")
set (CMAKE_SKIP_BUILD_RPATH false)
set (CMAKE_BUILD_WITH_INSTALL_RPATH true)
set (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set (CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
# Node
add_executable(Simulation 
src/ext_svr.c
src/ext_work.c
src/rtiostream_interface.c
src/updown.c
src/rtiostream_tcpip.c
src/rtiostream_utils.c
src/Simulation.cpp
src/Simulation_data.cpp
src/main.cpp
src/ros2nodeinterface.cpp
src/rtGetInf.cpp
src/rtGetNaN.cpp
src/rt_nonfinite.cpp
src/slros2_initialize.cpp
src/slros_busmsg_conversion.cpp
)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
target_compile_options(Simulation PUBLIC 
  $<$<OR:$<COMPILE_LANGUAGE:CXX>,$<COMPILE_LANGUAGE:C>>:
  -Wall -Wextra -Wpedantic
  >)
endif()
target_compile_options(Simulation PRIVATE
  $<$<OR:$<COMPILE_LANGUAGE:CXX>,$<COMPILE_LANGUAGE:C>>:
  -DMODEL=Simulation
  -DNUMST=2
  -DNCSTATES=12
  -DHAVESTDIO
  -DMODEL_HAS_DYNAMICALLY_LOADED_SFCNS=0
  -DON_TARGET_WAIT_FOR_START=1
  -DCLASSIC_INTERFACE=0
  -DALLOCATIONFCN=0
  -DTID01EQ=1
  -DEXT_MODE=1
  -DONESTEPFCN=1
  -DTERMFCN=1
  -DMULTI_INSTANCE_CODE=0
  -DINTEGER_CODE=0
  -DMT=0
  -DROS2_PROJECT
  -DSTACK_SIZE=64
  -D__MW_TARGET_USE_HARDWARE_RESOURCES_H__
  -DRT=RT
  -fpermissive  >
  )
ament_target_dependencies(
  Simulation
  "rclcpp"
  "std_msgs"
)
target_include_directories(Simulation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(Simulation 
	${CMAKE_DL_LIBS}
    )
install(TARGETS Simulation
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})
ament_export_include_directories(
  include
)
foreach(file_i ${CUDA_BINS})
add_custom_command(
                   TARGET Simulation
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND}
                   ARGS -E copy ${PROJECT_SOURCE_DIR}/src/${file_i} ${CMAKE_INSTALL_PREFIX}/Simulation_ert_rtw/${file_i}
				   )
add_custom_command(
                   TARGET Simulation
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND}
                   ARGS -E copy ${PROJECT_SOURCE_DIR}/src/${file_i} ${CMAKE_INSTALL_PREFIX}/codegen/exe/Simulation/${file_i}
				   )				   
endforeach(file_i)
ament_package()
# Generated on Sat May 03 19:07:51 2025
