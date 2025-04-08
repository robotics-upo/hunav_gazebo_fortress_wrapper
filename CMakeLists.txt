cmake_minimum_required(VERSION 3.8)
project(hunav_gazebo_fortress_wrapper VERSION 1.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(hunav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ros_gz REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
# # Needed in launch/gz_sim.launch.py.in
set(GZ_SIM_VER ${gz-sim_VERSION_MAJOR})

ign_find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
#set(IGN_COMMON_VER ${ignition-common_VERSION_MAJOR})


include_directories(include)
include_directories(SYSTEM
  ${TINYXML2_INCLUDE_DIR}
)

link_directories(
  ${TINYXML2_LIBRARY_DIRS}
)

# add_library(HuNavActorPluginIGN SHARED src/HuNavActorPlugin_fortress.cpp)
# target_include_directories(HuNavActorPluginIGN PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# $<INSTALL_INTERFACE:include>)
# target_link_libraries(HuNavActorPluginIGN 
#     ${TINYXML2_LIBRARY}
#     #ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
#     ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
# )     

# ament_target_dependencies(HuNavActorPluginIGN 
#   rclcpp    
#   ros_gz
#   #gz-sim6
#   ignition-gazebo6
#   tf2_geometry_msgs
#   hunav_msgs 
# ) 


add_library(HuNavSystemPluginIGN SHARED src/HuNavSystemPlugin_fortress.cpp)
target_include_directories(HuNavSystemPluginIGN PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_link_libraries(HuNavSystemPluginIGN 
    ${TINYXML2_LIBRARY}
    #ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
)     

ament_target_dependencies(HuNavSystemPluginIGN 
  rclcpp    
  ros_gz
  #gz-sim6
  ignition-gazebo6
  tf2_geometry_msgs
  hunav_msgs 
) 


install(TARGETS HuNavSystemPluginIGN #HuNavActorPluginIGN 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  #LIBRARY DESTINATION /usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins
  LIBRARY DESTINATION ${ignition-gazebo6_PLUGIN_PATH}
  RUNTIME DESTINATION bin
)

# file(COPY TARGET HuNavPluginIGN
#   DESTINATION /usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins
# )

# # Instalar el plugin para Gazebo Fortress
# install(TARGETS HuNavActorPluginIGN
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )


add_executable(hunav_gazebo_world_generator src/WorldGenerator.cpp)
ament_target_dependencies(hunav_gazebo_world_generator rclcpp hunav_msgs geometry_msgs tf2_geometry_msgs tf2 tf2_ros tinyxml2_vendor)
target_link_libraries(hunav_gazebo_world_generator ${TINYXML2_LIBRARY})
install(TARGETS hunav_gazebo_world_generator
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY include/
        DESTINATION include
)

install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY worlds/
    DESTINATION share/${PROJECT_NAME}/worlds
)

# install(DIRECTORY media/models
#     DESTINATION share/${PROJECT_NAME}/
# )


ament_package()
