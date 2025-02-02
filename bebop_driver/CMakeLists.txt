cmake_minimum_required(VERSION 3.5)
project(bebop_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(parrot_arsdk REQUIRED)
find_package(bebop_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)

message(STATUS "Catkin INCLUE_DIR: ${catkin_INCLUDE_DIRS}")
message(STATUS "Catkin LIBS: ${catkin_LIBRARIES}")
message(STATUS "ARSDK3 INCLUDE_DIR: ${ARSDK_INCLUDE_DIR}")
message(STATUS "ARSDK3 LIB_DIR: ${ARSDK_LIB_DIR}")
message(STATUS "ARSDK3 LIBS: ${ARSDK_LIBS}")

find_package(Boost REQUIRED)
pkg_check_modules(AVCODEC REQUIRED libavcodec)
pkg_check_modules(AVFORMAT REQUIRED libavformat)
pkg_check_modules(SWSCALE REQUIRED libswscale)

include_directories(include 
                    ${ARSDK_INCLUDE_DIR}
                    ${Boost_INCLUDE_DIRS}
                    /home/shaswatgarg/bebop_ws/install/parrot_arsdk/include
                    ${AVCODEC_INCLUDE_DIRS}
                    ${AVFORMAT_INCLUDE_DIRS}
                    ${SWSCALE_INCLUDE_DIRS})

add_library(bebop SHARED src/bebop.cpp)
ament_target_dependencies(bebop rclcpp parrot_arsdk bebop_msgs)
target_link_libraries(bebop
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${ARSDK_LIBS}
                      /home/shaswatgarg/bebop_ws/install/parrot_arsdk/lib/parrot_arsdk
                      ${AVCODEC_LIBRARIES}
                      ${AVFORMAT_LIBRARIES}
                      ${SWSCALE_LIBRARIES})

add_library(bebop_driver_component SHARED src/bebop_driver_component.cpp)
target_compile_definitions(bebop_driver_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(bebop_driver_component rclcpp std_msgs parrot_arsdk bebop_msgs nav_msgs tf2_geometry_msgs tf2_ros geometry_msgs image_transport camera_info_manager)
target_link_libraries(bebop_driver_component
                      ${catkin_LIBRARIES}
                      bebop)

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

rclcpp_components_register_nodes(bebop_driver_component "bebop_driver::BebopDriverComponent")

# Install CPP executables:
install(TARGETS
  bebop
  bebop_driver_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES component_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
