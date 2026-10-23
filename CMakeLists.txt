cmake_minimum_required(VERSION 3.16)
project(zed_open_capture_ros)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
pkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)

include(GNUInstallDirs)
include(FetchContent)

# Open Capture uses HIDAPI for the ZED 2/2i sensor module. Build the small
# hidraw backend locally so the package does not alter the host Python/CUDA
# environment and does not require a system-wide HIDAPI development package.
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(HIDAPI_WITH_HIDRAW ON CACHE BOOL "" FORCE)
set(HIDAPI_WITH_LIBUSB OFF CACHE BOOL "" FORCE)
set(HIDAPI_WITH_TESTS OFF CACHE BOOL "" FORCE)
set(HIDAPI_BUILD_HIDTEST OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  hidapi
  GIT_REPOSITORY https://github.com/libusb/hidapi.git
  GIT_TAG hidapi-0.14.0
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(hidapi)

# Prefer the adjacent checkout for local development. Fall back to the pinned
# upstream revision so this ROS package can also be built on its own.
get_filename_component(ZOC_SIBLING "${CMAKE_CURRENT_SOURCE_DIR}/../zed-open-capture" ABSOLUTE)
if(EXISTS "${ZOC_SIBLING}/src/videocapture.cpp")
  set(ZOC_SOURCE_DIR "${ZOC_SIBLING}")
else()
  FetchContent_Declare(
    zed_open_capture
    GIT_REPOSITORY https://github.com/stereolabs/zed-open-capture.git
    GIT_TAG 93739154903a66c112ee52c3c6fbfd83b7218617
  )
  FetchContent_GetProperties(zed_open_capture)
  if(NOT zed_open_capture_POPULATED)
    FetchContent_Populate(zed_open_capture)
  endif()
  set(ZOC_SOURCE_DIR "${zed_open_capture_SOURCE_DIR}")
endif()

add_library(zed_open_capture_vendor STATIC
  "${ZOC_SOURCE_DIR}/src/videocapture.cpp"
  "${ZOC_SOURCE_DIR}/src/sensorcapture.cpp"
)
target_compile_features(zed_open_capture_vendor PUBLIC cxx_std_14)
target_compile_definitions(zed_open_capture_vendor PUBLIC
  VIDEO_MOD_AVAILABLE
  SENSORS_MOD_AVAILABLE
)
target_include_directories(zed_open_capture_vendor PUBLIC
  "${ZOC_SOURCE_DIR}/include"
)
target_link_libraries(zed_open_capture_vendor PUBLIC
  hidapi::hidraw
  PkgConfig::LIBUSB
  Threads::Threads
)
set_target_properties(zed_open_capture_vendor PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

add_executable(zed_open_capture_node src/zed_open_capture_node.cpp)
add_executable(stereo_splitter_node src/stereo_splitter_node.cpp)
target_compile_features(zed_open_capture_node PRIVATE cxx_std_17)
target_compile_options(zed_open_capture_node PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(zed_open_capture_node SYSTEM PRIVATE
  "${ZOC_SOURCE_DIR}/examples/include"
)
target_include_directories(zed_open_capture_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(zed_open_capture_node
  geometry_msgs
  rclcpp
  sensor_msgs
  tf2_ros
)
target_link_libraries(zed_open_capture_node
  zed_open_capture_vendor
  ${OpenCV_LIBS}
)
target_compile_features(stereo_splitter_node PRIVATE cxx_std_17)
target_compile_options(stereo_splitter_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(stereo_splitter_node rclcpp sensor_msgs)

install(TARGETS zed_open_capture_node stereo_splitter_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/record_stereo.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
