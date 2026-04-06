cmake_minimum_required(VERSION 3.11)
project(jp_gnss_hat)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Optional: rtcm_msgs enables standard /rtcm topic bridge
find_package(rtcm_msgs QUIET)
if(rtcm_msgs_FOUND)
    message(STATUS "rtcm_msgs found - standard /rtcm topic bridge enabled")
else()
    message(STATUS "rtcm_msgs not found - only jp_gnss_hat/rtk protocol available")
endif()

# GnssHat — jimmypaputto/GnssHat driver (https://github.com/jimmypaputto/GnssHat)
# Uses system-installed version if available, otherwise fetches from GitHub.
# System dependency: libgpiod-dev (sudo apt install libgpiod-dev)
find_library(GNSSHAT_LIBRARY GnssHat)
find_path(GNSSHAT_INCLUDE_DIR jimmypaputto/GnssHat.hpp)

if(GNSSHAT_LIBRARY AND GNSSHAT_INCLUDE_DIR)
    message(STATUS "Found system GnssHat: ${GNSSHAT_LIBRARY}")
else()
    message(STATUS "System GnssHat not found — fetching from GitHub")
    include(FetchContent)
    FetchContent_Declare(GnssHat
        GIT_REPOSITORY https://github.com/jimmypaputto/GnssHat.git
        GIT_SHALLOW TRUE
    )
    FetchContent_GetProperties(GnssHat)
    if(NOT gnsshat_POPULATED)
        FetchContent_Populate(GnssHat)
        add_subdirectory(${gnsshat_SOURCE_DIR} ${gnsshat_BINARY_DIR} EXCLUDE_FROM_ALL)
    endif()
    set(GNSSHAT_FETCHED TRUE)
endif()

# Generate custom messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Geofence.msg"
    "msg/PositionVelocityTime.msg"
    "msg/DilutionOverPrecision.msg"
    "msg/SatelliteInfo.msg"
    "msg/RfBlock.msg"
    "msg/GeofencingNav.msg"
    "msg/Navigation.msg"
    "msg/Rtcm3Frame.msg"
    "msg/RtkCorrections.msg"
    "msg/GnssConfig.msg"
    "srv/GetGeofencingCfg.srv"
    "srv/GetGnssConfig.srv"
    "srv/SetGnssConfig.srv"
    DEPENDENCIES std_msgs
)

add_executable(
    gnss_node
    src/Config.cpp
    src/Converter.cpp
    src/GnssNode.cpp
    src/Main.cpp
    src/Rtk.cpp
)
ament_target_dependencies(
    gnss_node
    rclcpp
    rclcpp_lifecycle
    lifecycle_msgs
    std_msgs
    sensor_msgs
    geometry_msgs
    ament_index_cpp
)
target_link_libraries(gnss_node GnssHat yaml-cpp)

if(GNSSHAT_FETCHED)
    set(_gnsshat_inc "${CMAKE_CURRENT_BINARY_DIR}/_gnsshat_include")
    if(NOT EXISTS "${_gnsshat_inc}/jimmypaputto")
        file(MAKE_DIRECTORY "${_gnsshat_inc}")
        file(CREATE_LINK "${gnsshat_SOURCE_DIR}/src" "${_gnsshat_inc}/jimmypaputto" SYMBOLIC)
    endif()
    target_include_directories(gnss_node PRIVATE "${_gnsshat_inc}")
endif()

if(rtcm_msgs_FOUND)
    target_compile_definitions(gnss_node PRIVATE JP_GNSS_NTRIP)
    ament_target_dependencies(gnss_node rtcm_msgs)
endif()

# Link generated message headers to executable
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}
    rosidl_typesupport_cpp)
target_link_libraries(gnss_node "${cpp_typesupport_target}")

install(
    DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config/
)

install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch/
)

install(
    TARGETS gnss_node
    DESTINATION lib/${PROJECT_NAME}
)

if(GNSSHAT_FETCHED)
    install(TARGETS GnssHat LIBRARY DESTINATION lib)
endif()

ament_package()
