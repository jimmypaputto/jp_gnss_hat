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

# gnsshat — jimmypaputto/gnsshat driver (https://github.com/jimmypaputto/gnsshat)
# Uses system-installed version if available, otherwise fetches from GitHub.
# System dependency: libgpiod-dev (sudo apt install libgpiod-dev)
find_library(GNSSHAT_LIBRARY gnsshat)
find_path(GNSSHAT_INCLUDE_DIR jimmypaputto/GnssHat.hpp)

if(GNSSHAT_LIBRARY AND GNSSHAT_INCLUDE_DIR)
    message(STATUS "Found system gnsshat: ${GNSSHAT_LIBRARY}")
else()
    message(STATUS "System gnsshat not found — fetching from GitHub")
    include(FetchContent)
    FetchContent_Declare(gnsshat
        GIT_REPOSITORY https://github.com/jimmypaputto/gnsshat.git
        GIT_SHALLOW TRUE
    )
    FetchContent_GetProperties(gnsshat)
    if(NOT gnsshat_POPULATED)
        FetchContent_Populate(gnsshat)
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

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

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
target_link_libraries(gnss_node gnsshat yaml-cpp)

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

# ---- Unit tests ----
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    set(JP_GNSS_LIB_SOURCES
        src/Config.cpp
        src/Converter.cpp
        src/Rtk.cpp
    )

    macro(jp_gnss_hat_add_test TARGET_NAME TEST_SOURCE)
        ament_add_gtest(${TARGET_NAME} ${TEST_SOURCE} ${JP_GNSS_LIB_SOURCES})
        ament_target_dependencies(${TARGET_NAME}
            rclcpp rclcpp_lifecycle std_msgs sensor_msgs geometry_msgs)
        target_link_libraries(${TARGET_NAME}
            gnsshat yaml-cpp "${cpp_typesupport_target}")
        if(GNSSHAT_FETCHED)
            target_include_directories(${TARGET_NAME} PRIVATE "${_gnsshat_inc}")
        endif()
        if(rtcm_msgs_FOUND)
            target_compile_definitions(${TARGET_NAME} PRIVATE JP_GNSS_NTRIP)
            ament_target_dependencies(${TARGET_NAME} rtcm_msgs)
        endif()
    endmacro()

    jp_gnss_hat_add_test(test_config    test/test_config.cpp)
    jp_gnss_hat_add_test(test_converter test/test_converter.cpp)
    jp_gnss_hat_add_test(test_rtk       test/test_rtk.cpp)
endif()

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
    install(TARGETS gnsshat LIBRARY DESTINATION lib)
endif()

ament_package()
