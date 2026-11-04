cmake_minimum_required(VERSION 3.16)
project(ekf2_golden_cpp LANGUAGES CXX)

if(NOT PX4_SOURCE_ROOT OR NOT HOST_INCLUDE)
    message(FATAL_ERROR "Run tools/golden_cpp/build.py to prepare the fixed sources")
endif()

set(EKF "${PX4_SOURCE_ROOT}/src/modules/ekf2/EKF")
if(NOT REFERENCE_PROFILE)
    set(REFERENCE_PROFILE core_imu)
endif()
if(NOT REFERENCE_PROFILE STREQUAL "core_imu" AND NOT REFERENCE_PROFILE STREQUAL "baro_mag")
    message(FATAL_ERROR "Unsupported fixed reference profile")
endif()
# Unconditional sources match the pinned EKF/CMakeLists.txt.
add_library(px4_ekf STATIC
    ${EKF}/control.cpp
    ${EKF}/covariance.cpp
    ${EKF}/ekf.cpp
    ${EKF}/ekf_helper.cpp
    ${EKF}/estimator_interface.cpp
    ${EKF}/height_control.cpp
    ${EKF}/velocity_fusion.cpp
    ${EKF}/position_fusion.cpp
    ${EKF}/yaw_fusion.cpp
    ${EKF}/imu_down_sampler/imu_down_sampler.cpp
    ${EKF}/aid_sources/fake_height_control.cpp
    ${EKF}/aid_sources/fake_pos_control.cpp
    ${EKF}/aid_sources/ZeroGyroUpdate.cpp
    ${EKF}/aid_sources/ZeroVelocityUpdate.cpp
    ${EKF}/aid_sources/zero_innovation_heading_update.cpp
    ${EKF}/bias_estimator/bias_estimator.cpp
    ${EKF}/output_predictor/output_predictor.cpp
    ${PX4_SOURCE_ROOT}/src/lib/geo/geo.cpp
    ${PX4_SOURCE_ROOT}/src/lib/lat_lon_alt/lat_lon_alt.cpp
)
if(REFERENCE_PROFILE STREQUAL "baro_mag")
    target_sources(px4_ekf PRIVATE
        ${EKF}/aid_sources/barometer/baro_height_control.cpp
        ${EKF}/aid_sources/magnetometer/mag_control.cpp
        ${EKF}/aid_sources/magnetometer/mag_fusion.cpp
        ${PX4_SOURCE_ROOT}/src/lib/world_magnetic_model/geo_mag_declination.cpp
    )
    target_compile_definitions(px4_ekf PUBLIC CONFIG_EKF2_BAROMETER CONFIG_EKF2_MAGNETOMETER)
endif()
target_include_directories(px4_ekf PUBLIC
    ${HOST_INCLUDE}
    ${PX4_SOURCE_ROOT}/platforms/common/include
    ${PX4_SOURCE_ROOT}/src
    ${PX4_SOURCE_ROOT}/src/lib
    ${PX4_SOURCE_ROOT}/src/lib/matrix
    ${EKF}
    ${EKF}/python
)
target_compile_features(px4_ekf PUBLIC cxx_std_17)
target_compile_definitions(px4_ekf PUBLIC __PX4_POSIX)
target_compile_options(px4_ekf PUBLIC
    -include px4_platform_common/defines.h
    -fno-fast-math -fno-associative-math -ffp-contract=off
    -fno-strict-aliasing # Same as pinned cmake/px4_add_common_flags.cmake.
)

add_executable(golden_cpp main.cpp)
target_link_libraries(golden_cpp PRIVATE px4_ekf)
target_compile_options(golden_cpp PRIVATE -Wall -Wextra -Werror)
if(REFERENCE_PROFILE STREQUAL "baro_mag")
    add_executable(golden_cpp_aids main.cpp)
    target_link_libraries(golden_cpp_aids PRIVATE px4_ekf)
    target_compile_definitions(golden_cpp_aids PRIVATE DUMP_AID_STATUS)
    target_compile_options(golden_cpp_aids PRIVATE -Wall -Wextra -Werror)
endif()
