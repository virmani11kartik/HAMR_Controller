cmake_minimum_required(VERSION 3.16)
project(ekf_imu_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE   # quiets CMP0135 warning
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

# Common includes for both tests
set(PROJ_INCLUDES
  ${CMAKE_SOURCE_DIR}/src
  ${CMAKE_SOURCE_DIR}/tests/arduino_shim
)

# ---- test_ekf ----
add_executable(test_ekf
  tests/test_ekf.cpp
  tests/test_shims.cpp
  src/ekf_localization.h
  src/odometry.h
)
target_include_directories(test_ekf PRIVATE ${PROJ_INCLUDES})
target_link_libraries(test_ekf PRIVATE GTest::gtest_main)
add_test(NAME test_ekf COMMAND test_ekf)

# ---- test_odometry ----
add_executable(test_odometry
  tests/test_odometry.cpp
  tests/test_shims.cpp
  src/odometry.h
)
target_include_directories(test_odometry PRIVATE ${PROJ_INCLUDES})
target_link_libraries(test_odometry PRIVATE GTest::gtest_main)
add_test(NAME test_odometry COMMAND test_odometry)
