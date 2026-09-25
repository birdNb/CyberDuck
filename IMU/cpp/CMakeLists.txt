add_library(imu_lsm6 STATIC
    src/lsm6.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../i2c_gpio.c
)
target_include_directories(imu_lsm6 PUBLIC include ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_compile_features(imu_lsm6 PUBLIC cxx_std_17)
