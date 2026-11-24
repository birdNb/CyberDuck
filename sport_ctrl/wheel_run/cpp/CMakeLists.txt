add_executable(wheel_drive
    src/main.cpp
    src/bus.cpp
    src/config.cpp
    src/plant.cpp
    src/getup.cpp
)
target_include_directories(wheel_drive PRIVATE
    include
    ${CMAKE_SOURCE_DIR}/IMU/cpp/include
)
target_link_libraries(wheel_drive PRIVATE imu_lsm6 pthread)
target_compile_features(wheel_drive PRIVATE cxx_std_17)
