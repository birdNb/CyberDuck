add_executable(duck_locomotion
    src/layout.cpp
    src/gamepad.cpp
    src/engine.cpp
    src/main.cpp
)
target_include_directories(duck_locomotion PRIVATE
    include
    ${CMAKE_SOURCE_DIR}/locomotion/third_party/onnxruntime
)
target_link_libraries(duck_locomotion PRIVATE motor_can imu_lsm6 dl)
target_compile_features(duck_locomotion PRIVATE cxx_std_17)
