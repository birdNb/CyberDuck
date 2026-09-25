add_library(motor_can STATIC
    src/protocol.cpp
    src/bus.cpp
)
target_include_directories(motor_can PUBLIC include)
target_compile_features(motor_can PUBLIC cxx_std_17)
