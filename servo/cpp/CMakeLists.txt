add_library(servo_bus STATIC src/bus.cpp)
target_include_directories(servo_bus PUBLIC include)
target_compile_features(servo_bus PUBLIC cxx_std_17)
