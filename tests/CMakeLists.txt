add_executable(userial_c test_c_compilation.c)
add_executable(userial_cpp test_cpp_compilation.cpp)

target_compile_options(userial_c PRIVATE "-Wall")
target_compile_options(userial_cpp PRIVATE "-Wall")