# include(ExternalProject)
# 
# 
# cmake_minimum_required(VERSION 3.0)
# project(rust_ffi)
# 
# add_executable(rust_ffi cpp/main.cpp)
# 
# set_directory_properties(PROPERTIES EP_PREFIX ${CMAKE_BINARY_DIR}/Rust)
# 
# # Add rust_kf
# ExternalProject_Add(
#     rust_kf
#     DOWNLOAD_COMMAND ""
#     CONFIGURE_COMMAND ""
#     BUILD_COMMAND cargo build COMMAND cargo build --release
#     BINARY_DIR "${CMAKE_SOURCE_DIR}/rust"
#     INSTALL_COMMAND ""
#     LOG_BUILD ON)
# 
# add_dependencies(rust_ffi rust_kf)
# 
# # link rust_kf's library files to the cpp executable 
# target_link_libraries(rust_ffi
#     debug "${CMAKE_SOURCE_DIR}/rust/target/debug/kalman_rs.lib"
#     optimized "${CMAKE_SOURCE_DIR}/rust/target/release/kalman_rs.lib"
#     ws2_32 userenv advapi32)
# 
# set_target_properties(rust_ffi PROPERTIES CXX_STANDARD 11 CXX_STANDARD_REQUIRED ON)
# 


include(ExternalProject)


cmake_minimum_required(VERSION 3.0)
project(rust_ffi)

add_executable(rust_ffi src/ffi/main.cpp)

set_directory_properties(PROPERTIES EP_PREFIX ${CMAKE_BINARY_DIR}/Rust)

# Add rust_kf
ExternalProject_Add(
    rust_kf
    DOWNLOAD_COMMAND ""
    CONFIGURE_COMMAND ""
    BUILD_COMMAND cargo build COMMAND cargo build --release
    BINARY_DIR "${CMAKE_SOURCE_DIR}"
    INSTALL_COMMAND ""
    LOG_BUILD ON)

add_dependencies(rust_ffi rust_kf)

# link rust_kf's library files to the cpp executable 
target_link_libraries(rust_ffi
    debug "${CMAKE_SOURCE_DIR}/target/debug/kalman_rs.lib"
    optimized "${CMAKE_SOURCE_DIR}/target/release/kalman_rs.lib"
    ws2_32 userenv advapi32)

set_target_properties(rust_ffi PROPERTIES CXX_STANDARD 11 CXX_STANDARD_REQUIRED ON)


# cd ..  && cargo b && cargo b --release && cd build && cmake .. -G "Visual Studio 15 2017 Win64" && msbuild ALL_BUILD.vcxproj