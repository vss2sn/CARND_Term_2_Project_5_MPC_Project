project(MPC)

cmake_minimum_required (VERSION 3.5)

# -g allows for gdb debugging
# turn on -03 for best performance
add_definitions(-std=c++11 -O3)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(sources src/MPC.cpp src/main.cpp)

set(uWS_CUSTOM_INSTALL_DIR "YES")  # NO if system-wide install

include_directories(/usr/local/include)
link_directories(/usr/local/lib)
include_directories(src/Eigen-3.3)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

# NOTE:
# Minor addition for convenience, I have not installed
# the uWebSockets library system wide.
if(${uWS_CUSTOM_INSTALL_DIR} MATCHES "YES")

include_directories(../uWebSockets/bin/include)
link_directories(../uWebSockets/bin/lib64)

include_directories(../Ipopt-3.12.7/install/include)
link_directories(../Ipopt-3.12.7/install/lib)

endif(${uWS_CUSTOM_INSTALL_DIR} MATCHES "YES")

add_executable(mpc ${sources})

target_link_libraries(mpc ipopt z ssl uv uWS)
