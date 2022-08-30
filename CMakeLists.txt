project(MPC)

cmake_minimum_required (VERSION 3.5)

# -g allows for gdb debugging
# turn on -03 for best performance
add_definitions(-std=c++11 -O3)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(sources src/MPC.cpp src/main.cpp)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)
include_directories(src/Eigen-3.3)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/opt/homebrew/Cellar/openssl@3/3.0.5/include)
include_directories(/opt/homebrew/Cellar/cppad/20220000.4/include)
include_directories(/opt/homebrew/Cellar/ipopt/3.14.9_1/include)
include_directories(/opt/homebrew/Cellar/libuv/1.44.2/include)
link_directories(/opt/homebrew/Cellar/openssl@3/3.0.5/lib)
link_directories(/opt/homebrew/Cellar/libuv/1.44.2/lib)
link_directories(/opt/homebrew/Cellar/ipopt/3.14.9_1/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_executable(mpc ${sources})

target_link_libraries(mpc ipopt z ssl uv uWS)

