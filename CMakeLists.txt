cmake_minimum_required(VERSION 3.16)
project(kalman_filter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)
find_package(Eigen3 REQUIRED)

add_executable(kalman_filter src/kalman_filter/kalman_filter.cpp)
add_executable(extended_kalman_filter src/extended_kalman_filter/extended_kalman_filter.cpp)

target_include_directories(kalman_filter PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${Python3_INCLUDE_DIRS}
    ${Python3_NumPy_INCLUDE_DIRS}
)

target_link_libraries(kalman_filter PRIVATE
    ${Python3_LIBRARIES}
    Eigen3::Eigen
)

target_include_directories(extended_kalman_filter PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${Python3_INCLUDE_DIRS}
    ${Python3_NumPy_INCLUDE_DIRS}
)

target_link_libraries(extended_kalman_filter PRIVATE
    ${Python3_LIBRARIES}
    Eigen3::Eigen
)
