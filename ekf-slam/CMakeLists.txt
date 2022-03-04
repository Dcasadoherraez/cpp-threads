cmake_minimum_required(VERSION 3.0.2)
project(multi_threaded_ekf)

SET(CMAKE_BUILD_TYPE Release)
#  -pthread -fopenmp --> threading flags
add_compile_options(-Wall -O3 -march=native -std=c++2a -DENABLE_INTERNAL_FMT=ON)

# Eigen
find_package(Eigen3 REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

set(INCLUDE_DIRS
    "/usr/include/eigen3"
    ${OpenCV_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
)

set(THIRD_PARTY_LIBS
    ${OpenCV_LIBS}
)

include_directories(${INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node
    src/main.cpp
    src/ekf.cpp
    src/drawer.cpp
)

target_link_libraries(${PROJECT_NAME}_node 
${THIRD_PARTY_LIBS})
