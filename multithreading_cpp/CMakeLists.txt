cmake_minimum_required(VERSION 3.0.2)
project(multi_thread_test)

SET(CMAKE_BUILD_TYPE Release)
#  -pthread -fopenmp --> threading flags
add_compile_options(-Wall -O3 -march=native -lpthread -std=c++2a)

# Eigen
find_package(Eigen3 REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# Threads
find_package(Threads REQUIRED)

set(INCLUDE_DIRS
    "/usr/include/eigen3"
    ${OpenCV_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
)

set(THIRD_PARTY_LIBS
    ${OpenCV_LIBS}
    Threads::Threads
)

include_directories(${INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node
    # src/lambda_main.cc
    # src/async_main.cc
    src/mutex/race_condition_main.cc
)

target_link_libraries(${PROJECT_NAME}_node 
${THIRD_PARTY_LIBS})
