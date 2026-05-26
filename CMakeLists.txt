cmake_minimum_required(VERSION 3.16)
project(sheep_recon LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL 1.10 REQUIRED COMPONENTS common io filters registration kdtree search surface segmentation features)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(sheep_reconstruct
  src/main.cpp
)
target_include_directories(sheep_reconstruct PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(sheep_reconstruct ${PCL_LIBRARIES})

add_executable(gp3_single
  src/gp3_single.cpp
)
target_include_directories(gp3_single PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(gp3_single ${PCL_LIBRARIES})
