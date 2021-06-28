cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project(cloud-plane-seg LANGUAGES CXX)
set(project_sources main.cpp)

find_package(PCL 1.8.1 REQUIRED)
find_package(Boost COMPONENTS program_options filesystem REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#boost: link_directories not required as Boost_LIBRARIES fully qualified paths

add_executable(${PROJECT_NAME} ${project_sources})
target_link_libraries (${PROJECT_NAME} ${PCL_LIBRARIES} ${Boost_LIBRARIES})
