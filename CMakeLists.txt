cmake_minimum_required(VERSION 2.8)
project(draw_trajectory LANGUAGES CXX)

find_package(PCL REQUIRED)
include_directories(
${PCL_INCLUDE_DIRS})

file(GLOB_RECURSE DIR_SRCS    ${CMAKE_CURRENT_SOURCE_DIR}/src/*)
file(GLOB_RECURSE DIR_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/*)


add_executable(
${PROJECT_NAME} 
${DIR_SRCS}
${DIR_HEADERS}
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)

target_link_libraries(${PROJECT_NAME}
		${PCL_LIBRARIES}
)



