project(ICP_SET LANGUAGES CXX)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

set(DIR_SRC ./icp.cpp)
set(DIR_HEADER ./icp.h)

add_library(${PROJECT_NAME} ${DIR_SRC} ${DIR_HEADER})

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} )

