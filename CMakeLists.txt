cmake_minimum_required(VERSION 2.8)

project(diameter_estimation CXX)

file(COPY ${CMAKE_SOURCE_DIR}/resource/ DESTINATION ${PROJECT_BINARY_DIR}/resource/)
add_definitions(-DRESOURCE_DIR="${PROJECT_BINARY_DIR}/resource/")

# find_package(PCL 1.8 REQUIRED COMPONENTS common io)
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# if(PCL_FOUND)
#     target_include_directories(Main PUBLIC ${PCL_INCLUDE_DIRS})
#     target_link_libraries(Main PUBLIC ${PCL_LIBS})
# endif()

set(SOURCES
    main.cpp
    diameter_estimation.cpp
)

set(HEADERS
    diameter_estimation.hpp
)

add_executable(a.out ${SOURCES} ${HEADERS})

target_link_libraries(a.out ${PCL_LIBRARIES})

# find_package(OpenCV REQUIRED)
# if(OpenCV_FOUND)
#     target_include_directories(Main PUBLIC ${OpenCV_INCLUDE_DIRS})
#     target_link_libraries(Main ${OpenCV_LIBS})
# endif()
