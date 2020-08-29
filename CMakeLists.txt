cmake_minimum_required(VERSION 3.10)
project(bit_related)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED core highgui imgproc imgcodecs)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        include/
)

set(planner
        include/planner_core.h
        src/planner_core.cpp

        include/RRT.h
        src/RRT.cpp
        )

set(common
        include/Point2d.h
        include/Linesegment2d.h
        include/AABB2d.h
        include/OBB2d.h
        include/Node2d.h

        src/Point2d.cpp
        src/Linesegment.cpp
        src/AABB2d.cpp
        src/OBB2d.cpp
        )

SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

add_executable(map_generator src/map_generator.cpp)
target_link_libraries(map_generator ${OpenCV_LIBS})

add_executable(main src/main.cpp ${common} ${planner})
target_link_libraries(main ${OpenCV_LIBS})

add_executable(OBB_test src/OBB_test.cpp ${common})