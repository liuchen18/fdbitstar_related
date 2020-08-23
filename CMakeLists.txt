cmake_minimum_required(VERSION 3.10)
project(bit_related)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED core highgui imgproc imgcodecs)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        include/
)

add_library(planner
        include/motion_plan.h
        include/planner_core.h

        src/motion_plan.cpp
        src/planner_core.cpp

        )

add_library(common
        include/Point2d.h
        include/Linesegment2d.h
        include/AABB2d.h
        include/OBB2d.h

        src/Point2d.cpp
        src/Linesegment.cpp
        src/AABB2d.cpp
        src/OBB2d.cpp
        )

add_executable(main src/motion_plan.cpp)
target_link_libraries(main ${OpenCV_LIBS} ${planner})

add_executable(map_generator src/map_generator.cpp)
target_link_libraries(map_generator ${OpenCV_LIBS})