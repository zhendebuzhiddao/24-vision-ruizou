cmake_minimum_required(VERSION 3.5)

project(WLX CXX)

add_executable(${PROJECT_NAME}
    main.cpp
    # buzhidao.hpp
    # buzhidao.cpp
)

find_package(OpenCV REQUIRED)

target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${OpenCV_INCLUDE_DIRS}
        "include"
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        ${OpenCV_LIBS}
)
