cmake_minimum_required(VERSION 3.17)
project(SwarmCtrl)

set(CMAKE_CXX_STANDARD 14)

include_directories(include)

set(USER_SRC
        src/SwarmCtrl.cpp
        main.cpp
        )

add_executable(SwarmCtrl ${USER_SRC})