cmake_minimum_required(VERSION 3.26)
project(PhysicsEngine)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SFML_DIR "/opt/homebrew/lib/cmake/SFML")

include_directories("/opt/homebrew/include")

find_package(SFML 3 COMPONENTS Graphics Window System REQUIRED)

add_executable(PhysicsEngine main.cpp
        Collider.h
        Object.h
        Constraint.h
        Ball.h
        Grid.h
        PhysicsWorld.h
        PhysicsWorld.cpp
        UI/InfoPanel.h
        UI/CounterPanel.h)

target_link_libraries(PhysicsEngine PRIVATE SFML::Graphics SFML::Window SFML::System)