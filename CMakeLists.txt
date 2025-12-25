cmake_minimum_required(VERSION 3.26)
project(PhysicsEngine)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SFML_DIR "/opt/homebrew/lib/cmake/SFML")

include_directories("/opt/homebrew/include")

find_package(SFML 3 COMPONENTS Graphics Window System REQUIRED)

# Main executable
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

# ============ TESTING ============
option(BUILD_TESTS "Build the test suite" ON)

if(BUILD_TESTS)
    include(FetchContent)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    enable_testing()

    add_executable(PhysicsTests
        tests/test_main.cpp
        tests/test_verlet.cpp
        tests/test_constraints.cpp
        tests/test_collision.cpp
        PhysicsWorld.cpp
    )

    target_include_directories(PhysicsTests PRIVATE ${CMAKE_SOURCE_DIR})
    target_link_libraries(PhysicsTests PRIVATE 
        GTest::gtest_main 
        SFML::Graphics 
        SFML::Window 
        SFML::System
    )

    include(GoogleTest)
    gtest_discover_tests(PhysicsTests)
endif()
