cmake_minimum_required(VERSION 3.20)

project(NeedlePlanner VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

set(CMAKE_PREFIX_PATH "~/Qt/6.9.2/macos")

find_package(Qt6 REQUIRED COMPONENTS Core Widgets OpenGL OpenGLWidgets)

set(HEADERS
    Core/Conflict.hpp
    Core/Position.hpp
    Core/Constraint.hpp
    Core/AgentNeedle.hpp
    Core/Path.hpp
    Core/WorldModel.hpp

    Planner/CBS.h
    Planner/AStarSearch.h

    UI/TestMainWindow.h
)

set(SOURCES
    Planner/CBS.cpp
    Planner/AStarSearch.cpp

    UI/TestMainWindow.cpp
    UI/main.cpp
    UI/GLWidget.cpp
)



add_executable(${PROJECT_NAME} ${HEADERS} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt6::Core
    Qt6::Widgets
    Qt6::OpenGL
    Qt6::OpenGLWidgets
)

# ---- Core library (Planner only, no UI/Qt) ----
add_library(PlannerCore STATIC
    Planner/CBS.cpp
    Planner/AStarSearch.cpp
)
target_include_directories(PlannerCore PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# ---- Tests (console, no Qt) ----
add_executable(NeedleCollisionTest tests/NeedleCollisionTest.cpp)
target_include_directories(NeedleCollisionTest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(NeedleCollisionTest PRIVATE PlannerCore)
