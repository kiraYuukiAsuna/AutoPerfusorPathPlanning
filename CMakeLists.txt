cmake_minimum_required(VERSION 3.20)
project(NeedlePlanner VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 REQUIRED COMPONENTS Core Widgets 3DCore 3DRender 3DExtras 3DInput 3DLogic)

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
)



add_executable(${PROJECT_NAME} ${HEADERS} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt6::Core
    Qt6::Widgets
    Qt6::3DCore
    Qt6::3DRender
    Qt6::3DExtras
    Qt6::3DInput
    Qt6::3DLogic
)
