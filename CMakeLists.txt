cmake_minimum_required(VERSION 3.20)

if (POLICY CMP0144)
    cmake_policy(SET CMP0144 NEW)
endif()

project(OpenFlightLab LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(raylib REQUIRED)

set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")

set(PROJECT_SOURCES
    "${SRC_DIR}/main.cpp"
    "${SRC_DIR}/ball.cpp"
    "${SRC_DIR}/FlightSimulator.cpp"
    "${SRC_DIR}/engine/Engine.cpp"
    "${SRC_DIR}/engine/EventBus.cpp"
    "${SRC_DIR}/flightsim/Application.cpp"
    "${SRC_DIR}/flightsim/core/EngineState.cpp"
    "${SRC_DIR}/flightsim/core/FlightSimulationSubsystem.cpp"
    "${SRC_DIR}/flightsim/core/SimulationConfig.cpp"
    "${SRC_DIR}/flightsim/core/SimulationEngine.cpp"
    "${SRC_DIR}/flightsim/io/ConsoleOutput.cpp"
    "${SRC_DIR}/flightsim/io/DataLogger.cpp"
    "${SRC_DIR}/flightsim/ofm/OFMInterface.cpp"
    "${SRC_DIR}/flightsim/physics/Atmosphere.cpp"
    "${SRC_DIR}/flightsim/physics/FlightDynamics.cpp"
    "${SRC_DIR}/flightsim/physics/Quaternion.cpp"
)

add_executable(Application ${PROJECT_SOURCES})

set_target_properties(Application PROPERTIES
    OUTPUT_NAME "Application"
)

target_include_directories(Application PRIVATE
    "${SRC_DIR}"
    "${SRC_DIR}/engine"
    "${SRC_DIR}/flightsim"
    "${SRC_DIR}/flightsim/core"
    "${SRC_DIR}/flightsim/physics"
    "${SRC_DIR}/flightsim/io"
    "${SRC_DIR}/flightsim/ofm"
)

target_link_libraries(Application PRIVATE raylib::raylib)

target_compile_definitions(Application PRIVATE PLATFORM_DESKTOP _DEFAULT_SOURCE)

if (MSVC)
    target_compile_options(Application PRIVATE /W4)
else()
    target_compile_options(Application PRIVATE -Wall -Wno-missing-braces)
endif()

if (MINGW)
    target_link_options(Application PRIVATE -static-libgcc -static-libstdc++)
endif()
