cmake_minimum_required(VERSION 3.16)
project(lifelong)

set(CMAKE_CXX_STANDARD 17)

# Set the default value of PYTHON to false
option(PYTHON "Enable Python" OFF)

if (PYTHON)
    set(PYTHON_FLAG 1)
else ()
    set(PYTHON_FLAG 0)
endif ()

# Print the value of PYTHON
message(STATUS "PYTHON: ${PYTHON}")

IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "RELEASE")
ENDIF ()

set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

file(GLOB SOURCES
        src/ActionModel.cpp
        src/common.cpp
        src/CompetitionSystem.cpp
        src/Entry.cpp
        src/Evaluation.cpp
        src/Grid.cpp
        src/Logger.cpp
        src/MAPFPlanner.cpp
        src/Simulator.cpp
        src/States.cpp
        src/TaskManager.cpp
        src/TaskScheduler.cpp
        inc/.*cpp
        default_planner/*.cpp

        # Solution2 sources are now enabled
        Solution2/src/*.cpp
        Solution2/src/LaCAM2/*.cpp
        Solution2/src/LaCAM2/SUO2/*.cpp
        Solution2/src/LNS/*.cpp
        Solution2/src/LNS/Parallel/*.cpp
        Solution2/src/RHCR/interface/*.cpp
        Solution2/src/RHCR/main/*.cpp
        Solution2/src/util/*.cpp
        )

include_directories("inc")
include_directories("default_planner")
include_directories("Solution")
include_directories("Solution2")
include_directories("Solution2/inc")
include_directories("Solution2/src")

# Find Boost and its components, including iostreams for compression
cmake_policy(SET CMP0167 NEW) # Suppress Boost warning
SET(Boost_USE_STATIC_LIBS ON)
find_package(Boost 1.49.0 REQUIRED COMPONENTS program_options system filesystem log log_setup iostreams)

include_directories(${Boost_INCLUDE_DIRS})

# MODIFIED: Use PkgConfig to reliably find compression libraries
find_package(PkgConfig REQUIRED)
pkg_check_modules(BZIP2 REQUIRED bzip2)
pkg_check_modules(LZMA REQUIRED liblzma)
pkg_check_modules(ZSTD REQUIRED libzstd)
pkg_check_modules(ZLIB REQUIRED zlib)

find_package(OpenMP REQUIRED)
find_package(spdlog REQUIRED)
find_package(Threads REQUIRED)

IF (PYTHON)
    message(STATUS "Python support is enabled")
    # Python configuration would go here
ELSE ()
    message(STATUS "Python support is disabled")

    # The main executable for your project
    add_executable(lifelong src/driver.cpp ${SOURCES})

    # Include directories found by PkgConfig
    include_directories(${BZIP2_INCLUDE_DIRS})
    include_directories(${LZMA_INCLUDE_DIRS})
    include_directories(${ZSTD_INCLUDE_DIRS})
    include_directories(${ZLIB_INCLUDE_DIRS})

    # Link all necessary libraries to the lifelong executable
    target_link_libraries(lifelong PRIVATE ${Boost_LIBRARIES})
    target_link_libraries(lifelong PRIVATE OpenMP::OpenMP_CXX)
    target_link_libraries(lifelong PRIVATE spdlog::spdlog)
    
    # Link the compression libraries found by PkgConfig
    target_link_libraries(lifelong PRIVATE ${BZIP2_LIBRARIES})
    target_link_libraries(lifelong PRIVATE ${LZMA_LIBRARIES})
    target_link_libraries(lifelong PRIVATE ${ZSTD_LIBRARIES})
    target_link_libraries(lifelong PRIVATE ${ZLIB_LIBRARIES})

    target_compile_definitions(lifelong PRIVATE PYTHON=${PYTHON_FLAG})

ENDIF ()

add_custom_target(clean_all
        COMMAND ${CMAKE_BUILD_TOOL} clean
        COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/CMakeCache.txt
        COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/lifelong
        COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/build
        COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/*.so
        )

set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES
        "${CMAKE_BINARY_DIR}/lifelong;
       ${CMAKE_BINARY_DIR}/build;
       ${CMAKE_BINARY_DIR}/*.so"
        )
