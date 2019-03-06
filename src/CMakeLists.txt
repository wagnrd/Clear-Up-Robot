cmake_minimum_required(VERSION 3.1)

project(robotik)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/../cmake/Modules/)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Arnl)
find_package(SonArnl)

if(${Arnl_FOUND})
    add_definitions(-DARNL)
endif()
if(${SonArnl_FOUND})
    add_definitions(-DSONARNL)
endif()

set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib:$ORIGIN/")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../bin)

add_subdirectory(robotServer)
#add_subdirectory(behaviourLibrary)
#add_subdirectory(snippets)
add_subdirectory(project)
