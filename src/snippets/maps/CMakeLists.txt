cmake_minimum_required(VERSION 2.8)
project(maps C CXX)

find_package(Aria REQUIRED)

include_directories(
    .
    ${Aria_INCLUDE_DIRS}
    ${robotServer_INCLUDE_DIRS}
)

set( LIBRARIES
    ${LIBRARIES}
    ${Aria_LIBRARIES}
    ${robotServer_LIBRARIES}
)

file(GLOB CURRENT_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
file(GLOB CURRENT_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.c*")

add_executable(${PROJECT_NAME} ${CURRENT_SOURCES} ${CURRENT_HEADERS})

target_link_libraries(${PROJECT_NAME} ${LIBRARIES})
