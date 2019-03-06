cmake_minimum_required(VERSION 2.8)
project(robotServer C CXX)

find_package(Aria REQUIRED)
find_package(BaseArnl REQUIRED)
find_package(Arnl)
find_package(SonArnl)

include_directories(
    .
    ${Aria_INCLUDE_DIRS}
    ${Arnl_INCLUDE_DIRS}
    ${SonArnl_INCLUDE_DIRS}
)

set( LIBRARIES
    ${LIBRARIES}
    ${Aria_LIBRARIES}
    ${Arnl_LIBRARIES}
    ${SonArnl_LIBRARIES}
)

include_directories(include)

file(GLOB CURRENT_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h")
file(GLOB CURRENT_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c*")

add_library(${PROJECT_NAME} SHARED ${CURRENT_SOURCES} ${CURRENT_HEADERS})
add_library(${PROJECT_NAME}Static STATIC ${CURRENT_SOURCES} ${CURRENT_HEADERS})

target_link_libraries(${PROJECT_NAME} ${LIBRARIES})

set(robotServer_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/robotServer/include CACHE INTERNAL "")
set(robotServer_LIBRARY_DIRS ${CMAKE_BINARY_DIR} CACHE INTERNAL "")
if(WIN32)
    set(robotServer_LIBRARIES ${LIBRARIES} robotServerStatic CACHE INTERNAL "")
else(WIN32)
    set(robotServer_LIBRARIES ${LIBRARIES} robotServer CACHE INTERNAL "")
endif(WIN32)
