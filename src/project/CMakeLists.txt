CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
project(project C CXX)

find_package(Aria REQUIRED)
find_package(BaseArnl REQUIRED)
find_package(Arnl REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    .
    ${Aria_INCLUDE_DIRS}
    ${Arnl_INCLUDE_DIRS}
    ${SonArnl_INCLUDE_DIRS}
    ${robotServer_INCLUDE_DIRS}
)

set( LIBRARIES
    ${LIBRARIES}
    ${Aria_LIBRARIES}
    ${Arnl_LIBRARIES}
    ${SonArnl_LIBRARIES}
    ${robotServer_LIBRARIES}
    ${OpenCV_LIBS}
)

file(GLOB_RECURSE CURRENT_HEADERS "*.h*")
file(GLOB_RECURSE CURRENT_SOURCES "*.c*")

add_executable(${PROJECT_NAME} ${CURRENT_SOURCES} ${CURRENT_HEADERS})

target_link_libraries(${PROJECT_NAME} ${LIBRARIES})
