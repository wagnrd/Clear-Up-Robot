FIND_PACKAGE(Aria REQUIRED)

INCLUDE(searchPath)

FIND_PATH(ArNetworking_INCLUDE_DIR ArNetworking.h
    "${ARIA_PATH}ArNetworking/include"
    )
	
SET(ArNetworking_FOUND FALSE)

IF(WIN32)
  IF(MSVC10)
    SET(ArNetworking_SUFFIX_NAME VC10)
  ELSEIF(MSVC90)
    SET(ArNetworking_SUFFIX_NAME VC9)
  ELSEIF(MSVC80)
    SET(ArNetworking_SUFFIX_NAME VC8)
  ELSEIF(MSVC71)
    SET(ArNetworking_SUFFIX_NAME VC71)
  ELSE()
    SET(ArNetworking_SUFFIX_NAME "")
  ENDIF()
 
  FIND_LIBRARY(ArNetworking_LIBRARY_DEBUG
    NAMES ArNetworkingDebug${ArNetworking_SUFFIX_NAME}
    PATHS 
    "${ARIA_PATH}lib"
  )
  
  FIND_LIBRARY(ArNetworking_LIBRARY_RELEASE
    NAMES ArNetworking${ArNetworking_SUFFIX_NAME}
    PATHS 
    "${ARIA_PATH}lib"
    )
	
  SET(ArNetworking_LIBRARIES "")
  IF(ArNetworking_LIBRARY_RELEASE AND ArNetworking_INCLUDE_DIR)
    SET(ArNetworking_INCLUDE_DIRS ${ArNetworking_INCLUDE_DIR})
    LIST(APPEND ArNetworking_LIBRARIES optimized ${ArNetworking_LIBRARY_RELEASE})
    SET(ArNetworking_FOUND TRUE)
  ENDIF()
  IF(ArNetworking_LIBRARY_DEBUG AND ArNetworking_INCLUDE_DIR)
    SET(ArNetworking_INCLUDE_DIRS ${ArNetworking_INCLUDE_DIR})
    LIST(APPEND ArNetworking_LIBRARIES debug ${ArNetworking_LIBRARY_DEBUG})
    SET(ArNetworking_FOUND TRUE)
  ENDIF()	
ELSE()
  FIND_LIBRARY(ArNetworking_LIBRARY
    NAMES ArNetworking
    PATHS 
    "${ARIA_PATH}lib"
    )
  IF(ArNetworking_LIBRARY AND ArNetworking_INCLUDE_DIR)
    SET(ArNetworking_INCLUDE_DIRS ${ArNetworking_INCLUDE_DIR})
    SET(ArNetworking_LIBRARIES
        ${ArNetworking_LIBRARY}
	${CMAKE_THREAD_LIBS_INIT}
	${CMAKE_DL_LIBS}
	-lrt
    )
    SET(ArNetworking_FOUND TRUE)
  ENDIF()
ENDIF()
  
MARK_AS_ADVANCED(
    ArNetworking_INCLUDE_DIR
    ArNetworking_LIBRARY_DEBUG
    ArNetworking_LIBRARY
    ArNetworking_LIBRARY_RELEASE
    )

IF(ArNetworking_FOUND)
    IF(NOT ArNetworking_FIND_QUIETLY)
        MESSAGE(STATUS "Found ArNetworking: ${ArNetworking_LIBRARY}")
    ENDIF(NOT ArNetworking_FIND_QUIETLY)
ELSE(ArNetworking_FOUND)
    IF(ArNetworking_FIND_REQUIRED)
        MESSAGE(FATAL_ERROR "Could not find the ArNetworking Library")
    ENDIF(ArNetworking_FIND_REQUIRED)
ENDIF(ArNetworking_FOUND)
