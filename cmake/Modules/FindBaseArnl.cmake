FIND_PACKAGE(ArNetworking REQUIRED)

INCLUDE(searchPath)

FIND_PATH(BaseArnl_INCLUDE_DIR Arnl.h
    "${ARNL_PATH}include"
    )
	
SET(BaseArnl_FOUND FALSE)

IF(WIN32)
  IF(MSVC10)
    SET(BaseArnl_SUFFIX_NAME VC10)
  ELSEIF(MSVC90)
    SET(BaseArnl_SUFFIX_NAME VC9)
  ELSEIF(MSVC80)
    SET(BaseArnl_SUFFIX_NAME VC8)
  ELSEIF(MSVC71)
    SET(BaseArnl_SUFFIX_NAME VC71)
  ELSE()
    SET(BaseArnl_SUFFIX_NAME "")
  ENDIF()
 
  FIND_LIBRARY(BaseArnl_LIBRARY_DEBUG
    NAMES BaseArnlDebug${BaseArnl_SUFFIX_NAME} ArNetworkingDebug${BaseArnl_SUFFIX_NAME}
    PATHS 
    "${ARNL_PATH}lib"
  )
  
  FIND_LIBRARY(BaseArnl_LIBRARY_RELEASE
    NAMES BaseArnl${BaseArnl_SUFFIX_NAME} ArNetworking${BaseArnl_SUFFIX_NAME}
    PATHS 
    "${ARNL_PATH}lib"
    )
	
  SET(BaseArnl_LIBRARIES "")
  IF(BaseArnl_LIBRARY_RELEASE AND BaseArnl_INCLUDE_DIR)
    LIST(APPEND BaseArnl_LIBRARIES optimized ${BaseArnl_LIBRARY_RELEASE})
    SET(BaseArnl_FOUND TRUE)
  ENDIF()
  IF(BaseArnl_LIBRARY_DEBUG AND BaseArnl_INCLUDE_DIR)
    LIST(APPEND BaseArnl_LIBRARIES debug ${BaseArnl_LIBRARY_DEBUG})
    SET(BaseArnl_FOUND TRUE)
  ENDIF()	
  SET(BaseArnl_INCLUDE_DIRS 
	${BaseArnl_INCLUDE_DIR} 
        ${ArNetworking_INCLUDE_DIRS}
	"C:/MobileRobots/ARNL/include/Aria"
  )
ELSE()
  FIND_LIBRARY(BaseArnl_LIBRARY
    NAMES BaseArnl
    PATHS 
    "${ARNL_PATH}lib"
    )
  IF(BaseArnl_LIBRARY AND BaseArnl_INCLUDE_DIR)
    SET(BaseArnl_INCLUDE_DIRS ${BaseArnl_INCLUDE_DIR})
    SET(BaseArnl_LIBRARIES ${BaseArnl_LIBRARY})
    SET(BaseArnl_FOUND TRUE)
  ENDIF()
ENDIF()

LIST(APPEND BaseArnl_INCLUDE_DIRS ${ArNetworking_INCLUDE_DIRS})
LIST(APPEND BaseArnl_LIBRARIES ${ArNetworking_LIBRARIES})
  
MARK_AS_ADVANCED(
    BaseArnl_INCLUDE_DIR
    BaseArnl_LIBRARY_DEBUG
    BaseArnl_LIBRARY
    BaseArnl_LIBRARY_RELEASE
    )

IF(BaseArnl_FOUND)
    IF(NOT BaseArnl_FIND_QUIETLY)
        MESSAGE(STATUS "Found BaseArnl: ${BaseArnl_LIBRARY}")
    ENDIF(NOT BaseArnl_FIND_QUIETLY)
ELSE(BaseArnl_FOUND)
    IF(BaseArnl_FIND_REQUIRED)
        MESSAGE(FATAL_ERROR "Could not find the BaseArnl Library")
    ENDIF(BaseArnl_FIND_REQUIRED)
ENDIF(BaseArnl_FOUND)

