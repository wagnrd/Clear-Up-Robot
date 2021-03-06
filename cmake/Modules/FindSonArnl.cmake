FIND_PACKAGE(BaseArnl REQUIRED)

INCLUDE(searchPath)

FIND_PATH(SonArnl_INCLUDE_DIR Arnl.h
    "${ARNL_PATH}include"
    )
	
SET(SonArnl_FOUND FALSE)

IF(WIN32)
  IF(MSVC10)
    SET(SonArnl_SUFFIX_NAME VC10)
  ELSEIF(MSVC90)
    SET(SonArnl_SUFFIX_NAME VC9)
  ELSEIF(MSVC80)
    SET(SonArnl_SUFFIX_NAME VC8)
  ELSEIF(MSVC71)
    SET(SonArnl_SUFFIX_NAME VC71)
  ELSE()
    SET(SonArnl_SUFFIX_NAME "")
  ENDIF()
 
  FIND_LIBRARY(SonArnl_LIBRARY_DEBUG
    NAMES SonArnlDebug${SonArnl_SUFFIX_NAME}
    PATHS 
    "${ARNL_PATH}lib"
  )
  
  FIND_LIBRARY(SonArnl_LIBRARY_RELEASE
    NAMES SonArnl${SonArnl_SUFFIX_NAME}
    PATHS 
    "${ARNL_PATH}lib"
    )
	
  SET(SonArnl_LIBRARIES "")
  IF(SonArnl_LIBRARY_RELEASE AND SonArnl_INCLUDE_DIR)
    LIST(APPEND SonArnl_LIBRARIES optimized ${SonArnl_LIBRARY_RELEASE})
    SET(SonArnl_FOUND TRUE)
  ENDIF()
  IF(SonArnl_LIBRARY_DEBUG AND SonArnl_INCLUDE_DIR)
    LIST(APPEND SonArnl_LIBRARIES debug ${SonArnl_LIBRARY_DEBUG})
    SET(SonArnl_FOUND TRUE)
  ENDIF()	
ELSE()
  FIND_LIBRARY(SonArnl_LIBRARY
    NAMES SonArnl
    PATHS 
    "${ARNL_PATH}lib"
    )
  IF(SonArnl_LIBRARY AND SonArnl_INCLUDE_DIR)
    SET(SonArnl_LIBRARIES ${SonArnl_LIBRARY})
    SET(SonArnl_FOUND TRUE)
  ENDIF()
ENDIF()

LIST(APPEND Arnl_LIBRARIES ${BASEArnl_LIBRARIES})
SET(SonArnl_INCLUDE_DIRS ${SonArnl_INCLUDE_DIR} ${BASEArnl_INCLUDE_DIRS})
  
MARK_AS_ADVANCED(
    SonArnl_INCLUDE_DIR
    SonArnl_LIBRARY_DEBUG
    SonArnl_LIBRARY
    SonArnl_LIBRARY_RELEASE
    )

IF(SonArnl_FOUND)
    IF(NOT SonArnl_FIND_QUIETLY)
        MESSAGE(STATUS "Found SonArnl: ${SonArnl_LIBRARY}")
    ENDIF(NOT SonArnl_FIND_QUIETLY)
ELSE(SonArnl_FOUND)
    IF(SonArnl_FIND_REQUIRED)
        MESSAGE(FATAL_ERROR "Could not find the SonArnl Library")
    ENDIF(SonArnl_FIND_REQUIRED)
ENDIF(SonArnl_FOUND)

