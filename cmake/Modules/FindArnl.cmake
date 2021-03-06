FIND_PACKAGE(BaseArnl REQUIRED)

INCLUDE(searchPath)

FIND_PATH(Arnl_INCLUDE_DIR Arnl.h
    "${ARNL_PATH}include"
    )
	
SET(Arnl_FOUND FALSE)

IF(WIN32)
  IF(MSVC10)
    SET(Arnl_SUFFIX_NAME VC10)
  ELSEIF(MSVC90)
    SET(Arnl_SUFFIX_NAME VC9)
  ELSEIF(MSVC80)
    SET(Arnl_SUFFIX_NAME VC8)
  ELSEIF(MSVC71)
    SET(Arnl_SUFFIX_NAME VC71)
  ELSE()
    SET(Arnl_SUFFIX_NAME "")
  ENDIF()
 
  FIND_LIBRARY(Arnl_LIBRARY_DEBUG
    NAMES ArnlDebug${Arnl_SUFFIX_NAME}
    PATHS 
    "${ARNL_PATH}lib"
  )
  
  FIND_LIBRARY(Arnl_LIBRARY_RELEASE
    NAMES Arnl${Arnl_SUFFIX_NAME}
    PATHS 
    "${ARNL_PATH}lib"
    )
	
  SET(Arnl_LIBRARIES "")
  IF(Arnl_LIBRARY_RELEASE AND Arnl_INCLUDE_DIR)
    LIST(APPEND Arnl_LIBRARIES optimized ${Arnl_LIBRARY_RELEASE})
    SET(Arnl_FOUND TRUE)
  ENDIF()
  IF(Arnl_LIBRARY_DEBUG AND Arnl_INCLUDE_DIR)
    LIST(APPEND Arnl_LIBRARIES debug ${Arnl_LIBRARY_DEBUG})
    SET(Arnl_FOUND TRUE)
  ENDIF()	
ELSE()
  FIND_LIBRARY(Arnl_LIBRARY
    NAMES Arnl
    PATHS 
    /usr/local/Arnl/lib
    )
  IF(Arnl_LIBRARY AND Arnl_INCLUDE_DIR)
    SET(Arnl_LIBRARIES ${Arnl_LIBRARY})
    SET(Arnl_FOUND TRUE)
  ENDIF()
ENDIF()

LIST(APPEND Arnl_LIBRARIES ${BaseArnl_LIBRARIES})
SET(Arnl_INCLUDE_DIRS ${Arnl_INCLUDE_DIR} ${BaseArnl_INCLUDE_DIRS})
  
MARK_AS_ADVANCED(
    Arnl_INCLUDE_DIR
    Arnl_LIBRARY_DEBUG
    Arnl_LIBRARY
    Arnl_LIBRARY_RELEASE
    )

IF(Arnl_FOUND)
    IF(NOT Arnl_FIND_QUIETLY)
        MESSAGE(STATUS "Found Arnl: ${Arnl_LIBRARY}")
    ENDIF(NOT Arnl_FIND_QUIETLY)
ELSE(Arnl_FOUND)
    IF(Arnl_FIND_REQUIRED)
        MESSAGE(FATAL_ERROR "Could not find the Arnl Library")
    ENDIF(Arnl_FIND_REQUIRED)
ENDIF(Arnl_FOUND)
