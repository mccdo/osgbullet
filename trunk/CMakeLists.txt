# # # # # #  project setup  # # # # # #
cmake_minimum_required( VERSION 2.8 )
project( osgBullet )

# Please keep in sync with Version.h
set( OSGBULLET_VERSION 1.09.00 )


set( CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules;${CMAKE_MODULE_PATH}" )


set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin )
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib )
if( WIN32 )
    set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} )
else()
    set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY} )
endif()
foreach( CONF ${CMAKE_CONFIGURATION_TYPES} )
    string( TOUPPER "${CONF}" CONF )
    set( "CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CONF}" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}" )
    set( "CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${CONF}" "${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}" )
    if( WIN32 )
        set( "CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CONF}" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}" )
    else()
        set( "CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CONF}" "${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}" )
    endif()
endforeach()


IF( WIN32 AND MSVC )
    OPTION( OSGBULLET_WIN32_USE_MP "Build with multiple processes (/MP option; VS 2005 and above)." ON )
    MARK_AS_ADVANCED( OSGBULLET_WIN32_USE_MP )
    IF( OSGBULLET_WIN32_USE_MP )
        SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP" )
    ENDIF( OSGBULLET_WIN32_USE_MP )
ENDIF( WIN32 AND MSVC )

INCLUDE( CMakeMacros )

include( FindOSGHelper )
include( FindBulletHelper )

FIND_PACKAGE( OSGWorks REQUIRED )


option( OSGBULLET_P5_SUPPORT "Enable to include support for the P5 glove and build the related test code" OFF )
if( OSGBULLET_P5_SUPPORT )
    add_definitions( -DUSE_P5 )
    set( OSGBULLET_P5_INCLUDE_DIR "" CACHE PATH "" )
    set( OSGBULLET_P5_LIBRARY "" CACHE FILEPATH "" )
else()
    unset( OSGBULLET_P5_INCLUDE_DIR CACHE )
    unset( OSGBULLET_P5_LIBRARY CACHE )
endif()


INCLUDE_DIRECTORIES( 
    ${PROJECT_SOURCE_DIR}/include
    ${OSG_INCLUDE_DIRS}
    ${OSGWORKS_INCLUDE_DIR}
    ${BULLET_INCLUDE_DIRS}
    ${BULLET_EXTRAS_INCLUDE_DIR}
    ${OSGBULLET_P5_INCLUDE_DIR}
)


# Support for collision-only (no dynamics)
if( BULLET_DYNAMICS_FOUND )
    add_definitions( -DUSE_BULLET_DYNAMICS )
endif()

option( BUILD_SHARED_LIBS "Set to ON to build osgBullet for dynamic linking. Use OFF for static." ON )
if( BUILD_SHARED_LIBS )
    add_definitions( -DOSGBULLET_SHARED )
else()
    add_definitions( -DOSGBULLET_STATIC )
endif()


option( OSGBULLET_USE_DOUBLE_PRECISION "Use Bullet built with double precision" OFF )
if( OSGBULLET_USE_DOUBLE_PRECISION )
    ADD_DEFINITIONS( -DBT_USE_DOUBLE_PRECISION)
endif()


OPTION( OSGBULLET_BUILD_APPLICATIONS "Enable to build applications" ON )
IF( OSGBULLET_BUILD_APPLICATIONS )
    ADD_SUBDIRECTORY( applications )
ENDIF( OSGBULLET_BUILD_APPLICATIONS )

OPTION( OSGBULLET_BUILD_EXAMPLES "Enable to build example code" ON )
IF( OSGBULLET_BUILD_EXAMPLES )
    ADD_SUBDIRECTORY( examples )
ENDIF( OSGBULLET_BUILD_EXAMPLES )

OPTION( OSGBULLET_BUILD_TESTS "Enable to build test code" OFF )
IF( OSGBULLET_BUILD_TESTS )
    ADD_SUBDIRECTORY( tests )
ENDIF( OSGBULLET_BUILD_TESTS )

option( OSGBULLET_INSTALL_DATA "Enable to add the data directory to the install target" ON )
if( OSGBULLET_INSTALL_DATA )
    add_subdirectory( data )
endif()


ADD_SUBDIRECTORY( src/osgbCollision )
ADD_SUBDIRECTORY( src/osgbDynamics )
ADD_SUBDIRECTORY( src/osgbInteraction )
ADD_SUBDIRECTORY( src/plugins )


#
# Doxygen
OPTION( OSGBULLET_DOCUMENTATION "Enable to create build target for Doxygen documentation." OFF )
MARK_AS_ADVANCED( CLEAR OSGBULLET_DOCUMENTATION )
IF( OSGBULLET_DOCUMENTATION )
    FIND_PACKAGE( Doxygen REQUIRED )
    SET( HAVE_DOT "NO" )
    IF( DOXYGEN_DOT_PATH )
        SET( HAVE_DOT "YES" )
    ENDIF( DOXYGEN_DOT_PATH )
    
    CONFIGURE_FILE( ${PROJECT_SOURCE_DIR}/doc/doxyfile.cmake
        ${PROJECT_BINARY_DIR}/doc/doxyfile
    )
    ADD_CUSTOM_TARGET( Documentation ${DOXYGEN_EXECUTABLE}
        ${PROJECT_BINARY_DIR}/doc/doxyfile
    )
ENDIF( OSGBULLET_DOCUMENTATION )
