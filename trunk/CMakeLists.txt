# # # # # #  project setup  # # # # # #
cmake_minimum_required( VERSION 2.8.8 )
project( osgBullet )

# Please keep in sync with Version.h
set( OSGBULLET_VERSION 2.00.02 )
set( OSGBCOLLISION_MAJOR_VERSION 2 )
set( OSGBCOLLISION_MINOR_VERSION 0 )
set( OSGBCOLLISION_SUB_VERSION 2 )

set( CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules;${CMAKE_MODULE_PATH}" )


set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin )
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib )
if( WIN32 )
    set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} )
else()
    set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY} )
endif()

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

FIND_PACKAGE( osgWorks 2.0.0 REQUIRED )
if( osgWorks_FOUND )
    include( ${osgWorks_USE_FILE} )
endif()

# OSGBULLET_USE_DOUBLE_PRECISION gets set to TRUE or FALSE in FindBulletHelper.cmake.
# If TRUE, add the BT double precision definition globally.
if( OSGBULLET_USE_DOUBLE_PRECISION )
    add_definitions( -DBT_USE_DOUBLE_PRECISION )
endif()


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
    #${OSGWORKS_INCLUDE_DIR}
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



OPTION( OSGBULLET_BUILD_APPLICATIONS "Enable to build applications" ON )
IF( OSGBULLET_BUILD_APPLICATIONS )
    ADD_SUBDIRECTORY( applications )
ENDIF( OSGBULLET_BUILD_APPLICATIONS )

OPTION( OSGBULLET_BUILD_EXAMPLES "Enable to build example code" ON )
IF( OSGBULLET_BUILD_EXAMPLES )
    ADD_SUBDIRECTORY( examples )
ENDIF( OSGBULLET_BUILD_EXAMPLES )

option( OSGBULLET_BUILD_TESTS "Enable to build test code" OFF )
if( OSGBULLET_BUILD_TESTS )
    enable_testing()
    add_subdirectory( tests )
endif()

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
find_package( Doxygen )
if( DOXYGEN_FOUND )
    set( HAVE_DOT "NO" )
    if( DOXYGEN_DOT_PATH )
        set( HAVE_DOT "YES" )
    endif()
    
    configure_file( ${PROJECT_SOURCE_DIR}/doc/doxyfile.cmake
        ${PROJECT_BINARY_DIR}/doc/doxyfile
    )
    add_custom_target( Documentation ${DOXYGEN_EXECUTABLE}
        ${PROJECT_BINARY_DIR}/doc/doxyfile
    )
endif()

# On windows provide the user with the pdb files 
# for debugging if they are present
if(MSVC)
    install(
        DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/\${CMAKE_INSTALL_CONFIG_NAME}/
        DESTINATION lib
        USE_SOURCE_PERMISSIONS
        COMPONENT libosgbbullet-dev
        FILES_MATCHING PATTERN "*.pdb"
    )
endif(MSVC)

# Install the CMake find script for osgWorks
install( FILES "extra/FindosgBullet.cmake"
    DESTINATION "share/${CMAKE_PROJECT_NAME}/extra"
    COMPONENT libosgbbullet
)

#
# Create and install all of the auto find tools
# =============================================

# Add all targets to the build-tree export set
export(TARGETS osgbInteraction osgbCollision osgbDynamics
  FILE "${PROJECT_BINARY_DIR}/lib/osgBulletLibraryDepends.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE osgBullet)

# Create a osgBulletBuildTreeSettings.cmake file for the use from the build tree
configure_file(osgBulletBuildTreeSettings.cmake.in
  "${PROJECT_BINARY_DIR}/lib/osgBulletBuildTreeSettings.cmake" @ONLY)

#configure_file(osgBulletConfig.cmake.in
#  "${PROJECT_BINARY_DIR}/lib/osgBulletConfig.cmake" @ONLY)
configure_file(osgBulletConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}/lib/osgBulletConfigVersion.cmake" @ONLY)
configure_file(UseosgBullet.cmake.in
  "${PROJECT_BINARY_DIR}/lib/UseosgBullet.cmake" @ONLY)

# Install the export set for use with the install-tree
install(EXPORT osgbullet-targets DESTINATION lib)

include( CMakePackageConfigHelpers )
# Create the osgBulletConfig.cmake and osgBulletConfigVersion files
#file(RELATIVE_PATH CONF_REL_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/lib"
#   "${CMAKE_INSTALL_PREFIX}/include")

configure_package_config_file( 
    osgBulletConfig.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/lib/osgBulletConfig.cmake
    INSTALL_DESTINATION lib
    PATH_VARS OSGBULLET_USE_DOUBLE_PRECISION )

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/lib/osgBulletConfig.cmake 
              ${CMAKE_CURRENT_BINARY_DIR}/lib/osgBulletConfigVersion.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/lib/UseosgBullet.cmake
        DESTINATION lib )


