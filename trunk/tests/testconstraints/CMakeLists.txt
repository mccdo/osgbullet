set( _exeName testconstraints )

_osgBulletMakeInteractionExe( ${_exeName}
    testconstraints.cpp
    ctest.cpp
    ctest.h
)

add_test(
    NAME ${_exeName}
    COMMAND ${_exeName} --ctest
)
