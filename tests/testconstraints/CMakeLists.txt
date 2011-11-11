set( _exeName testconstraints )

_osgBulletMakeInteractionExe( ${_exeName}
    testconstraints.cpp
)

add_test(
    NAME ${_exeName}
    COMMAND ${_exeName}
)
