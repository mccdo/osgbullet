set( LIB_NAME osgdb_osgbDynamics )

_osgBulletPlugin( ${LIB_NAME}
    dotosgCreationRecord.cpp
    dotosgPhysicsData.cpp
)

INCLUDE( ModuleInstall REQUIRED )
