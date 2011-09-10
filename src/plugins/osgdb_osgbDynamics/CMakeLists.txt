set( LIB_NAME osgdb_osgbDynamics )

_osgBulletPlugin( ${LIB_NAME}
    dotosgArticulationRecord.cpp
    dotosgCreationRecord.cpp
    dotosgPhysicsData.cpp
)

INCLUDE( ModuleInstall REQUIRED )
