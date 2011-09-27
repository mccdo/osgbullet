set( LIB_NAME osgdb_osgbdynamics )

_osgBulletPlugin( ${LIB_NAME}
    dotosgArticulationRecord.cpp
    dotosgCreationRecord.cpp
    dotosgPhysicsData.cpp
)

INCLUDE( ModuleInstall REQUIRED )
