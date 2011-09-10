// Copyright (c) 2010 Skew Matrix Software LLC. All rights reserved.

#include <osgbInteraction/ArticulationRecord.h>
#include <osg/Object>
#include <osg/Vec3d>


namespace osgbInteraction
{


ArticulationRecord::ArticulationRecord()
  : _version( 2 )
{
}
ArticulationRecord::ArticulationRecord( const osg::Vec3d& axis, const osg::Vec3d& pivotPoint )
  : _axis( axis ),
    _pivotPoint( pivotPoint ),
    _version( 2 )
{
}

ArticulationRecord::ArticulationRecord( const ArticulationRecord& rhs, const osg::CopyOp& copyop )
  : _axis( rhs._axis ),
    _pivotPoint( rhs._pivotPoint ),
    _version( rhs._version )
{
}


ArticulationRecord::~ArticulationRecord()
{
}


// osgbInteraction
}
