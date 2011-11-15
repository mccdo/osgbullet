/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgBullet is (C) Copyright 2009-2011 by Kenneth Mark Bryden
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License version 2.1 as published by the Free Software Foundation.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 *************** <auto-copyright.pl END do not edit this line> ***************/

#include <osgbDynamics/Constraints.h>
#include "dotosgMatrixIO.h"

#include <iostream>
#include <string>

#include <osg/io_utils>

#include <osgDB/Registry>
#include <osgDB/Input>
#include <osgDB/Output>


bool Constraint_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool Constraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy Constraint_Proxy
(
    new osgbDynamics::Constraint,
    "Constraint",
    "Object Constraint",
    Constraint_readLocalData,
    Constraint_writeLocalData
);


bool SliderConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool SliderConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy SliderConstraint_Proxy
(
    new osgbDynamics::SliderConstraint,
    "SliderConstraint",
    "Object Constraint SliderConstraint",
    SliderConstraint_readLocalData,
    SliderConstraint_writeLocalData
);


bool TwistSliderConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool TwistSliderConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy TwistSliderConstraint_Proxy
(
    new osgbDynamics::TwistSliderConstraint,
    "TwistSliderConstraint",
    "Object Constraint SliderConstraint TwistSliderConstraint",
    TwistSliderConstraint_readLocalData,
    TwistSliderConstraint_writeLocalData
);


bool LinearSpringConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool LinearSpringConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy LinearSpringConstraint_Proxy
(
    new osgbDynamics::LinearSpringConstraint,
    "LinearSpringConstraint",
    "Object Constraint LinearSpringConstraint",
    LinearSpringConstraint_readLocalData,
    LinearSpringConstraint_writeLocalData
);


bool AngleSpringConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool AngleSpringConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy AngleSpringConstraint_Proxy
(
    new osgbDynamics::AngleSpringConstraint,
    "AngleSpringConstraint",
    "Object Constraint AngleSpringConstraint",
    AngleSpringConstraint_readLocalData,
    AngleSpringConstraint_writeLocalData
);


bool LinearAngleSpringConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool LinearAngleSpringConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy LinearAngleSpringConstraint_Proxy
(
    new osgbDynamics::LinearAngleSpringConstraint,
    "LinearAngleSpringConstraint",
    "Object Constraint LinearAngleSpringConstraint",
    LinearAngleSpringConstraint_readLocalData,
    LinearAngleSpringConstraint_writeLocalData
);


bool FixedConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool FixedConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy FixedConstraint_Proxy
(
    new osgbDynamics::FixedConstraint,
    "FixedConstraint",
    "Object Constraint FixedConstraint",
    FixedConstraint_readLocalData,
    FixedConstraint_writeLocalData
);


bool PlanarConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool PlanarConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy PlanarConstraint_Proxy
(
    new osgbDynamics::PlanarConstraint,
    "PlanarConstraint",
    "Object Constraint PlanarConstraint",
    PlanarConstraint_readLocalData,
    PlanarConstraint_writeLocalData
);


bool BoxConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool BoxConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy BoxConstraint_Proxy
(
    new osgbDynamics::BoxConstraint,
    "BoxConstraint",
    "Object Constraint BoxConstraint",
    BoxConstraint_readLocalData,
    BoxConstraint_writeLocalData
);


bool HingeConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool HingeConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy HingeConstraint_Proxy
(
    new osgbDynamics::HingeConstraint,
    "HingeConstraint",
    "Object Constraint HingeConstraint",
    HingeConstraint_readLocalData,
    HingeConstraint_writeLocalData
);


bool CardanConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool CardanConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy CardanConstraint_Proxy
(
    new osgbDynamics::CardanConstraint,
    "CardanConstraint",
    "Object Constraint CardanConstraint",
    CardanConstraint_readLocalData,
    CardanConstraint_writeLocalData
);


bool BallAndSocketConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool BallAndSocketConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy BallAndSocketConstraint_Proxy
(
    new osgbDynamics::BallAndSocketConstraint,
    "BallAndSocketConstraint",
    "Object Constraint BallAndSocketConstraint",
    BallAndSocketConstraint_readLocalData,
    BallAndSocketConstraint_writeLocalData
);


bool RagdollConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool RagdollConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy RagdollConstraint_Proxy
(
    new osgbDynamics::RagdollConstraint,
    "RagdollConstraint",
    "Object Constraint RagdollConstraint",
    RagdollConstraint_readLocalData,
    RagdollConstraint_writeLocalData
);


bool WheelSuspensionConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool WheelSuspensionConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy WheelSuspensionConstraint_Proxy
(
    new osgbDynamics::WheelSuspensionConstraint,
    "WheelSuspensionConstraint",
    "Object Constraint WheelSuspensionConstraint",
    WheelSuspensionConstraint_readLocalData,
    WheelSuspensionConstraint_writeLocalData
);


bool Constraint_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    osgbDynamics::Constraint& cons = static_cast< osgbDynamics::Constraint& >( obj );

    osg::Matrix m;
    if( readMatrix( m, fr, "rbAXform" ) )
        cons.setAXform( m );
    else
    {
        osg::notify( osg::WARN ) << "Constraint_readLocalData: Bad input data at \"rbAXform\"." << std::endl;
        return( false );
    }

    if( readMatrix( m, fr, "rbBXform" ) )
        cons.setBXform( m );
    else
    {
        osg::notify( osg::WARN ) << "Constraint_readLocalData: Bad input data at \"rbBXform\"." << std::endl;
        return( false );
    }

    return( true );
}
bool Constraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    const osgbDynamics::Constraint& cons = static_cast< const osgbDynamics::Constraint& >( obj );

    writeMatrix( cons.getAXform(), fw, "rbAXform" );
    writeMatrix( cons.getBXform(), fw, "rbBXform" );

    return( true );
}


bool SliderConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    osgbDynamics::SliderConstraint& cons = static_cast< osgbDynamics::SliderConstraint& >( obj );

    if( fr.matchSequence( "Axis %f %f %f" ) )
    {
        osg::Vec3 axis;
        fr[1].getFloat( ( axis[0] ) );
        fr[2].getFloat( ( axis[1] ) );
        fr[3].getFloat( ( axis[2] ) );
        cons.setAxis( axis );
        fr += 4;
    }
    else
    {
        osg::notify( osg::WARN ) << "SliderConstraint_readLocalData: Bad input data at \"Axis\"." << std::endl;
        return( false );
    }

    if( fr.matchSequence( "Limit %f %f" ) )
    {
        osg::Vec2 limit;
        fr[1].getFloat( ( limit[0] ) );
        fr[2].getFloat( ( limit[1] ) );
        cons.setLimit( limit );
        fr += 3;
    }
    else
    {
        osg::notify( osg::WARN ) << "SliderConstraint_readLocalData: Bad input data at \"Limit\"." << std::endl;
        return( false );
    }

    return( true );
}
bool SliderConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    const osgbDynamics::SliderConstraint& cons = static_cast< const osgbDynamics::SliderConstraint& >( obj );

    fw.indent() << "Axis " << cons.getAxis() << std::endl;
    fw.indent() << "Limit " << cons.getLimit() << std::endl;

    return( true );
}


bool TwistSliderConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    // No-op, but this function must exist to supprt the TwistSliderConstraint object.
    return( true );
}
bool TwistSliderConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    // No-op, but this function must exist to supprt the TwistSliderConstraint object.
    return( true );
}


bool LinearSpringConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    // TBD
    return( true );
}
bool LinearSpringConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    // TBD
    return( true );
}


bool AngleSpringConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    // TBD
    return( true );
}
bool AngleSpringConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    // TBD
    return( true );
}


bool LinearAngleSpringConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    // TBD
    return( true );
}
bool LinearAngleSpringConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    // TBD
    return( true );
}


bool FixedConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    // No-op, but this function must exist to supprt the TwistSliderConstraint object.
    return( true );
}
bool FixedConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    // No-op, but this function must exist to supprt the TwistSliderConstraint object.
    return( true );
}


bool PlanarConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    osgbDynamics::PlanarConstraint& cons = static_cast< osgbDynamics::PlanarConstraint& >( obj );

    if( fr.matchSequence( "Low limit %f %f" ) )
    {
        osg::Vec2 loLimit;
        fr[2].getFloat( ( loLimit[0] ) );
        fr[3].getFloat( ( loLimit[1] ) );
        cons.setLowLimit( loLimit );
        fr += 4;
    }
    else
    {
        osg::notify( osg::WARN ) << "PlanarConstraint_readLocalData: Bad input data at \"Low limit\"." << std::endl;
        return( false );
    }

    if( fr.matchSequence( "High limit %f %f" ) )
    {
        osg::Vec2 hiLimit;
        fr[2].getFloat( ( hiLimit[0] ) );
        fr[3].getFloat( ( hiLimit[1] ) );
        cons.setHighLimit( hiLimit );
        fr += 4;
    }
    else
    {
        osg::notify( osg::WARN ) << "PlanarConstraint_readLocalData: Bad input data at \"High limit\"." << std::endl;
        return( false );
    }

    osg::Matrix m;
    if( readMatrix( m, fr, "Orient" ) )
        cons.setOrientation( m );
    else
    {
        osg::notify( osg::WARN ) << "PlanarConstraint_readLocalData: Bad input data at \"Orient\"." << std::endl;
        return( false );
    }

    return( true );
}
bool PlanarConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    const osgbDynamics::PlanarConstraint& cons = static_cast< const osgbDynamics::PlanarConstraint& >( obj );

    fw.indent() << "Low limit " << cons.getLowLimit() << std::endl;
    fw.indent() << "High limit " << cons.getHighLimit() << std::endl;
    writeMatrix( cons.getOrientation(), fw, "Orient" );

    return( true );
}


bool BoxConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    osgbDynamics::BoxConstraint& cons = static_cast< osgbDynamics::BoxConstraint& >( obj );

    if( fr.matchSequence( "Low limit %f %f %f" ) )
    {
        osg::Vec3 loLimit;
        fr[2].getFloat( ( loLimit[0] ) );
        fr[3].getFloat( ( loLimit[1] ) );
        fr[4].getFloat( ( loLimit[2] ) );
        cons.setLowLimit( loLimit );
        fr += 5;
    }
    else
    {
        osg::notify( osg::WARN ) << "BoxConstraint_readLocalData: Bad input data at \"Low limit\"." << std::endl;
        return( false );
    }

    if( fr.matchSequence( "High limit %f %f %f" ) )
    {
        osg::Vec3 hiLimit;
        fr[2].getFloat( ( hiLimit[0] ) );
        fr[3].getFloat( ( hiLimit[1] ) );
        fr[4].getFloat( ( hiLimit[2] ) );
        cons.setHighLimit( hiLimit );
        fr += 5;
    }
    else
    {
        osg::notify( osg::WARN ) << "BoxConstraint_readLocalData: Bad input data at \"High limit\"." << std::endl;
        return( false );
    }

    osg::Matrix m;
    if( readMatrix( m, fr, "Orient" ) )
        cons.setOrientation( m );
    else
    {
        osg::notify( osg::WARN ) << "BoxConstraint_readLocalData: Bad input data at \"Orient\"." << std::endl;
        return( false );
    }

    return( true );
}
bool BoxConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    const osgbDynamics::BoxConstraint& cons = static_cast< const osgbDynamics::BoxConstraint& >( obj );

    fw.indent() << "Low limit " << cons.getLowLimit() << std::endl;
    fw.indent() << "High limit " << cons.getHighLimit() << std::endl;
    writeMatrix( cons.getOrientation(), fw, "Orient" );

    return( true );
}


bool HingeConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    // TBD
    return( true );
}
bool HingeConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    // TBD
    return( true );
}


bool CardanConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    // TBD
    return( true );
}
bool CardanConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    // TBD
    return( true );
}


bool BallAndSocketConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    osgbDynamics::BallAndSocketConstraint& cons = static_cast< osgbDynamics::BallAndSocketConstraint& >( obj );

    if( fr.matchSequence( "Point %f %f %f" ) )
    {
        osg::Vec3 point;
        fr[1].getFloat( ( point[0] ) );
        fr[2].getFloat( ( point[1] ) );
        fr[3].getFloat( ( point[2] ) );
        cons.setPoint( point );
        fr += 4;
    }
    else
    {
        osg::notify( osg::WARN ) << "BallAndSocketConstraint_readLocalData: Bad input data at \"Point\"." << std::endl;
        return( false );
    }

    return( true );
}
bool BallAndSocketConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    const osgbDynamics::BallAndSocketConstraint& cons = static_cast< const osgbDynamics::BallAndSocketConstraint& >( obj );

    fw.indent() << "Point " << cons.getPoint() << std::endl;

    return( true );
}


bool RagdollConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    // TBD
    return( true );
}
bool RagdollConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    // TBD
    return( true );
}


bool WheelSuspensionConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    // TBD
    return( true );
}
bool WheelSuspensionConstraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    // TBD
    return( true );
}
