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


bool Constraint_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    return( false );
}
bool Constraint_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    return( false );
}


bool SliderConstraint_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    osgbDynamics::SliderConstraint& sc = static_cast< osgbDynamics::SliderConstraint& >( obj );

    osg::Matrix m;
    if( readMatrix( m, fr, "rbAXform" ) )
        sc.setAXform( m );
    else
    {
        osg::notify( osg::WARN ) << "SliderConstraint_readLocalData: Bad input data at \"rbAXform\"." << std::endl;
        return( false );
    }

    if( readMatrix( m, fr, "rbBXform" ) )
        sc.setBXform( m );
    else
    {
        osg::notify( osg::WARN ) << "SliderConstraint_readLocalData: Bad input data at \"rbBXform\"." << std::endl;
        return( false );
    }

    if( fr.matchSequence( "Axis %f %f %f" ) )
    {
        osg::Vec3 axis;
        fr[1].getFloat( ( axis[0] ) );
        fr[2].getFloat( ( axis[1] ) );
        fr[3].getFloat( ( axis[2] ) );
        sc.setAxisInA( axis );
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
        sc.setLimit( limit );
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
    const osgbDynamics::SliderConstraint& sc = static_cast< const osgbDynamics::SliderConstraint& >( obj );

    writeMatrix( sc.getAXform(), fw, "rbAXform" );
    writeMatrix( sc.getBXform(), fw, "rbBXform" );
    fw.indent() << "Axis " << sc.getAxisInA() << std::endl;
    fw.indent() << "Limit " << sc.getLimit() << std::endl;

    return( true );
}

