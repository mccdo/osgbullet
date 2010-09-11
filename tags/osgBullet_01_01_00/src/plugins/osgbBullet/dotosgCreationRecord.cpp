/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgBullet is (C) Copyright 2009 by Kenneth Mark Bryden
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

#include "osgbBullet/CreationRecord.h"

#include <iostream>
#include <string>

#include <osg/io_utils>

#include <osgDB/Registry>
#include <osgDB/Input>
#include <osgDB/Output>


bool Creation_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool Creation_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy Creation_Proxy
(
    new osgbBullet::CreationRecord,
    "CreationRecord",
    "Object CreationRecord",
    Creation_readLocalData,
    Creation_writeLocalData
);




bool Creation_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    osgbBullet::CreationRecord& cr = static_cast< osgbBullet::CreationRecord& >( obj );
    bool advance( false );

    if( fr.matchSequence( "Version %i" ) )
    {
        fr[1].getUInt( cr._version );
        fr+=2;
        advance = true;
    }
    else if( fr.matchSequence( "COM %f %f %f" ) )
    {
        float x, y, z;
        fr[1].getFloat( x );
        fr[2].getFloat( y );
        fr[3].getFloat( z );
        cr._com = osg::Vec3( x, y, z );
        fr+=4;
        advance = true;
    }
    else if( fr.matchSequence( "Use COM" ) )
    {
        cr._comSet = ( fr[2].matchString( "true" ) );
        fr+=3;
        advance = true;
    }
    else if( fr.matchSequence( "Scale %f %f %f" ) )
    {
        float x, y, z;
        fr[1].getFloat( x );
        fr[2].getFloat( y );
        fr[3].getFloat( z );
        cr._scale = osg::Vec3( x, y, z );
        fr+=4;
        advance = true;
    }
    else if( fr.matchSequence( "Collision shape %i" ) )
    {
        unsigned int uint;
        fr[2].getUInt( uint );
        cr._shapeType = (BroadphaseNativeTypes)( uint );
        fr+=3;
        advance = true;
    }
    else if( fr.matchSequence( "Mass %f" ) )
    {
        fr[1].getFloat( cr._mass );
        fr+=2;
        advance = true;
    }
    else if( fr.matchSequence( "Decimator percent %f" ) )
    {
        fr[2].getFloat( cr._decimatorPercent );
        fr+=3;
        advance = true;
    }
    else if( fr.matchSequence( "Decimator max error %f" ) )
    {
        fr[3].getFloat( cr._decimatorMaxError );
        fr+=4;
        advance = true;
    }
    else if( fr.matchSequence( "Decimator ignore boundaries" ) )
    {
        cr._decimatorIgnoreBoundaries = ( fr[3].matchString( "true" ) );
        fr+=4;
        advance = true;
    }
    else if( fr.matchSequence( "Simplify percent %f" ) )
    {
        fr[2].getFloat( cr._simplifyPercent );
        fr+=3;
        advance = true;
    }
    else if( fr.matchSequence( "VertexAgg max verts %i" ) )
    {
        fr[3].getUInt( cr._vertexAggMaxVerts );
        fr+=4;
        advance = true;
    }
    else if( fr.matchSequence( "VertexAgg min cell size %f %f %f" ) )
    {
        float x, y, z;
        fr[4].getFloat( x );
        fr[5].getFloat( y );
        fr[6].getFloat( z );
        cr._vertexAggMinCellSize = osg::Vec3( x, y, z );
        fr+=7;
        advance = true;
    }
    else if( fr.matchSequence( "Reducer group threshold %f" ) )
    {
        fr[3].getFloat( cr._reducerGroupThreshold );
        fr+=4;
        advance = true;
    }
    else if( fr.matchSequence( "Reducer max edge error %f" ) )
    {
        fr[4].getFloat( cr._reducerMaxEdgeError );
        fr+=5;
        advance = true;
    }
    else if( fr.matchSequence( "Overall" ) )
    {
        cr._overall = fr[1].matchString( "true" );
        fr+=2;
        advance = true;
    }
    else if( fr.matchSequence( "Node name" ) )
    {
        cr._nodeName = fr[2].getStr();
        fr+=3;
        advance = true;
    }
    else if( fr.matchSequence( "Cylinder axis %i" ) )
    {
        unsigned int uint;
        fr[2].getUInt( uint );
        cr._axis = (osgbBullet::AXIS)( uint );
        fr+=3;
        advance = true;
    }

    return( advance );
}

bool Creation_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    const osgbBullet::CreationRecord& cr = static_cast< const osgbBullet::CreationRecord& >( obj );

    fw.indent() << "Version " << 1 << std::endl;
    fw.indent() << "COM " << cr._com << std::endl;
    fw.indent() << "Use COM " << std::boolalpha << cr._comSet << std::endl;
    fw.indent() << "Scale " << cr._scale << std::endl;
    fw.indent() << "Collision shape " << (unsigned int)( cr._shapeType ) << std::endl;
    fw.indent() << "Mass " << cr._mass << std::endl;

    fw.indent() << "Decimator percent " << cr._decimatorPercent << std::endl;
    fw.indent() << "Decimator max error " << cr._decimatorMaxError << std::endl;
    fw.indent() << "Decimator ignore boundaries " << cr._decimatorIgnoreBoundaries << std::endl;
    fw.indent() << "Simplify percent " << cr._simplifyPercent << std::endl;
    fw.indent() << "VertexAgg max verts " << cr._vertexAggMaxVerts << std::endl;
    fw.indent() << "VertexAgg min cell size " << cr._vertexAggMinCellSize << std::endl;
    fw.indent() << "Reducer group threshold " << cr._reducerGroupThreshold << std::endl;
    fw.indent() << "Reducer max edge error " << cr._reducerMaxEdgeError << std::endl;

    fw.indent() << "Overall " << cr._overall << std::endl;
    if( !cr._nodeName.empty() )
        fw.indent() << "Node name " << cr._nodeName << std::endl;
    fw.indent() << "Cylinder axis " << cr._axis << std::endl;

    return( true );
}
