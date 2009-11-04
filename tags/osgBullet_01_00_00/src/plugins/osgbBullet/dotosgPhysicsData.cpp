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

#include "osgbBullet/PhysicsState.h"
#include "osgbBullet/MotionState.h"
#include "osgbBullet/Utils.h"

#include <osgDB/Registry>
#include <osgDB/Input>
#include <osgDB/Output>
#include <osg/MatrixTransform>

#include "osgwTools/AbsoluteModelTransform.h"

#include <iostream>
#include <string>

#include <osg/io_utils>


bool PhysicsData_readLocalData( osg::Object& obj, osgDB::Input& fr );
bool PhysicsData_writeLocalData( const osg::Object& obj, osgDB::Output& fw );

osgDB::RegisterDotOsgWrapperProxy PhysicsData_Proxy
(
    new osgbBullet::PhysicsData,
    "PhysicsData",
    "Object PhysicsData",
    PhysicsData_readLocalData,
    PhysicsData_writeLocalData
);




bool readMatrix( osg::Matrix& matrix, osgDB::Input& fr, const char* keyword="Matrix" )
{
    bool iteratorAdvanced = false;
    
    if (fr[0].matchWord(keyword) && fr[1].isOpenBracket())
    {
        int entry = fr[0].getNoNestedBrackets();

        fr += 2;

        int row=0;
        int col=0;
        double v;
        while (!fr.eof() && fr[0].getNoNestedBrackets()>entry)
        {
            if (fr[0].getFloat(v))
            {
                matrix(row,col)=v;
                ++col;
                if (col>=4)
                {
                    col = 0;
                    ++row;
                }
                ++fr;
            }
            else fr.advanceOverCurrentFieldOrBlock();
        }
        ++fr; // last closing bracket
        iteratorAdvanced = true;
    }        
        
    return iteratorAdvanced;
}

bool writeMatrix( const osg::Matrix& matrix, osgDB::Output& fw, const char* keyword="Matrix" )
{
    fw.indent() << keyword <<" {" << std::endl;
    fw.moveIn();
    fw.indent() << matrix(0,0) << " " << matrix(0,1) << " " << matrix(0,2) << " " << matrix(0,3) << std::endl;
    fw.indent() << matrix(1,0) << " " << matrix(1,1) << " " << matrix(1,2) << " " << matrix(1,3) << std::endl;
    fw.indent() << matrix(2,0) << " " << matrix(2,1) << " " << matrix(2,2) << " " << matrix(2,3) << std::endl;
    fw.indent() << matrix(3,0) << " " << matrix(3,1) << " " << matrix(3,2) << " " << matrix(3,3) << std::endl;
    fw.moveOut();
    fw.indent() << "}"<< std::endl;
    return true;
}





bool PhysicsData_readLocalData( osg::Object& obj, osgDB::Input& fr )
{
    osgbBullet::PhysicsData& pd = static_cast< osgbBullet::PhysicsData& >( obj );
    bool advance( false );

    if( fr.matchSequence( "Version %i" ) )
    {
        fr[1].getUInt( pd._version );
        fr+=2;
        advance = true;

        osg::notify( osg::INFO ) << "OSGB: Found version " << pd._version << std::endl;

        pd._cr = static_cast< osgbBullet::CreationRecord* >( fr.readObject() );
        osg::notify( osg::INFO ) << "OSGB: CreationRecord " << pd._cr.get() << std::endl;

        bool readM = readMatrix( pd._osgTransform, fr, "OSGTransform" );
        osg::notify( osg::INFO ) << "OSGB: OSGTransform " << readM << std::endl;

        readM = readMatrix( pd._bodyWorldTransform, fr, "BodyWorldTransform" );
        osg::notify( osg::INFO ) << "OSGB: BodyWorldTransform " << readM << std::endl;
        if( fr.matchSequence( "Linear velocity %f %f %f" ) )
        {
            osg::Vec3& v( pd._linearVelocity );
            fr[2].getFloat( (v[0]) );
            fr[3].getFloat( (v[1]) );
            fr[4].getFloat( (v[2]) );
            fr += 5;

            osg::notify( osg::INFO ) << "OSGB: Found linear velocity " << v << std::endl;
        }
        if( fr.matchSequence( "Angular velocity %f %f %f" ) )
        {
            osg::Vec3& v( pd._angularVelocity );
            fr[2].getFloat( (v[0]) );
            fr[3].getFloat( (v[1]) );
            fr[4].getFloat( (v[2]) );
            fr += 5;

            osg::notify( osg::INFO ) << "OSGB: Found angular velocity " << v << std::endl;
        }
    }
    else if( fr.matchSequence( "FileName" ) )
    {
        pd._fileName = fr[1].getStr();
        fr+=2;
        advance = true;

        osg::notify( osg::INFO ) << "OSGB: Found fileName " << pd._fileName << std::endl;
    }

    osg::notify( osg::INFO ) << "OSGB: advance " << advance << std::endl;
    return( advance );
}

bool PhysicsData_writeLocalData( const osg::Object& obj, osgDB::Output& fw )
{
    const osgbBullet::PhysicsData& pd = static_cast< const osgbBullet::PhysicsData& >( obj );

    fw.indent() << "Version " << pd._version << std::endl;

    fw.writeObject( *(pd._cr) );

    // The AMT matrix is different from the RB matrix. We need to save it
    // separately here so that we can display the OSG subgraph transformed
    // correctly while waiting for physics data to load.
    btMotionState* motion = pd._body->getMotionState();
    osgbBullet::MotionState* ms = dynamic_cast< osgbBullet::MotionState* >( motion );
    if( ms != NULL )
    {
        osg::Transform* trans = ms->getTransform();
        if( trans->asMatrixTransform() != NULL )
        {
            const osg::Matrix& mt( trans->asMatrixTransform()->getMatrix() );
            writeMatrix( mt, fw, "OSGTransform" );
        }
        else
        {
            osgwTools::AbsoluteModelTransform* amt = dynamic_cast< osgwTools::AbsoluteModelTransform* >( trans );
            if( amt != NULL )
            {
                const osg::Matrix& mt( amt->getMatrix() );
                writeMatrix( mt, fw, "OSGTransform" );
            }
        }
    }

    // Save rigid body state.
    osg::Matrix m( osgbBullet::asOsgMatrix( pd._body->getWorldTransform() ) );
    writeMatrix( m, fw, "BodyWorldTransform" );
    osg::Vec3 lv( osgbBullet::asOsgVec3( pd._body->getLinearVelocity() ) );
    fw.indent() << "Linear velocity " << lv << std::endl;
    osg::Vec3 av( osgbBullet::asOsgVec3( pd._body->getAngularVelocity() ) );
    fw.indent() << "Angular velocity " << av << std::endl;

    if( !pd._fileName.empty() )
        fw.indent() << "FileName \"" << pd._fileName << "\"" << std::endl;

    return( true );
}
