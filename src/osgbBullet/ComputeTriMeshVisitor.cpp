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

#include <osg/Transform>
#include <osg/Drawable>
#include <osg/Geode>
#include <osg/PrimitiveSet>
#include <osg/TriangleFunctor>

#include <osgbBullet/ComputeTriMeshVisitor.h>

#include <iostream>

using namespace osgbBullet;
using namespace osg;

struct ComputeTriMeshFunc
{
    ComputeTriMeshFunc()
    {
        vertices = new osg::Vec3Array;

        vertices->clear();
    }

    void inline operator()( const osg::Vec3 v1, const osg::Vec3 v2, const osg::Vec3 v3, bool _temp )
    {
        vertices->push_back( v1 );
        vertices->push_back( v2 );
        vertices->push_back( v3 );
    }

    osg::ref_ptr< osg::Vec3Array > vertices;
};

ComputeTriMeshVisitor::ComputeTriMeshVisitor( osg::NodeVisitor::TraversalMode traversalMode )
    : osg::NodeVisitor( traversalMode )
{
    stack.push_back( osg::Matrix::identity() );
    mesh = new osg::Vec3Array;
}

void ComputeTriMeshVisitor::reset()
{
    stack.clear();
    stack.push_back( osg::Matrix::identity() );
    mesh->clear();
}

void ComputeTriMeshVisitor::apply( osg::Node & node )
{
    traverse( node );
}

void ComputeTriMeshVisitor::apply( osg::Transform & transform )
{
    osg::Matrix matrix;

    matrix = stack.back();

    transform.computeLocalToWorldMatrix( matrix, this );

    pushMatrix( matrix );

    traverse( transform );

    popMatrix();
}

void ComputeTriMeshVisitor::apply( osg::Geode & geode )
{
    for( unsigned int i = 0; i < geode.getNumDrawables(); ++i )
    {
        applyDrawable( geode.getDrawable( i ) );
    }
}

void ComputeTriMeshVisitor::applyDrawable( osg::Drawable * drawable )
{
    osg::TriangleFunctor< ComputeTriMeshFunc > functor;
    drawable->accept( functor );

    const osg::Matrix& matrix = stack.back();
    for( osg::Vec3Array::iterator iter = functor.vertices->begin();
         iter != functor.vertices->end(); ++iter )
    {
        mesh->push_back( *iter * matrix );
    }
}

