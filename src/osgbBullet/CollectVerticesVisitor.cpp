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

#include <osgbBullet/CollectVerticesVisitor.h>
#include <osg/Transform>
#include <osg/Geometry>
#include <osg/Geode>

#include <iostream>

using namespace osgbBullet;
using namespace osg;


CollectVerticesVisitor::CollectVerticesVisitor( osg::NodeVisitor::TraversalMode traversalMode )
    : osg::NodeVisitor( traversalMode )
{
    verts_ = new osg::Vec3Array;
    reset();
}

void CollectVerticesVisitor::reset()
{
    stack_.clear();
    stack_.push_back( osg::Matrix::identity() );
    verts_->clear();
}

void CollectVerticesVisitor::apply( osg::Transform& transform )
{
    osg::Matrix matrix;
    matrix = stack_.back();
    transform.computeLocalToWorldMatrix( matrix, this );
    pushMatrix( matrix );

    traverse( transform );

    popMatrix();
}

void CollectVerticesVisitor::apply( osg::Geode& geode )
{
    unsigned int idx;
    for( idx = 0; idx < geode.getNumDrawables(); idx++ )
        applyDrawable( geode.getDrawable( idx ) );
}

void CollectVerticesVisitor::applyDrawable( osg::Drawable* drawable )
{
    osg::Geometry* geom = dynamic_cast< osg::Geometry* >( drawable );
    if( geom == NULL )
        return;

    const osg::Vec3Array* in = dynamic_cast< const osg::Vec3Array* >( geom->getVertexArray() );
    if( in == NULL )
    {
        osg::notify( osg::WARN ) << "CollectVerticesVisitor: Non-Vec3 vertex array encountered." << std::endl;
        return;
    }

    const osg::Matrix& m( stack_.back() );
    osg::Vec3Array::const_iterator iter;
    for( iter = in->begin(); iter != in->end(); iter++ )
    {
        verts_->push_back( *iter * m );
    }
}

