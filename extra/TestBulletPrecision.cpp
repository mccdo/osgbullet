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

// Originally contributed by Kevin Godby.

// This test program determins whether Bullet
// was compiled with single or double precision.

// Build this as a standalone Bullet application to determine whether
// Bullet was built with single or double precision, then configure
// the osgBullet CMake variable OSGBULLET_DOUBLE_PRECISION accordingly.

#include <bullet/LinearMath/btScalar.h>

#include <typeinfo>
#include <iostream>

int main(int argc, char **argv) {
    if (typeid(btScalar) == typeid(double)) {
        std::cout << "double" << std::endl;
    } else if (typeid(btScalar) == typeid(float)) {
        std::cout << "float" << std::endl;
    } else {
        std::cerr << "ERROR: Type of btScalar is [" << typeid(btScalar).name() << "]." << std::endl;
        return 1;
    }

    return 0;
}

