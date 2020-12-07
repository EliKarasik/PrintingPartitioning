# Object Partitioning for Support-Free 3D-Printing

This is the source code for the EuroGraphics 2019 paper ["Object Partitioning for Support-Free 3D-Printing"](https://www.cs.huji.ac.il/~raananf/projects/3dpart/).

## Dependencies and Building

* CGAL
* Boost
* Qt5
* assimp

Use CMake to build

On OS X:
* brew install cgal boost qt assimp
* may need to set CMAKE_PREFIX_PATH=/usr/local/opt/qt/

On OpenSuse:
* Install:
    * cgal-devel
    * libqt5-qtbase-common-devel, libQt5Gui-devel, libQt5OpenGL-devel, libQt5Widgets-devel, libqt5-qtsvg-devel, libQt5Concurrent-devel, libqt3d-devel
    * assimp-devel
    * boost-
* In some cases, this may be required: ln -s /usr/lib64/libGLU.so.1 /usr/lib64/libGLU.so

## Usage


