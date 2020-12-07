# Object Partitioning for Support-Free 3D-Printing

This is the source code for the EuroGraphics 2019 paper ["Object Partitioning for Support-Free 3D-Printing"](https://www.cs.huji.ac.il/~raananf/projects/3dpart/).

![header image](https://www.cs.huji.ac.il/%7Eraananf/projects/3dpart/teaser.jpg)

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
                                                                                                                           
Allowed options:
*  --angle arg (=135)                    critical overhang angle
*  --methods arg (=0.5,0.3,0.1,0.1)      distribution of methods for search (4 values)
*  --bt-linear arg (=0.985)              backtrack linear probability
*  --bt-exp arg (=0.76)                  backtrack exponential probability
*  --file arg                            file to process
*  -a [ --automatic ]                    don't open windows until result
*  --silent                              no debug prints
*  --time-limit arg (=50)                runtime limit in minutes
*  --cut-limit arg (=5000)               amount of cuts to try before stopping
*  --help                                print a help message
