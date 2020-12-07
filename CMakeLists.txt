cmake_minimum_required(VERSION 3.7)
project(Slice LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
find_package(assimp)
if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    # Assimp CMake is broken on mac
    set(ASSIMP_LIBRARIES "/usr/local/opt/assimp/lib/libassimp.dylib")
endif()
find_package(CGAL COMPONENTS Core)

find_package(Qt5 COMPONENTS Core Gui Widgets 3DCore 3DRender 3DInput 3DExtras)
include(${CGAL_USE_FILE})
find_package(Boost COMPONENTS graph thread system chrono date_time atomic serialization program_options REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)


if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    SET(OPTIMIZATION "-O0 -g")
else(CMAKE_BUILD_TYPE STREQUAL "RELEASE")
    SET(OPTIMIZATION "-O3")
endif()
add_definitions(${OPTIMIZATION})
add_definitions("-Wno-write-strings")
add_definitions("-DCGAL_EIGEN3_ENABLED")

set(SOURCE_FILES main.cpp SliceApp.cpp SliceApp.h  defs.h
        QHeatMap.cpp HoughVisualization.cpp HoughVisualization.h HoughSpace.cpp HoughSpace.h
        3D/Visualizer3D.cpp 3D/Visualizer3D.h 3D/Polyhedron.h 3D/Polyhedron_voxels.cpp 3D/Polyhedron_skeleton.cpp
        3D/PolyhedronVisualizer.cpp 3D/PolyhedronVisualizer.h 3D/Polyhedron.cpp 3D/Polyhedron_cut.cpp
        Algorithms/CutAlgorithm.cpp Algorithms/CutAlgorithm.h
        Algorithms/HoughAlgorithm.cpp Algorithms/HoughAlgorithm.h
        Algorithms/CutAlgorithmWidget.cpp Algorithms/CutAlgorithmWidget.h
        Algorithms/ExternalBaseAlgorithm.cpp  Algorithms/ExternalBaseAlgorithm.h
        Algorithms/BinarySearchHough.cpp Algorithms/BinarySearchHough.h
        Algorithms/StochasTip.cpp Algorithms/StochasTip.h
        Algorithms/ContractionAlgorithm.h Algorithms/ContractionAlgorithm.cpp
        Algorithms/RandomCut.cpp Algorithms/RandomCut.h)
add_executable(Slice ${SOURCE_FILES})
target_link_libraries(Slice Qt5::Core Qt5::Gui Qt5::Widgets Qt5::3DCore Qt5::3DRender Qt5::3DInput Qt5::3DExtras)
target_link_libraries(Slice ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})
target_link_libraries(Slice ${ASSIMP_LIBRARIES})
target_link_libraries(Slice ${EIGEN3_LIBRARIES})
target_link_libraries(Slice ${Boost_LIBRARIES})
target_link_libraries(Slice ${CMAKE_DL_LIBS})
target_link_libraries(Slice Eigen3::Eigen)
