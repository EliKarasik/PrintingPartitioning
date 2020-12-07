#ifndef SLICE_DEFS_H
#define SLICE_DEFS_H

#include <QtCore/QRectF>
#include <QtGui/QPen>
#include <QApplication>
#include <QGraphicsView>

#include <CGAL/point_generators_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Lazy_exact_nt.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arr_naive_point_location.h>
#include <CGAL/Simple_polygon_visibility_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Alpha_shape_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Origin.h>
#include <CGAL/Polyhedron_3.h>


using namespace std;

template <class E>
void throw_with_trace(const E& e) { //no trace on old boost :(
    throw e;
}

typedef unsigned int uint;
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Exact_predicates_exact_constructions_kernel EK;

//2D basics
typedef K::Point_2 Point2;
typedef K::Segment_2 Segment2;
typedef K::Vector_2 Vector2;
typedef K::Line_2 Line2;
typedef K::Direction_2 Direction;
typedef CGAL::Polygon_2<K> CPolygon;
typedef CPolygon::Vertex_const_circulator VertexCirculator;
typedef std::vector<Point2> PointSet;
typedef pair<Point2, Point2> PointPair;
typedef std::vector<PointPair> PointPairSet;

//3D basics
typedef K::Point_3 Point3;
typedef K::Plane_3 Plane3;
typedef K::Vector_3 Vector3;
typedef K::Segment_3 Segment3;
typedef K::Line_3 Line3;
typedef K::Triangle_3 Triangle3;
typedef CGAL::Surface_mesh<K::Point_3> Mesh;
typedef Mesh::Property_map<Mesh::Face_index, bool> FaceBoolMap;
typedef Mesh::Property_map<Mesh::Face_index, int> FaceBaseMap;
typedef Mesh::Property_map<Mesh::Face_index, Triangle3> FaceTriangleMap;
typedef Mesh::Property_map<Mesh::Face_index, Vector3> FaceNormalMap;
typedef boost::graph_traits<Mesh>::vertex_descriptor vertex_descriptor;

template <class V>
V normalize(V vector) {
    return vector / sqrt(vector.squared_length());
}


#define STRETCH_TARGET 200
#define MAX_DIST ((int)(STRETCH_TARGET*0.5))
#define TWO_DIST (2*MAX_DIST+1)
#define ANGLES 101

#include <math.h>

extern double crit_angle;
extern double cos_crit_angle;
const double EPSILON = 0.01;

#endif //SLICE_DEFS_H
