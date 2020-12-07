#ifndef SLICE_POLYHEDRON_H
#define SLICE_POLYHEDRON_H

#include "../defs.h"
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_halfedge_graph_segment_primitive.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <CGAL/Mean_curvature_flow_skeletonization.h>
#include <QtGui/QMatrix4x4>

class Visualizer3D;

typedef CGAL::AABB_tree<CGAL::AABB_traits<K, CGAL::AABB_face_graph_triangle_primitive<Mesh>>> PolyhedronAABB;
typedef CGAL::AABB_tree<CGAL::AABB_traits<K, CGAL::AABB_triangle_primitive<K, std::list<Triangle3>::iterator>>> TriangAABB;
typedef CGAL::Mean_curvature_flow_skeletonization<Mesh> Skeletonization;

//typedef Skeletonization::Skeleton Skeleton;
class SerializablePoint3 {
public:
    Point3 point;

    template<class Archive>
    void save(Archive &ar, const unsigned int version) const {
        ar << point.x();
        ar << point.y();
        ar << point.z();
    }

    template<class Archive>
    void load(Archive &ar, const unsigned int version) {
        K::FT x, y, z;
        ar >> x;
        ar >> y;
        ar >> z;
        point = Point3(x, y, z);
    }

    SerializablePoint3();

    SerializablePoint3(Point3 point);

    BOOST_SERIALIZATION_SPLIT_MEMBER()
};

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, SerializablePoint3> Skeleton;

typedef CGAL::Simple_cartesian<int> IntKernel;
typedef IntKernel::Point_3 IntPoint;

Point3 from_int_point(IntPoint point);

typedef IntKernel::Vector_3 IntVector;
typedef Point3 Point;
typedef Plane3 InfinityFace;
typedef Mesh::Face_index Face;

struct JunctionDescriptor {
    IntPoint center;
    vector<Point3> small_cc;
    Point3 cc_center;
    Plane3 plane;
    int radius;
};

struct TipDescriptor {
    Point3 point;
    Vector3 direction;
    bool original;
};

struct DistanceAndFace {
    K::FT distance;
    Mesh::Face_index face;

    DistanceAndFace(Mesh::Face_index face, K::FT distance) : distance(distance), face(face) {}

    bool operator<(const DistanceAndFace &other) const {
        return distance > other.distance;
    }
};

struct GrowingCutState {
    Plane3 maximal_cut;
    set<Mesh::Face_index> component;
    priority_queue<DistanceAndFace> boundary;
    priority_queue<DistanceAndFace> currently_intersected;
    K::FT good_score, bad_score;
    K::FT cut_distance;
    bool fully_printable;
};

const double printable_cutoff = 10;

typedef pair<Mesh::Vertex_index, Mesh::Vertex_index> VertexPair;
struct CutDS {
    Mesh *poly;
    Mesh *refined;
    map<Point3, VertexPair> vertices; //indices in refined
    map<Point3, Point2> projection_map;
    map<Point2, VertexPair> projected_vertices; //indices in refined
    //set<Point3> added_points;
    K::Aff_transformation_3 rotation;
    map<Mesh::Face_index, int> coloring; //indices in refined
};

struct BaseUpResult {
    QMatrix4x4 transform;
    double xsize;
    double ysize;
};

struct CutOption {
    Plane3 plane;
    //vector<Point3> polyline;
    map<Face, Segment3> face_to_segment;
    vector<Segment3> segments;
    set<Face> parallel;

    CutOption(Plane3 plane);
};

class Polyhedron {
public:

    typedef Segment3 BadPrimitive;

    static Point UninitializedPoint;

    Plane3 infinity_face(Face face);

    void faces_from_plane(Plane3, set<Face> &faces);

    Face face_from_plane(Plane3 plane);

    static InfinityFace infinity_face(CutOption cut);

    Polyhedron(string file_name);

    ~Polyhedron();

    Polyhedron *copy();


    void growing_cut(Vector3 base_normal, Vector3 cut_normal, Face start, Plane3 base, GrowingCutState &state,
                     double cutoff = printable_cutoff);

    void growing_cut_base(Plane3 base, Vector3 cut_normal, Face start, GrowingCutState &state);

    void growing_cut_tip(Vector3 cut_normal, Face start, GrowingCutState &state);

    shared_ptr<CutOption> intersect_from_face(Plane3 base, Face face); //one obj on reference side
    shared_ptr<CutOption> intersect(Plane3 base); //no reference point - both sides can have more than one
    shared_ptr<CutOption> intersect_single_line(Plane3 base, Point3 closest); //only a single polyline
    void refine_poly(shared_ptr<CutOption> &cut, CutDS &ds);

    set<Polyhedron *> cut(shared_ptr<CutOption> cut, bool preserve_metadata = false);

    Polyhedron *merge(Polyhedron *other, bool full = false);

    void get_overlap(Polyhedron *other, set<Face> &overlap);

    K::FT cut_area(shared_ptr<CutOption> cut);

    bool printable(bool full = false);

    bool printable(InfinityFace base, const set<Face> &dont_check = set<Face>());


    size_t size();

    Point vertex(size_t idx);

    Point vertex_inside(size_t idx);

    int point_clamp(Point near);

    Face face_clamp(Point near);

    Face ray_intersect(Point3 point, Vector3 direction);

    bool contains_point(Point point);

    bool contains_triangle(Triangle3 triangle);

    BadPrimitive bad_primitive(uint primitive_idx);

    bool is_bad(uint primitive_idx);

    size_t primitive_size();

    uint bad_primitive_count();

    K::FT area();

    uint num_intersections(Plane3 plane);

    Mesh *get_mesh();

    int polyidx();

    shared_ptr<Skeleton> get_skeleton();


    void load_voxelization();

    void load_tips();

    void voxelize();

    void recognize_junctions();

    void recognize_tips();

    void max_suppress(vector<double> &values);

    shared_ptr<JunctionDescriptor> junctions_in_cube(IntPoint center, int max_radius);

    void tips_from_plane(Plane3 plane);

    void voxel_cube(IntPoint center, int max_radius);

    void calculate_distances();

    IntPoint closest_point(IntPoint point);

    inline int voxel_idx(IntPoint point);

    void project_junctions(Polyhedron *parent);

    void project_tips(Polyhedron *parent);

    vector<bool> _voxels;
    vector<shared_ptr<JunctionDescriptor>> _junctions;
    vector<shared_ptr<TipDescriptor>> _tips;
    vector<int> _distances;

    void recognize_bases();

    vector<double> _base_areas;
    vector<Plane3> _bases;
    vector<Face> _base_face;
    FaceBaseMap _base_map;
    FaceNormalMap normals;
    FaceTriangleMap triangles;

    vector<Plane3> all_cuts;


    void skeletonize();

    void load_skeleton();

    void trim_skeleton(shared_ptr<Skeleton> skeleton);

    Plane3 suggested_base;
    Polyhedron *merge_base;
    vector<Polyhedron *> merge_children;
    map<Point3, vector<Mesh::Vertex_index>> _skeleton_mapping;

    int get_resolution();

    K::Aff_transformation_3 project_2(Plane3 plane);

    BaseUpResult base_up();

    void write_mesh(string filename);

private:
    int _idx;
    Mesh *_poly;
    std::string _filename;
    shared_ptr<Skeleton> _skeleton;
    int _resolution;

    Polyhedron(Mesh *inner_poly);

    PolyhedronAABB *_aabb;

    void regularize_mesh();

    void minimize_mesh();

    void seperate_ccs(Mesh *mesh, list<Mesh *> &connected);

    void prepare_aabb();

    void init_maps();

public:
    const double local_minima_score = 100;

    inline double violates_base(Plane3 base, Face face) {
        if (base.is_degenerate())
            return local_minima_score;

        double first_layer_sensitivity = 1;

        bool same_plane = true;
        K::FT min_dist = numeric_limits<double>::infinity();
        Mesh::vertex_index lower_vertex;
        for (auto v : _poly->vertices_around_face(_poly->halfedge(face))) {
            auto point = _poly->point(v);
            auto dist = CGAL::squared_distance(point, base);
            if (dist > first_layer_sensitivity) {
                same_plane = false;
                if (base.oriented_side(point) == CGAL::ON_POSITIVE_SIDE) //todo
                    return local_minima_score;
            }
            if (dist < min_dist) {
                min_dist = dist;
                lower_vertex = v;
            }
        }
        if (same_plane)
            return 0;

        Vector3 base_normal = normalize(base.orthogonal_vector());
        Vector3 face_normal = normals[face];
        auto cosine = -base_normal * face_normal;
        if (cosine < cos_crit_angle) {
            return 1;
        }
        if (cosine > 0 or min_dist < first_layer_sensitivity) //local minima irrelevant
            return 0;

        //local minima check
        for (auto v : _poly->vertices_around_target(_poly->halfedge(lower_vertex))) {
            auto dist = CGAL::squared_distance(_poly->point(v), base);
            if (dist < min_dist)
                return 0;
        }
        return local_minima_score;
    }

};

typedef pair<Polyhedron *, Polyhedron *> PolyhedronPair;

#endif //SLICE_POLYHEDRON_H
