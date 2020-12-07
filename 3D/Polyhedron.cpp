#include <assimp/cimport.h>
#include <assimp/Importer.hpp>
#include <assimp/Exporter.hpp>
#include <assimp/scene.h>
#include <assimp/config.h>
#include <assimp/postprocess.h>

#include "Polyhedron.h"
#include <CGAL/Polygon_mesh_processing/bbox.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_ratio_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_stop_predicate.h>
#include <CGAL/Side_of_triangle_mesh.h>
#include <CGAL/subdivision_method_3.h>
#include <CGAL/Bbox_3.h>
#include <sys/stat.h>

#include <QtGui/QQuaternion>
#include <QtGui/QMatrix4x4>

static int poly_idx = 0;
static const int resolution = 100;
typedef CGAL::SM_Vertex_index VertexIndex;

static double scaling_factor = 1;

void Polyhedron::init_maps() {
    normals = _poly->add_property_map<Face, Vector3>("f:normal").first;
    triangles = _poly->add_property_map<Face, Triangle3>("f:triang").first;
    for (auto face : _poly->faces()) {
        auto first_halfedge = _poly->halfedge(face);
        Point3 prev = _poly->point(_poly->source(first_halfedge));
        Point3 cur = _poly->point(_poly->target(first_halfedge));
        Point3 next = _poly->point(_poly->target(_poly->next(first_halfedge)));
        triangles[face] = Triangle3(prev, cur, next);
        normals[face] = normalize(triangles[face].supporting_plane().orthogonal_vector());
    }
}

Polyhedron::Polyhedron(Mesh *inner_poly) : suggested_base(0, 0, 1, 0), merge_base(nullptr),
                                           _idx(poly_idx++), _poly(inner_poly),
                                           _skeleton(nullptr), _resolution(resolution), _aabb(nullptr) {
    init_maps();
}

Polyhedron::Polyhedron(std::string filename) : suggested_base(0, 0, 1, 0), merge_base(nullptr), _idx(poly_idx++),
                                               _filename(filename), _resolution(resolution), _aabb(nullptr) {
    _poly = new Mesh();
    Assimp::Importer importer;
    const struct aiScene *scene = importer.ReadFile(filename.c_str(), aiProcess_JoinIdenticalVertices
                                                                      | aiProcess_Triangulate
    );
    if (scene->mNumMeshes != 1) {
        throw std::runtime_error("Not exactly one mesh in the loaded file");
    }
    auto mesh = scene->mMeshes[0];
    map<Point3, VertexIndex> points_map;
    map<int, VertexIndex> point_indices;
    VertexIndex next_index = VertexIndex(0);
    for (uint i = 0; i < mesh->mNumVertices; i++) {
        Point3 added(mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z);
        if (points_map.find(added) == points_map.end()) {
            _poly->add_vertex(added);
            points_map[added] = next_index;
            next_index++;
        }
        point_indices[i] = points_map.at(added);
    }
    for (uint i = 0; i < mesh->mNumFaces; i++) {
        auto face = mesh->mFaces[i];
        list<VertexIndex> face_vertices;
        for (auto idx_ptr = face.mIndices; idx_ptr != face.mIndices + face.mNumIndices; idx_ptr++) {
            face_vertices.push_back(point_indices.at(*idx_ptr));
        }
        if (face_vertices.front() == face_vertices.back())
            continue;
        auto face_idx = _poly->add_face(face_vertices);
        if (face_idx == Mesh::null_face())
            throw std::runtime_error("Loaded mesh is not manifold");
    }

    list<Mesh *> seperated;
    seperate_ccs(_poly, seperated);
    if (seperated.size() > 1) {
        cout << "More than one connected component in mesh, working only on maximal one" << endl;
        auto maximal = seperated.front();
        for (auto m : seperated) {
            if (m->num_vertices() > maximal->num_vertices())
                maximal = m;
        }
        _poly = maximal;
    }

    for (auto edge : _poly->edges()) {
        if (_poly->is_border(edge)) {
            throw std::runtime_error("Loaded mesh is not closed");
        }
    }

    regularize_mesh();
    init_maps();

    //load_skeleton();
    load_voxelization();
    load_tips();
    //calculate_distances();
    //recognize_junctions();
}

namespace SMS = CGAL::Surface_mesh_simplification;

//void Polyhedron::minimize_mesh() {
//    //SMS::Count_ratio_stop_predicate<Mesh> stop(0.1);
//    SMS::Count_stop_predicate<Mesh> stop(1000);
//    SMS::edge_collapse(*_poly, stop, CGAL::parameters::get_cost(SMS::Edge_length_cost<Mesh>())
//            .get_placement(SMS::Midpoint_placement<Mesh>()));
//    _poly->collect_garbage();
//    //CGAL::Polygon_mesh_processing::isotropic_remeshing(_poly->faces(), 10, *_poly);
//    //
//    //_poly->collect_garbage();
//
//}

void Polyhedron::prepare_aabb() {
    if (_aabb == nullptr)
        _aabb = new PolyhedronAABB(CGAL::faces(*_poly).first, CGAL::faces(*_poly).second, *_poly);
}

Point Polyhedron::UninitializedPoint = Point(-1, -1, -1);

Polyhedron *Polyhedron::copy() {
    auto created = new Mesh(*_poly);
    //CGAL::copy_face_graph(*_poly, *created);
    auto res = new Polyhedron(created);
    res->_skeleton = _skeleton;
    res->_skeleton_mapping = _skeleton_mapping;
    res->_junctions = _junctions;
    res->_tips = _tips;
    res->all_cuts = all_cuts;
    res->suggested_base = suggested_base;
    return res;
}

void clean_isolated(Mesh *mesh) {
    for (auto vertex : mesh->vertices()) {
        if (mesh->is_isolated(vertex))
            mesh->remove_vertex(vertex);
    }
    mesh->collect_garbage();
}


void Polyhedron::seperate_ccs(Mesh *mesh, list<Mesh *> &connected) {
    typedef Mesh::Property_map <Face, boost::graph_traits<Mesh>::faces_size_type> FCCmap;
    FCCmap fccmap = mesh->add_property_map<Face, boost::graph_traits<Mesh>::faces_size_type>("f:CC").first;
    int num = CGAL::Polygon_mesh_processing::connected_components(*mesh, fccmap);
    if (num == 1) {
        clean_isolated(mesh);
        connected.push_back(mesh);
        return;
    }
    auto meshes = new Mesh *[num];
    for (int i = 0; i < num; i++) {
        meshes[i] = new Mesh(*mesh);
        fccmap = meshes[i]->add_property_map<Face, boost::graph_traits<Mesh>::faces_size_type>("f:CC").first;
        vector<int> range;
        range.push_back(i);
        CGAL::Polygon_mesh_processing::keep_connected_components(*meshes[i], range, fccmap);
        meshes[i]->collect_garbage();
        connected.push_back(meshes[i]);
    }
    delete mesh;
    delete[] meshes;
}

Plane3 Polyhedron::infinity_face(Mesh::face_index face) {
    return triangles[face].supporting_plane();
}

bool Polyhedron::printable(bool full) {
    if (this->printable(this->suggested_base))
        return true;
    if (this->printable(this->suggested_base.opposite())) {
        this->suggested_base = this->suggested_base.opposite();
        return true;
    }
    recognize_bases();
    for (auto base : _bases) {
        if (printable(base)) {
            this->suggested_base = base;
            //cout << "set base " << this->suggested_base << endl;
            return true;
        }
    }
    return false;
}

bool Polyhedron::printable(InfinityFace base, const set<Face> &dont_check) {
    K::FT cutoff = printable_cutoff;
    //if (lenient)
    //    cutoff *= 1.5;
    K::FT sum_areas = 0;
    for (auto face : _poly->faces()) {
        if (dont_check.find(face) != dont_check.end())
            continue;
        auto voilate_score = violates_base(base, face);
        sum_areas += voilate_score * CGAL::Polygon_mesh_processing::face_area(face, *_poly);
        if (sum_areas > cutoff) {
            //cout << "BAD PRINTABILITY. Sum areas:" <<  sum_areas << endl;
            return false;
        }
    }
    //cout << "Sum areas:" <<  sum_areas << endl;
    return sum_areas < cutoff;
}

size_t Polyhedron::size() {
    return _poly->num_faces();
}

Point Polyhedron::vertex(size_t idx) {
    return _poly->point(Mesh::vertex_index((uint) idx));
}

Point Polyhedron::vertex_inside(size_t idx) {
    return _poly->point(Mesh::vertex_index((uint) idx));
}

Segment3 Polyhedron::bad_primitive(uint primitive_idx) {
    auto edge = _poly->halfedge(Mesh::edge_index(primitive_idx));
    return Segment3(_poly->point(_poly->source(edge)), _poly->point(_poly->target(edge)));
}

bool Polyhedron::is_bad(uint idx) {
    auto half_edge = _poly->halfedge(Mesh::edge_index(idx));
    auto first_plane = infinity_face(_poly->face(half_edge));
    auto other_point = _poly->point(_poly->target(_poly->next(_poly->opposite(half_edge))));
    auto dist = CGAL::squared_distance(first_plane, other_point);
    if (dist < 0.01) return false;
    if (first_plane.oriented_side(other_point) == CGAL::ON_POSITIVE_SIDE)
        return true;
    auto opposite_face = _poly->face(_poly->opposite(half_edge));
    if (opposite_face == Mesh::null_face())
        return true;
    return violates_base(first_plane, opposite_face) > 0;
}

size_t Polyhedron::primitive_size() {
    return _poly->num_edges();
}

uint Polyhedron::bad_primitive_count() {
    uint count = 0;
    for (uint idx = 0; idx < primitive_size(); idx++) {
        count += is_bad(idx) ? 1 : 0;
    }
    return count;
}

bool Polyhedron::contains_point(Point point) {
    if (this->size() < 4)
        return false; //not really a mesh
    prepare_aabb();
    _aabb->accelerate_distance_queries();
    auto dist = _aabb->squared_distance(point);
    //cout << "Point dist: " << dist << endl;
    if (dist < 0.1)
        return true;
    auto checker = CGAL::Side_of_triangle_mesh<Mesh, K>(*_aabb);
    return checker(point) != CGAL::ON_UNBOUNDED_SIDE;
}

bool Polyhedron::contains_triangle(Triangle3 triangle) {
    for (uint i = 0; i < 3; i++) {
        if (not contains_point(triangle.vertex(i)))
            return false;
    }
    return true;
}

void Polyhedron::get_overlap(Polyhedron *other, set<Face> &overlap) {
    vector<Plane3> intersected_cuts;
    set<Face> on_cut, other_on_cut;
    for (auto c1 : all_cuts) {
        for (auto c2 : other->all_cuts) {
            if (c1 == c2) {
                set<Face> temp_on_cut, temp_other_on_cut;
                faces_from_plane(c1, temp_on_cut);
                other->faces_from_plane(c1, temp_other_on_cut);
                if (not temp_on_cut.empty() and not temp_other_on_cut.empty()) {
                    for (auto i : temp_on_cut)
                        on_cut.insert(i);
                    for (auto i: temp_other_on_cut) {
                        other_on_cut.insert(i);
                    }
                }
            }
        }
    }
    if (on_cut.empty() or other_on_cut.empty())
        return;
    list<Triangle3> other_triangles;
    for (auto fo : other_on_cut) {
        other_triangles.push_back(other->triangles[fo]);
    }

    TriangAABB search(other_triangles.begin(), other_triangles.end());
    search.accelerate_distance_queries();

    for (auto f : on_cut) {
        auto t = triangles[f];
        bool good = true;
        for (uint i = 0; i < 3; i++) {
            good = good and (search.squared_distance(t.vertex(i)) < 1);
        }
        if (good)
            overlap.insert(f);
    }
}

void Polyhedron::regularize_mesh() {
    auto bbox = ::CGAL::Polygon_mesh_processing::bbox(*_poly);
    double x = bbox.xmax() - bbox.xmin();
    double y = bbox.ymax() - bbox.ymin();
    double z = bbox.zmax() - bbox.zmin();
    scaling_factor = _resolution / std::max(x, std::max(y, z));
    for (auto vertex : _poly->vertices()) {
        Point3 old = _poly->point(vertex);
        _poly->point(vertex) = Point3((old.x() - bbox.xmin()) * scaling_factor,
                                      (old.y() - bbox.ymin()) * scaling_factor,
                                      (old.z() - bbox.zmin()) * scaling_factor);
    }
}

Mesh *Polyhedron::get_mesh() {
    return _poly;
}

InfinityFace Polyhedron::infinity_face(CutOption cut) {
    return cut.plane;
}

Polyhedron::~Polyhedron() {
    delete _poly;
    delete _aabb;
}

Face Polyhedron::face_clamp(Point near) {
    for (auto face : _poly->faces()) {
        if (CGAL::squared_distance(near, triangles[face]) < 1) {
            return face;
        }
    }
    return Mesh::null_face();
}

int Polyhedron::point_clamp(Point near) {
    int i = 0;
    for (auto vertex : _poly->vertices()) {
        Point3 candidate = _poly->point(vertex);
        if (CGAL::squared_distance(near, candidate) < 1)
            return i;
        i++;
    }
    return -1;
}

void Polyhedron::project_junctions(Polyhedron *parent) {
    for (auto junction : parent->_junctions) {
        if (contains_point(from_int_point(junction->center)))
            _junctions.push_back(junction);
    }
}

void Polyhedron::project_tips(Polyhedron *parent) {
    for (auto tip : parent->_tips) {
        if (contains_point(tip->point))
            _tips.push_back(tip);
    }
}

int Polyhedron::polyidx() {
    return _idx;
}

K::FT Polyhedron::area() {
    return CGAL::Polygon_mesh_processing::area(*_poly);
}

void Polyhedron::faces_from_plane(Plane3 plane, set<Face> &faces) {
    auto plane_normal = plane.orthogonal_vector();
    plane_normal /= sqrt(CGAL::to_double(plane_normal.squared_length()));
    for (auto face : _poly->faces()) {
        auto face_normal = normals[face];
        face_normal /= sqrt(CGAL::to_double(face_normal.squared_length()));
        auto triangle = triangles[face];
        K::FT dist = 0;
        for (auto i = 0; i < 3; i++) {
            dist = max(CGAL::squared_distance(triangle.vertex(i), plane), dist);
        }
        if (dist < 1 and abs(plane_normal * face_normal) > 0.99) {
            faces.insert(face);
        }
    }
}

Face Polyhedron::face_from_plane(Plane3 plane) {
    set<Face> faces;
    faces_from_plane(plane, faces);
    return *faces.begin();
}

typedef K::Ray_3 Ray3;

Face Polyhedron::ray_intersect(Point3 point, Vector3 direction) {
    prepare_aabb();
    auto ray = Ray3(point, direction);
    auto result = _aabb->first_intersected_primitive(ray);
    if (not result.is_initialized()) {
        return Face(); //nullface
    }
    return boost::get<Face>(result);
}

uint Polyhedron::num_intersections(Plane3 plane) {
    prepare_aabb();
    return (uint) _aabb->number_of_intersected_primitives(plane);
}

int Polyhedron::get_resolution() {
    return _resolution;
}

void Polyhedron::recognize_bases() {
    if (_bases.size() != 0)
        return;
    auto temp_base_map = _poly->add_property_map<Face, int>("f:TempBase").first;
    _base_map = _poly->add_property_map<Face, int>("f:Base").first;
    for (auto f : _poly->faces()) {
        temp_base_map[f] = -1;
        _base_map[f] = -1;
    }

    Mesh *mesh = _poly;

    set<Face> visited;
    vector<vector<Face>> full_bases;
    vector<Plane3> temp_bases;
    vector<double> temp_areas;
    K::FT max_area = 0;
    int base_idx = 0;
    for (auto start_face : mesh->faces()) {
        if (visited.find(start_face) != visited.end())
            continue;
        queue<Face> current_base_boundary;
        vector<Face> current_base;
        set<Face> visited_current_round;
        current_base_boundary.push(start_face);
        Vector3 average_normal = normals[start_face];
        K::FT base_area = 0;
        while (not current_base_boundary.empty()) {
            auto current_face = current_base_boundary.front();
            current_base_boundary.pop();
            auto neighbor_normal = normals[current_face];
            neighbor_normal /= sqrt(neighbor_normal.squared_length());
            if (average_normal * neighbor_normal > 0.97) { //todo
                visited.insert(current_face);
                current_base.push_back(current_face);
                auto area = CGAL::Polygon_mesh_processing::face_area(current_face, *mesh);
                base_area += area;
                double alpha = area / base_area;
                average_normal = normalize(alpha * neighbor_normal + (1 - alpha) * average_normal);
                for (auto neighbor : mesh->faces_around_face(mesh->halfedge(current_face))) {
                    if (neighbor.is_valid() and visited.find(neighbor) == visited.end() and
                        visited_current_round.find(neighbor) == visited_current_round.end()) {
                        visited_current_round.insert(neighbor);
                        current_base_boundary.push(neighbor);
                    }
                }
            }
        }
        //finished expanding base, calculate plane
        auto reference_point = CGAL::centroid(triangles[start_face]);
        auto far_plane = Plane3(reference_point - 100 * average_normal, average_normal);
        K::FT close_dist = pow(_resolution, 2), far_dist = 0;
        for (auto n : current_base) {
            auto d = CGAL::squared_distance(CGAL::centroid(triangles[n]), far_plane);
            if (d < close_dist)
                close_dist = d;
            if (d > far_dist)
                far_dist = d;
        }
        close_dist = sqrt(close_dist);
        far_dist = sqrt(far_dist);
        auto middle = (close_dist + far_dist) / 2 - _resolution;
        //cout << current_base.size() << endl;
        //cout << base_area << endl;
        //cout << middle << endl;
        if (base_area > 5) {
            temp_bases.emplace_back(reference_point + middle * average_normal, average_normal);
            temp_areas.push_back(base_area);
            full_bases.push_back(current_base);
            for (auto f : current_base)
                temp_base_map[f] = base_idx;
            base_idx++;
        }
        max_area = max(max_area, base_area);
    }

    //now we want to get rid of CPA voilations
    int final_idx = 0;
    for (int b = 0; b < base_idx; b++) {
        set<Face> neighbors;
        Plane3 base = temp_bases[b];
        auto base_normal = normalize(base.orthogonal_vector());
        for (auto f : full_bases[b]) {
            for (auto n : _poly->faces_around_face(_poly->halfedge(f))) {
                if (not n.is_valid())
                    continue;
                auto neighbor_base_idx = temp_base_map[n];
                if (neighbor_base_idx != b)
                    neighbors.insert(n);
            }
        }
        set<Face> next_step;
        for (uint i = 0; i < 6; i++) {
            for (auto n : neighbors) {
                for (auto nn : _poly->faces_around_face(_poly->halfedge(n))) {
                    if (nn.is_valid())
                        next_step.insert(nn);
                }
            }
            neighbors = next_step;
        }


        bool bad_base = false;
        for (auto n : neighbors) {
            if (bad_base)
                break;
            bool bad_angle = false;
            auto neighbor_base_idx = temp_base_map[n];
            if (neighbor_base_idx == -1 or neighbor_base_idx == b)
                continue;
            auto neighbor_normal = normalize(temp_bases[neighbor_base_idx].orthogonal_vector());
            auto angle = -base_normal * neighbor_normal;
            if (angle < cos_crit_angle)
                bad_angle = true;
            auto t = triangles[n];
            auto dist = numeric_limits<double>::infinity();
            auto side = CGAL::NEGATIVE;
            for (auto vi = 0; vi < 3; vi++) {
                auto cand_dist = CGAL::squared_distance(base, t.vertex(vi));
                if (cand_dist < dist) {
                    dist = cand_dist;
                    side = base.oriented_side(t.vertex(vi));
                }
            }
            if ((dist > 1 and dist < 2) and side == CGAL::NEGATIVE and bad_angle) {
                bad_base = true;
            }
            if (dist > 1 and (side == CGAL::POSITIVE)) {
                bad_base = true;
            }
        }
        if (bad_base)
            continue;
        for (auto f : full_bases[b]) {
            _base_map[f] = final_idx;
        }
        _bases.push_back(base);
        _base_face.push_back(full_bases[b][rand() % full_bases[b].size()]);
        //_base_face.push_back(full_bases[b][0]);
        _base_areas.push_back(temp_areas[b]);
        final_idx++;
    }
    //cout << "Total bases: " << final_idx << endl;


//    for (auto face : _poly->faces()) {
//        auto base_idx = _base_map[face];
//        if (base_idx == -1)
//            continue;
//        Plane3 base = _bases[base_idx];
//        auto base_normal = normalize(base.orthogonal_vector());
//        for (auto neighbor : _poly->faces_around_face(_poly->halfedge(face))) {
//            auto neighbor_base_idx = _base_map[neighbor];
//            if (neighbor_base_idx == -1 or neighbor_base_idx == base_idx)
//                continue;
//            auto neighbor_normal = normalize(_bases[neighbor_base_idx].orthogonal_vector());
//            auto angle = - base_normal * neighbor_normal;
//            if (angle < cos_crit_angle) {
//                _base_face[base_idx] = Mesh::null_face();
//                _base_face[neighbor_base_idx] = Mesh::null_face();
//            }
//        }
//    }

}

K::Aff_transformation_3 qt_cgal_transform(QMatrix4x4 mat) {
    float *data = mat.data();
    K::Aff_transformation_3 transform(data[0], data[4], data[8], data[12],
                                      data[1], data[5], data[9], data[13],
                                      data[2], data[6], data[10], data[14],
                                      data[15]);
    return transform;
}

BaseUpResult Polyhedron::base_up() {
    auto base_normal = normalize(suggested_base.opposite().orthogonal_vector());
    auto qbase_normal = QVector3D(base_normal.x(), base_normal.y(), base_normal.z());
    auto quaternion = QQuaternion::rotationTo(qbase_normal, QVector3D(0, 0, 1));
    auto qmatrix = QMatrix4x4();
    auto point = CGAL::centroid(triangles[face_from_plane(suggested_base)]);
    qmatrix.rotate(quaternion);
    qmatrix.translate(-point.x(), -point.y(), -point.z());
    auto transform = qt_cgal_transform(qmatrix);
    vector<Point3> points;
    for (auto p : _poly->points()) {
        points.push_back(transform(p));
    }
    for (auto child : merge_children) {
        for (auto p : child->get_mesh()->points()) {
            points.push_back(transform(p));
        }
    }
    auto bounding_box = bbox_3(points.begin(), points.end());
    QMatrix4x4 centering;
    centering.translate(-bounding_box.xmin(), -bounding_box.ymin());
    qmatrix = centering * qmatrix;
    BaseUpResult res;
    res.transform = qmatrix;
    res.xsize = bounding_box.xmax() - bounding_box.xmin();
    res.ysize = bounding_box.ymax() - bounding_box.ymin();
    return res;
}

void Polyhedron::write_mesh(string filename) {
    Mesh transformed;
    CGAL::copy_face_graph(*_poly, transformed);
    auto qt_transform = base_up().transform;
    qt_transform.scale(1 / scaling_factor);
    auto transform = qt_cgal_transform(qt_transform);
    for (auto v : transformed.vertices()) {
        transformed.point(v) = transform(transformed.point(v));
    }

    stringstream temp_stream;
    temp_stream << transformed;
    auto str = temp_stream.str();

    Assimp::Importer importer;
    Assimp::Exporter exporter;
    const struct aiScene *scene = importer.ReadFileFromMemory(str.c_str(), str.length(), 0);
    exporter.Export(scene, "stl", filename.c_str(), filename.size());

}

SerializablePoint3::SerializablePoint3() {}

SerializablePoint3::SerializablePoint3(Point3 point) : point(point) {}

Point3 from_int_point(IntPoint point) {
    return Point3(point.x() + 0.5, point.y() + 0.5, point.z() + 0.5);
}