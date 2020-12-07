#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include "Polyhedron.h"

namespace PMP = CGAL::Polygon_mesh_processing;

typedef boost::optional<PolyhedronAABB::Intersection_and_primitive_id<Plane3>::Type> Plane_intersection;

double approximate(double d) {
    return round(d * 1e5) / 1e5;
}

Point3 approximate(Point3 point) {
    return Point3(approximate(point.x()), approximate(point.y()), approximate(point.z()));
}

Mesh::halfedge_index find_halfedge(Mesh *mesh, Face face, Point3 point) {
    for (auto he : mesh->halfedges_around_face(mesh->halfedge(face))) {
        auto edge = Segment3(mesh->point(mesh->source(he)), mesh->point(mesh->target(he)));
        if (CGAL::squared_distance(point, edge) < 1e-8)
            return he;
    }
    return Mesh::null_halfedge();
}

//K::FT Polyhedron::cut_area(shared_ptr<CutOption> cut) {
//    Skeleton graph;
//    map<Point3, int> vertices;
//    for (auto face_and_segment : cut->face_to_segment) {
//        Segment3 segment = face_and_segment.second;
//        auto source = segment.source();
//        if (vertices.find(source) == vertices.end()) vertices[source] = boost
//        boost::add_vertex(graph, segment.source())
//    }
//}

K::FT Polyhedron::cut_area(shared_ptr<CutOption> cut) {
    auto rotation = project_2(cut->plane);
    list<list<Point2>> polylines;
    auto face_to_segment = cut->face_to_segment;
    map<Point3, Point3> first_neighbor, second_neighbor;
    for (auto face_and_segment : face_to_segment) {
        auto segment = face_and_segment.second;
        auto source = segment.source();
        auto target = segment.target();
        if (source == target) {
            //cout << "megafail" << endl;
            continue;
        }
        if (first_neighbor.find(source) == first_neighbor.end())
            first_neighbor[source] = target;
        else
            second_neighbor[source] = target;
        if (first_neighbor.find(target) == first_neighbor.end())
            first_neighbor[target] = source;
        else
            second_neighbor[target] = source;
    }
    set<Point3> processed;
    int i = 0;
    for (auto point_pair : first_neighbor) {
        auto source = point_pair.first;
        if (processed.find(source) != processed.end())
            continue;
        polylines.emplace_back();
        auto target = source;
        Point3 prev;
        do {
            processed.insert(target);
            auto projected3 = target.transform(rotation);
            auto projected2 = Point2(projected3.x(), projected3.y());
            polylines.back().push_back(projected2);
            if (first_neighbor[target] != prev) {
                prev = target;
                target = first_neighbor[target];
            } else {
                prev = target;
                target = second_neighbor[target];
            }
            i++;
            if (i == 100000)
                exit(0);
        } while (target != source);
    }
    K::FT sum = 0;
    for (auto polyline : polylines) {
        CPolygon poly(polyline.begin(), polyline.end());
        //sum += poly.area();
        sum = max(sum, abs(poly.area()));
    }
    return sum;
}

shared_ptr<CutOption> Polyhedron::intersect(Plane3 cut) {
    //auto normal = cut.orthogonal_vector();
    //normal /= sqrt(CGAL::to_double(normal.squared_length()));
    auto option = make_shared<CutOption>(cut);
    list<Plane_intersection> intersections;
    prepare_aabb();
    _aabb->all_intersections(cut, std::back_inserter(intersections));
    if (intersections.empty())
        return nullptr;
    for (auto intersection_pair : intersections) {
        auto primitive = intersection_pair->second;
        //auto prim_normal = infinity_face(primitive).orthogonal_vector();
        //prim_normal /= sqrt(CGAL::to_double(prim_normal.squared_length()));
        //if (abs(prim_normal * normal)> 0.999) {
        //    //parallel.insert(primitive);
        //    continue;
        //}
        //auto triangle = triangle_from_face(primitive);
        auto intersection = intersection_pair->first;
        if (intersection.which() == 1) {
            auto segment = boost::get<Segment3>(intersection);
            Point3 segment_points[2] = {approximate(segment.source()), approximate(segment.target())};
//            for (int i = 0; i < 3; i++) {
//                bool one_clamped = false;
//                for (int j = 0; j < 2; j++) {
//                    if (CGAL::squared_distance(segment_points[j], triangle.vertex(i)) < 1e-1) {
//                        segment_points[j] = triangle.vertex(i);
//                    }
//                    one_clamped = true;
//                }
//                if (one_clamped) {
//                    for (int j = 0; j < 2; j++) {
//                        if (CGAL::squared_distance(segment_points[j], triangle.vertex(i)) < 2e-1)
//                            segment_points[j] = triangle.vertex(i);
//                    }
//                }
//            }

            segment = Segment3(segment_points[0], segment_points[1]);
            //can have empty segments
            //if (segment.squared_length() < 0.01) {
            //    segment = Segment3(source, source);
            //}
            option->face_to_segment[primitive] = segment;
            option->segments.push_back(segment);
        } else if (intersection.which() == 2) { //triangle
            cout << "triangle intersect" << endl;
        } else if (intersection.which() == 0) { //point intersect
            cout << "point intersect" << endl;
        }
    }
    return option;
}

CutOption::CutOption(Plane3 plane) : plane(plane) {}

//TODO: this is still a lie?
shared_ptr<CutOption> Polyhedron::intersect_single_line(Plane3 cut, Point3 closest) {
    auto unclean = this->intersect(cut);
    if (unclean == nullptr)
        return nullptr;
    double best_distance = numeric_limits<double>::infinity();
    Face face;
    for (auto facesegment : unclean->face_to_segment) {
        auto dist = CGAL::squared_distance(triangles[facesegment.first], closest);
        if (dist < best_distance) {
            face = facesegment.first;
            best_distance = dist;
        }
    }

    auto clean = std::make_shared<CutOption>(cut);
    queue<Face> ws;
    ws.push(face);
    set<Face> processed;
    processed.insert(face);
    while (!ws.empty()) {
        auto current = ws.front();
        ws.pop();
        auto segment = unclean->face_to_segment[current];
        clean->face_to_segment[current] = segment;
        clean->segments.push_back(segment);
        for (auto neighbor : _poly->faces_around_face(_poly->halfedge(current))) {
            if (unclean->face_to_segment.find(neighbor) != unclean->face_to_segment.end() and
                processed.find(neighbor) == processed.end()) {
                ws.push(neighbor);
                processed.insert(neighbor);
            }
        }
    }
    return clean;
}


shared_ptr<CutOption> Polyhedron::intersect_from_face(Plane3 cut, Face reference_face) {
    shared_ptr<CutOption> unclean = intersect(cut);
    if (unclean == nullptr)
        return nullptr;
    queue<Face> workset;
    set<Face> reference_side_visited;
    workset.push(reference_face);
    reference_side_visited.insert(reference_face);
    auto option = std::make_shared<CutOption>(cut);
    while (not workset.empty()) {
        auto current = workset.front();
        workset.pop();
        //auto current_norm = infinity_face(current).orthogonal_vector();
        //marked.insert(current);
        //current_norm /= CGAL::sqrt(current_norm.squared_length());
        auto face_halfedge = _poly->halfedge(current);
        //if (abs(current_norm * normal) > 0.999 and CGAL::squared_distance(cut, _poly->point(_poly->target(he))) < 1e-4) {
        //    option->parallel.insert(current);
        //    cout << "parallel" << endl;
        //    continue;
        //else if
        if (unclean->face_to_segment.find(current) !=
            unclean->face_to_segment.end()) { //make sure not to divide parallel
            option->face_to_segment[current] = unclean->face_to_segment[current];
            option->segments.push_back(unclean->face_to_segment[current]);
            auto segment = unclean->face_to_segment[current];
            //insert only touching cut neighbors.
            for (int i = 0; i < 2; i++) {
                auto point = segment.point(i);
                auto he = find_halfedge(_poly, current, point);
                auto neighbor = _poly->face(_poly->opposite(he));
                if (neighbor.is_valid() and reference_side_visited.find(neighbor) == reference_side_visited.end()) {
                    workset.push(neighbor);
                    reference_side_visited.insert(neighbor);
                }
            }
            continue;
        }
        for (auto he : _poly->halfedges_around_face(face_halfedge)) {
            for (auto neighbor : _poly->faces_around_target(he)) {
                if (neighbor != Mesh::null_face() and
                    reference_side_visited.find(neighbor) == reference_side_visited.end()) {
                    workset.push(neighbor);
                    reference_side_visited.insert(neighbor);
                }
            }
        }
    }
    return option;
}


struct FaceInfo {
    int nesting_level;

    bool in_domain() { return nesting_level % 2 == 1; }
};

typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo, K> FaceBaseWithInfo;
typedef CGAL::Constrained_triangulation_face_base_2<K, FaceBaseWithInfo> ConstrainedFace;
typedef CGAL::Triangulation_data_structure_2<CGAL::Triangulation_vertex_base_2<K>, ConstrainedFace> TDS;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, CGAL::Exact_predicates_tag> CDT;

void mark_domains(CDT &ct, CDT::Face_handle start, int index, std::list<CDT::Edge> &border) {
    if (start->info().nesting_level != -1) {
        return;
    }
    std::list<CDT::Face_handle> queue;
    queue.push_back(start);
    while (!queue.empty()) {
        CDT::Face_handle fh = queue.front();
        queue.pop_front();
        if (fh->info().nesting_level == -1) {
            fh->info().nesting_level = index;
            for (int i = 0; i < 3; i++) {
                CDT::Edge e(fh, i);
                CDT::Face_handle n = fh->neighbor(i);
                if (n->info().nesting_level == -1) {
                    if (ct.is_constrained(e)) border.push_back(e);
                    else queue.push_back(n);
                }
            }
        }
    }
}

void mark_domains(CDT &cdt) {
    for (auto it = cdt.all_faces_begin(); it != cdt.all_faces_end(); ++it) {
        it->info().nesting_level = -1;
    }
    std::list<CDT::Edge> border;
    mark_domains(cdt, cdt.infinite_face(), 0, border);
    while (!border.empty()) {
        CDT::Edge e = border.front();
        border.pop_front();
        CDT::Face_handle n = e.first->neighbor(e.second);
        if (n->info().nesting_level == -1) {
            mark_domains(cdt, n, e.first->info().nesting_level + 1, border);
        }
    }
}

VertexPair add_to_ds(Point3 point, Mesh::Vertex_index idx, CutDS &ds) {
    auto search_iter = ds.vertices.find(point);
    if (search_iter != ds.vertices.end())
        return search_iter->second;
    if (idx == Mesh::null_vertex())
        idx = ds.refined->add_vertex(point);
    if (idx == Mesh::null_vertex())
        throw_with_trace(std::runtime_error("Can't add new vertex to DS"));
    Mesh::Vertex_index idx2 = ds.refined->add_vertex(point);
    auto pair = VertexPair(idx, idx2);
    ds.vertices[point] = pair;
    auto projected_point_3 = point.transform(ds.rotation);
    auto projected_point = Point2(projected_point_3.x(), projected_point_3.y());
    ds.projection_map[point] = projected_point;
    ds.projected_vertices[projected_point] = pair;
    return pair;
}

Mesh::Vertex_index find_in_face(Point3 point, Mesh::Face_index face, Mesh *mesh) {
    for (auto vertex : mesh->vertices_around_face(mesh->halfedge(face))) {
        //if (mesh->point(vertex) == point)
        //    return vertex;
        if (CGAL::squared_distance(mesh->point(vertex), point) < 1e-5)
            return vertex;
    }
    return Mesh::null_vertex();
}

K::Aff_transformation_3 Polyhedron::project_2(Plane3 plane) {
    auto norm = sqrt(CGAL::to_double(plane.a() * plane.a() + plane.b() * plane.b() + plane.c() * plane.c()));
    auto cosine = CGAL::to_double(plane.c()) / norm;
    auto sine = sqrt(1 - cosine * cosine);
    auto u1 = CGAL::to_double(plane.b()) / norm;
    auto u2 = CGAL::to_double(-plane.a()) / norm;
    K::Aff_transformation_3 rotation(cosine + u1 * u1 * (1 - cosine), u1 * u2 * (1 - cosine), u2 * sine,
                                     u1 * u2 * (1 - cosine), cosine + u2 * u2 * (1 - cosine), -u1 * sine,
                                     -u2 * sine, u1 * sine, cosine);
    return rotation;
}

void Polyhedron::refine_poly(shared_ptr<CutOption> &cut, CutDS &ds) {
    Mesh *poly = _poly;
    ds.refined = new Mesh();
    for (uint i = 0; i < poly->num_vertices(); i++)
        ds.refined->add_vertex(poly->point(Mesh::Vertex_index(i)));

    //cut faces!
    set<Mesh::Face_index> added_faces; //indices in the original poly
    bool uberfail = false;

    for (auto face_and_segment : cut->face_to_segment) {
        auto face = face_and_segment.first;
        if (added_faces.find(face) != added_faces.end())
            cout << "readding an existing face?!";
        added_faces.insert(face);

        auto segment = face_and_segment.second;
        auto first = segment.source(), second = segment.target();
        Mesh::Vertex_index first_idx = find_in_face(first, face, poly), second_idx = find_in_face(second, face, poly);
        Mesh::Face_index idx1, idx2, idx3;

        //actually not cutting?
//        if (first_idx != poly->null_vertex() and second_idx != poly->null_vertex()) {
//            cout << "Single add: " << face << endl;
//            for (auto v : poly->vertices_around_face(poly->halfedge(face))) {
//                cout << v << " ";
//            }
//            cout << endl;
//            add_to_ds(first, first_idx, ds);
//            add_to_ds(second, second_idx, ds);
//            auto he = poly->halfedge(face);
//            idx1 = ds.refined->add_face(poly->vertices_around_face(he));
//            if (idx1 == poly->null_face())
//                throw std::runtime_error("Decided no divide - can't add");
//            if (first_idx == second_idx)
//                continue; //no coloring to do
//            //find the correct he
//            while (not ((poly->source(he) == first_idx and poly->target(he) == second_idx) or
//                        (poly->source(he) == second_idx and poly->target(he) == first_idx)))
//                he = poly->next(he);
//            auto other = poly->point(poly->target(poly->next(he)));
//            auto color = cut->plane.oriented_side(other);
//            ds.coloring[idx1] = color;
//            //color the other side as well
//            auto opposite_he = poly->opposite(he);
//            other = poly->point(poly->target(poly->next(opposite_he)));
//            if (CGAL::squared_distance(other, cut->plane) > 0.1) {
//                //idx2 = refined->add_face(poly->vertices_around_face(opposite_he));
//                //coloring[idx2] = -color;
//                //added_faces.insert(poly->face(opposite_he));
//            }
//            continue;
//        }
//
//        if (first_idx != poly->null_vertex() and second_idx != poly->null_vertex()) {
//            cout << "BUMMER" << endl;
//        }

//        if (two) {
//            add_to_ds(first, first_idx, ds);
//            second_idx = add_to_ds(second, ds.refined->null_vertex(), ds);
//            //cout << first_idx << " " << second_idx << endl;
//            auto he = find_halfedge(poly, face, second);
//            Mesh::Vertex_index src = poly->source(he), dst = poly->target(he);
//            auto next = poly->next(he); auto next_target = poly->target(next);
//            int color;
//            if (src == first_idx) { //first -> second -> dst
//                color = cut->plane.oriented_side(poly->point(next_target));
//                idx1 = ds.refined->add_face(first_idx, second_idx, poly->target(next));
//                idx2 = ds.refined->add_face(second_idx, dst, poly->target(next));
//                ds.coloring[idx1] = color; ds.coloring[idx2] = color;
//            } else if (dst == first_idx) { //src -> second -> first
//                color = cut->plane.oriented_side(poly->point(next_target));
//                idx1 = ds.refined->add_face(second_idx, first_idx, poly->target(next));
//                idx2 = ds.refined->add_face(second_idx, poly->target(next), src);
//                ds.coloring[idx1] = color; ds.coloring[idx2] = color;
//            } else { //the other side
//                color = cut->plane.oriented_side(poly->point(dst));
//                idx1 = ds.refined->add_face(first_idx, second_idx, dst);
//                idx2 = ds.refined->add_face(first_idx, src, second_idx);
//                ds.coloring[idx1] = color; ds.coloring[idx2] = -color;
//            }
//            if (idx1 == poly->null_face() or idx2 == poly->null_face())
//                throw std::runtime_error("Couldn't divide face - two triangles case");
//            continue;
//        }

        //cutting to 3!
        auto prev_halfedge = Mesh::null_halfedge(), cur_halfedge = Mesh::null_halfedge();
        double prev_dist = 1e-8, cur_dist = 1e-8;
        for (auto he : poly->halfedges_around_face(poly->halfedge(face))) {
            auto edge = Segment3(poly->point(poly->source(he)), poly->point(poly->target(he)));
            auto second_distance = CGAL::squared_distance(edge, second);
            auto first_distance = CGAL::squared_distance(edge, first);
            if (first_distance < prev_dist) {
                if (prev_halfedge != Mesh::null_halfedge())
                    uberfail = true;
                prev_dist = first_distance;
                prev_halfedge = he;
            }
            if (second_distance < cur_dist) {
                if (cur_halfedge != Mesh::null_halfedge())
                    uberfail = true;
                cur_dist = second_distance;
                cur_halfedge = he;
            }
        }

        if (prev_halfedge == Mesh::null_halfedge() or cur_halfedge == Mesh::null_halfedge()) {
            for (auto he : poly->halfedges_around_face(poly->halfedge(face))) {
                auto edge = Segment3(poly->point(poly->source(he)), poly->point(poly->target(he)));
                cerr << CGAL::squared_distance(edge, first) << " " << CGAL::squared_distance(edge, second) << endl;
            }
            throw_with_trace(std::runtime_error("Dividing face - 3 case: points not on edges"));
        }

        if (poly->source(cur_halfedge) != poly->target(prev_halfedge)) {
            std::swap(first, second);
            std::swap(prev_halfedge, cur_halfedge);
        }

        VertexPair first_pair = add_to_ds(first, Mesh::null_vertex(), ds);
        VertexPair second_pair = add_to_ds(second, Mesh::null_vertex(), ds);
        //cout << first_idx << " " << second_idx << endl;
        Mesh::Vertex_index prev_src = poly->source(prev_halfedge), tip = poly->target(prev_halfedge);
        Mesh::Vertex_index cur_target = poly->target(cur_halfedge);

        auto tip_point = poly->point(tip);
        int color = cut->plane.oriented_side(tip_point);
        Mesh::Vertex_index first_tip, second_tip, first_other, second_other;
        if (color == CGAL::POSITIVE) {
            first_tip = first_pair.first;
            first_other = first_pair.second;
            second_tip = second_pair.first;
            second_other = second_pair.second;
        } else {
            first_tip = first_pair.second;
            first_other = first_pair.first;
            second_tip = second_pair.second;
            second_other = second_pair.first;
        }
        idx1 = ds.refined->add_face(first_tip, tip, second_tip);
        idx2 = ds.refined->add_face(first_other, cur_target, prev_src);
        idx3 = ds.refined->add_face(first_other, second_other, cur_target);
        if (idx1 == Mesh::null_face() or idx2 == Mesh::null_face() or idx3 == Mesh::null_face())
            throw_with_trace(std::runtime_error("Dividing face - 3 case: can't add faces"));
        ds.coloring[idx1] = color;
        ds.coloring[idx2] = -color;
        ds.coloring[idx3] = -color;

    }

    //add non-affected faces
    //cout << cut->parallel.size() << endl;
    for (auto face : cut->parallel) {
        auto idx1 = ds.refined->add_face(poly->vertices_around_face(poly->halfedge(face)));
        if (idx1 == Mesh::null_face())
            throw_with_trace(std::runtime_error("Can't add parallel face"));
        added_faces.insert(face);
        //auto color = (infinity_face(face).orthogonal_vector() * cut->plane.orthogonal_vector() > 0) ? -1 : 1;
        //coloring[idx1] = color;
    }
    for (auto face : poly->faces()) {
        if (added_faces.find(face) == added_faces.end()) {
            auto idx1 = ds.refined->add_face(poly->vertices_around_face(poly->halfedge(face)));
            if (idx1 == Mesh::null_face()) {
                for (auto v : poly->vertices_around_face(poly->halfedge(face))) {
                    cerr << v << " ";
                }
                cerr << endl;
                throw_with_trace(std::runtime_error("Can't add non-divided face"));
            }
        }
    }
}

set<Polyhedron *> Polyhedron::cut(shared_ptr<CutOption> cut, bool preserve_metadata) {
    CutDS ds;
    ds.poly = _poly;
    ds.rotation = project_2(cut->plane);
    refine_poly(cut, ds);

    //Self implemented triangulation. still buggy.
    CDT t;
    //cout << "new triang";
    //cout << cut->segments.size() << endl;
    for (auto segment : cut->segments) {
        if (segment.source() == segment.target())
            continue;
        //cout << segment.source() << " " << segment.target() << endl;
        //cout << projection_map[segment.source()] << " " << projection_map[segment.target()] << endl;
        t.insert_constraint(ds.projection_map[segment.source()], ds.projection_map[segment.target()]);
    }
    mark_domains(t);
    for (auto cell_iter = t.finite_faces_begin(); cell_iter != t.finite_faces_end(); ++cell_iter) {
        if (not cell_iter->info().in_domain())
            continue;
        auto v0p = cell_iter->vertex(0)->point();
        auto v1p = cell_iter->vertex(1)->point();
        auto v2p = cell_iter->vertex(2)->point();
        if (ds.projected_vertices.find(v0p) == ds.projected_vertices.end() or
            ds.projected_vertices.find(v1p) == ds.projected_vertices.end() or
            ds.projected_vertices.find(v2p) == ds.projected_vertices.end())
            throw_with_trace(std::runtime_error("projected point not saved"));
        auto v0 = ds.projected_vertices[v0p];
        auto v1 = ds.projected_vertices[v1p];
        auto v2 = ds.projected_vertices[v2p];

        Mesh::Face_index idx1, idx2;
        auto norm = CGAL::normal(ds.refined->point(v0.first), ds.refined->point(v1.first), ds.refined->point(v2.first));
        if (norm * cut->plane.orthogonal_vector() < 0) {
            idx1 = ds.refined->add_face(v0.first, v1.first, v2.first);
            idx2 = ds.refined->add_face(v2.second, v1.second, v0.second);
        } else {
            idx1 = ds.refined->add_face(v2.first, v1.first, v0.first);
            idx2 = ds.refined->add_face(v0.second, v1.second, v2.second);
        }
        if (idx1 == Mesh::null_face() or idx2 == Mesh::null_face()) {
            cerr << ds.refined->point(v0.first) << " , ";
            cerr << ds.refined->point(v1.first) << " , ";
            cerr << ds.refined->point(v2.first) << endl;
            throw_with_trace(std::runtime_error("3D Cut - couldn't add triangulation face"));
        }
    }

    set<Polyhedron *> result;
    list<Mesh *> components;
    seperate_ccs(ds.refined, components);
    for (auto mesh : components) {
        auto poly = new Polyhedron(mesh);
        poly->suggested_base = cut->plane;
        poly->all_cuts = all_cuts;
        poly->all_cuts.push_back(cut->plane);
        result.insert(poly);
    }
    if (preserve_metadata) {
        for (auto res : result) {
            res->trim_skeleton(_skeleton);
            res->project_junctions(this);
            res->project_tips(this);
            auto face = res->face_from_plane(cut->plane);
            if (face != Mesh::null_face()) { //TODO: fully remake it
                auto new_tip = make_shared<TipDescriptor>();
                new_tip->direction = cut->plane.orthogonal_vector();
                new_tip->direction /= sqrt(new_tip->direction.squared_length());
                new_tip->point = CGAL::centroid(triangles[face]);
                res->_tips.push_back(new_tip);
            } else {
                cout << "megafail" << endl;
            }
        }
    }
    return result;
}

typedef Mesh::Property_map <vertex_descriptor, EK::Point_3> Exact_point_map;
typedef Mesh::Property_map<vertex_descriptor, bool> Exact_point_computed;

struct Coref_point_map {
    // typedef for the property map
    typedef boost::property_traits<Exact_point_map>::value_type value_type;
    typedef boost::property_traits<Exact_point_map>::reference reference;
    typedef boost::property_traits<Exact_point_map>::category category;
    typedef boost::property_traits<Exact_point_map>::key_type key_type;
    // exterior references
    Mesh *mesh_ptr;
    Exact_point_computed _exact_point_computed;
    Exact_point_map _exact_point;

    Exact_point_computed exact_point_computed() const {
        return _exact_point_computed;
    }

    Exact_point_map exact_point() const {
        return _exact_point;
    }

    Mesh *mesh() const {
        return mesh_ptr;
    }

    // Converters
    CGAL::Cartesian_converter<K, EK> to_exact;
    CGAL::Cartesian_converter<EK, K> to_input;

    Coref_point_map() : mesh_ptr(nullptr) {

    }

    explicit Coref_point_map(Mesh *m)
            : mesh_ptr(m) {
        _exact_point_computed = m->add_property_map<vertex_descriptor, bool>("e:exact_points_computed").first;
        _exact_point = m->add_property_map<vertex_descriptor, EK::Point_3>("e:exact_point").first;
    }

    friend
    reference get(const Coref_point_map &map, key_type k) {
        // create exact point if it does not exist
        if (!map.exact_point_computed()[k]) {
            map.exact_point()[k] = map.to_exact(map.mesh()->point(k));
            map.exact_point_computed()[k] = true;
        }
        return map.exact_point()[k];
    }

    friend
    void put(const Coref_point_map &map, key_type k, const EK::Point_3 &p) {
        map.exact_point_computed()[k] = true;
        map.exact_point()[k] = p;
        // create the input point from the exact one
        map.mesh()->point(k) = map.to_input(p);
    }
};

Polyhedron *Polyhedron::merge(Polyhedron *other, bool full) {
    set<Face> this_overlap, other_overlap;
    get_overlap(other, this_overlap);
    other->get_overlap(this, other_overlap);
    cout << size() << " " << other->size() << endl;
    cout << "Overlap:" << this_overlap.size() << " " << other_overlap.size() << endl;
    cout << "First Base " << this->suggested_base << endl;
    cout << "Second Base " << other->suggested_base << endl;
    if ((this_overlap.size() == 0) or (other_overlap.size() == 0))
        return nullptr;
    Plane3 existing_base(0, 0, 0, 0);
    if (other->printable(this->suggested_base, other_overlap)) {
        existing_base = this->suggested_base;
        cout << "FIRST BASE EXISTS" << endl;
    } else if (this->printable(other->suggested_base, this_overlap)) {
        existing_base = other->suggested_base;
        cout << "SECOND BASE EXISTS" << endl;
    } else if (not full) {
        return nullptr;
    }
    //do the real merge
    auto self_pm = Coref_point_map(_poly);
    auto other_mesh = other->get_mesh();
    auto other_pm = Coref_point_map(other_mesh);
    auto result = new Mesh();
    auto result_pm = Coref_point_map(result);
    bool good = PMP::corefine_and_compute_union(*_poly, *other->get_mesh(), *result,
                                                PMP::parameters::vertex_point_map(self_pm),
                                                PMP::parameters::vertex_point_map(other_pm),
                                                PMP::parameters::vertex_point_map(result_pm)
    );
    if (not good) {
        delete result;
        return nullptr;
    }
    typedef Mesh::Property_map <Face, boost::graph_traits<Mesh>::faces_size_type> FCCmap;
    FCCmap fccmap = result->add_property_map<Face, boost::graph_traits<Mesh>::faces_size_type>("f:CC").first;
    int num = CGAL::Polygon_mesh_processing::connected_components(*result, fccmap);
    if (num > 1) {
        delete result;
        return nullptr;
    }
    //sucessfully merged, make sure we have a base
    auto res = new Polyhedron(result);
    for (auto cut : all_cuts)
        res->all_cuts.push_back(cut);
    for (auto cut : other->all_cuts)
        res->all_cuts.push_back(cut);
    //set base if have it
    res->suggested_base = existing_base;
    //return result if printable
    if (not res->printable(full)) {
        delete res;
        res = nullptr;
    }
    return res;
}


inline K::FT distance(Triangle3 face, Plane3 plane, bool close = true) {
    auto dist = CGAL::squared_distance(face.vertex(0), plane);
    for (uint i = 1; i <= 2; i++) {
        auto cand_dist = CGAL::squared_distance(face.vertex(i), plane);
        if (close and cand_dist < dist)
            dist = cand_dist;
        if (not close and cand_dist > dist)
            dist = cand_dist;
    }
    return dist;
}

Point3 close_point(Triangle3 face, Plane3 plane) {
    auto res = face.vertex(0);
    auto dist = CGAL::squared_distance(res, plane);
    for (uint i = 1; i <= 2; i++) {
        auto cand_dist = CGAL::squared_distance(face.vertex(i), plane);
        if (cand_dist < dist) {
            dist = cand_dist;
            res = face.vertex(i);
        }
    }
    return res;
}

void Polyhedron::growing_cut_base(Plane3 base, Vector3 cut_normal, Face start, GrowingCutState &state) {
    auto base_normal = normalize(base.orthogonal_vector());
    growing_cut(base_normal, cut_normal, start, base, state);
}

void Polyhedron::growing_cut_tip(Vector3 cut_normal, Face start, GrowingCutState &state) {
    growing_cut(-cut_normal, cut_normal, start, Plane3(0, 0, 0, 0), state);
}

void Polyhedron::growing_cut(Vector3 base_normal, Vector3 cut_normal, Face start, Plane3 base_plane,
                             GrowingCutState &state, double cutoff) {
    while (not state.boundary.empty()) state.boundary.pop();
    while (not state.currently_intersected.empty()) state.currently_intersected.pop();
    state.component.clear();
    state.bad_score = 0;
    state.good_score = 0;
    state.fully_printable = false;

    auto start_triangle = triangles[start];
    auto far_back_point = CGAL::Origin() + (cut_normal * 1000);
    auto initial_plane = Plane3(far_back_point, cut_normal);
    auto start_plane = Plane3(close_point(start_triangle, initial_plane), cut_normal);
    if (base_plane.is_degenerate()) {
        auto far_forward_point = CGAL::Origin() - (cut_normal * 1000);
        base_plane = Plane3(far_forward_point, base_normal);
    }
    set<Face> visited;
    visited.insert(start);
    state.boundary.emplace(start, distance(start_triangle, initial_plane));
    K::FT max_distance = 0;
    state.maximal_cut = initial_plane;
    state.cut_distance = 0;
    while (not state.boundary.empty() and state.bad_score < cutoff) {
        //cout << "Boundary size " <<  boundary.size() << endl;
        auto next = state.boundary.top();
        state.boundary.pop();
        max_distance = next.distance;
        state.maximal_cut = Plane3(close_point(triangles[next.face], initial_plane) + 0.001 * cut_normal, cut_normal);
        //state.maximal_cut = Plane3(close_point(triangle_from_face(next.face), initial_plane), cut_normal);
        state.cut_distance = sqrt(distance(triangles[next.face], start_plane));
        while (not state.currently_intersected.empty() and state.currently_intersected.top().distance < max_distance)
            state.currently_intersected.pop();
        auto area = CGAL::Polygon_mesh_processing::face_area(next.face, *_poly);
        auto base_test = violates_base(base_plane, next.face);
        if (base_test > 0) {
            state.bad_score += base_test * area;
        } else
            state.good_score += area;
        if (state.bad_score > cutoff) {
            state.boundary.push(next); //reinsert back to boundary and stop
            break;
        }
        state.currently_intersected.emplace(next.face, distance(triangles[next.face], initial_plane, false));
        state.component.insert(next.face);
        priority_queue<DistanceAndFace> back_boundary;
        //cout << " max distance " << max_distance << endl;
        for (auto f : _poly->faces_around_face(_poly->halfedge(next.face))) {
            if (f.is_valid() and visited.find(f) == visited.end()) {
                auto dist = distance(triangles[f], initial_plane);
                if (dist > max_distance) {
                    state.boundary.emplace(f, dist);
                } else {
                    back_boundary.emplace(f, dist);
                }
                visited.insert(f);
            }
        }

        //back enclaves
        K::FT enclave_bad_score = state.bad_score, enclave_good_score = state.good_score;
        while (not back_boundary.empty() and back_boundary.top().distance < max_distance) {
            auto top_face = back_boundary.top().face;
            back_boundary.pop();
            auto area = CGAL::Polygon_mesh_processing::face_area(top_face, *_poly);
            auto base_test = violates_base(base_plane, top_face);
            if (base_test > 0) {
                enclave_bad_score += base_test * area;
            } else
                enclave_good_score += area;
            if (enclave_bad_score > cutoff)
                break;
            for (auto f : _poly->faces_around_face(_poly->halfedge(top_face))) {
                if (f.is_valid() and visited.find(f) == visited.end()) {
                    auto dist = distance(triangles[f], initial_plane);
                    back_boundary.emplace(f, dist);
                    visited.insert(f);
                }
            }
        }
        //cout << enclave_bad_score;
        if (enclave_bad_score > cutoff)
            break;
        while (not back_boundary.empty()) {
            state.boundary.push(back_boundary.top());
            back_boundary.pop();
        }
        state.bad_score = enclave_bad_score;
        state.good_score = enclave_good_score;
    }
    state.fully_printable = state.boundary.empty();
}