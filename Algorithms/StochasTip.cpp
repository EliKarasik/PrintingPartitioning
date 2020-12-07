#include "StochasTip.h"

const int random_iters = 30;

StochasTip::StochasTip(SliceApp *app, Polyhedron *shape, Face reference_face, Vector3 cut_normal)
        : BinarySearchHough(app, shape, reference_face) {
    this->_cut_normal = normalize(cut_normal);
    this->_total_iterations += random_iters;
}

StochasTip::StochasTip(SliceApp *app, Polyhedron *shape, Face reference_face)
        : BinarySearchHough(app, shape, reference_face) {
    //auto plane = find_plane(shape, 5);
    auto best_dist = numeric_limits<double>::infinity();
    shared_ptr<TipDescriptor> best_tip = nullptr;
    for (auto tip : this->_shape->_tips) {
        auto dist = CGAL::squared_distance(tip->point, this->_reference_point);
        if (dist < best_dist) {
            best_dist = dist;
            best_tip = tip;
        }
    }
    auto point = best_tip->point;
    auto plane = Plane3(point, best_tip->direction);
    this->_cut_normal = normalize(plane.orthogonal_vector());
}

void StochasTip::preprocess() {
    this->_best_cut = Plane3(this->_epsilon_reference_point, this->_cut_normal);
}

void StochasTip::optimize() {
    auto initial_normal = this->_cut_normal;
    auto initial_plane = Plane3(this->_epsilon_reference_point, initial_normal);
    auto v1 = normalize(initial_plane.base1()), v2 = normalize(initial_plane.base2());
    std::minstd_rand generator;
    std::uniform_real_distribution<double> shift(cos(M_PI / 3), 1);
    std::uniform_real_distribution<double> rot(0, 2 * M_PI);
    for (auto i = 0; i < 30; i++) {
        auto theta = shift(generator);
        auto phi = rot(generator);
        this->_cut_normal = sin(theta) * (cos(phi) * v1 + sin(phi) * v2) + cos(theta) * initial_normal;
        optimize_iterate();
        emit this->progress_updated(i / (double) this->_total_iterations);
    }
    this->_cut_normal = normalize(this->_best_cut.orthogonal_vector());
    BinarySearchHough::optimize();
}

bool StochasTip::optimize_iterate() {
    //cout << "Cut normal: " << this->_cut_normal << endl;
    this->_shape->growing_cut_tip(this->_cut_normal, this->_reference_face, this->_last_cut_state);
    if (this->_last_cut_state.cut_distance < 1)
        return false;
    //auto cut_option = choose_intersect(this->_last_cut_state.maximal_cut);
    //this->_result = new CutDescription(cut_option);
    if (this->_last_cut_state.good_score > this->_best_score) {
        this->_best_score = this->_last_cut_state.good_score;
        this->_best_cut = this->_last_cut_state.maximal_cut;
    }
    //this->_best_cut = this->_last_cut_state.maximal_cut;
    //cout << "Current Score:" << this->_last_cut_state.good_score << endl;
    //cout << "Best Score: " << this->_best_score << endl;
    return true;
}

K::FT StochasTip::printability_score(shared_ptr<CutOption> base) {
    if (base == nullptr)
        return -1;
    auto cut_result = this->_shape->cut(base);
    bool reference_printable = false;
    InfinityFace infinity_base = Polyhedron::infinity_face(*base);
    K::FT score = 0;
    for (Polyhedron *shape : cut_result) {
        //TODO: this is not really necessary
        auto printable1 = shape->printable(infinity_base);
        auto printable2 = shape->printable(infinity_base.opposite());
        if ((printable1 or printable2) and shape->contains_point(this->_epsilon_reference_point)) {
            score += shape->area();
            reference_printable = true;
        }
    }
    if (cut_result.size() > 2)
        score = -1;
    if (!reference_printable)
        score = -1;
    for (Polyhedron *shape : cut_result)
        delete shape;
    return score;
}

Plane3 StochasTip::find_plane(Polyhedron *poly, int num_faces) {
    auto mesh = poly->get_mesh();
    Vector3 norm_sum(0, 0, 0);
    Face reference_face = poly->face_clamp(this->_reference_point);
    queue<Face> worklist;
    set<Face> reference_side_visited;
    worklist.push(reference_face);
    reference_side_visited.insert(reference_face);
    while (not worklist.empty()) {
        auto current = worklist.front();
        worklist.pop();
        auto current_norm = poly->normals[current];
        norm_sum += current_norm;
        num_faces--;
        if (num_faces == 0)
            break;

        for (auto he : mesh->halfedges_around_face(mesh->halfedge(current))) {
            for (auto neighbor : mesh->faces_around_target(he)) {
                if (neighbor != Mesh::null_face() and
                    reference_side_visited.find(neighbor) == reference_side_visited.end()) {
                    worklist.push(neighbor);
                    reference_side_visited.insert(neighbor);
                }
            }
        }
    }
    return Plane3(this->_reference_point, norm_sum);
}


void StochasTip::perturb_orientation() {
    Vector3 worst_normal = this->_cut_normal;
    auto worst_angle = 1.0;
    while (not this->_last_cut_state.boundary.empty()) {
        auto distance_and_face = this->_last_cut_state.boundary.top();
        this->_last_cut_state.boundary.pop();
        auto face = distance_and_face.face;
        auto face_normal = this->_shape->normals[face];
        auto angle = face_normal * this->_cut_normal;
        if (angle > -0.97 and angle < worst_angle) {
            worst_normal = face_normal;
            worst_angle = angle;
        }
    }
    auto to_add = cos_crit_angle - worst_angle;
    to_add = max(0.1, to_add);
    //cout << "Worst angle: " << worst_angle << endl;
    auto coeff = this->_alpha * to_add;
    this->_cut_normal = normalize((1 - coeff) * this->_cut_normal + coeff * worst_normal);
    //this->_cut_normal = normalize((1 - to_add) * this->_cut_normal + to_add * worst_normal);
    //this->_alpha = this->_alpha * 0.8;
    //this->_orientation = this->_hough.approximate(Plane3(this->_reference_point, this->_cut_normal)).orientation;
}


shared_ptr<CutOption> StochasTip::choose_intersect(InfinityFace infinity_base) {
    shared_ptr<CutOption> cut = this->_shape->intersect_from_face(infinity_base, this->_reference_face);
    return cut;
}


set<Polyhedron *> StochasTip::cut() {
    auto results = CutAlgorithm::cut();
    return results;
}


std::string StochasTip::name() {
    return "Tip";
}


StochasTipFactory::StochasTipFactory(SSliceApp *app, Polyhedron *shape) : AlgorithmFactory(app, shape) {
    for (auto tip : this->_shape->_tips) {
        this->add_weight(1);
    }

//    auto skeleton = *this->_shape->get_skeleton();
//    boost::graph_traits<Skeleton>::vertex_iterator vertices_iter, vertices_end;
//    for (boost::tie(vertices_iter, vertices_end) = vertices(skeleton); vertices_iter != vertices_end; ++vertices_iter) {
//        auto vertex = *vertices_iter;
//        if (boost::out_degree(vertex, skeleton) == 1) {
//            _tips.push_back(vertex);
//            this->add_weight(1);
//        }
//    }
    this->finalize();
}


CutAlgorithm *StochasTipFactory::construct(int idx) {
//    auto skeleton = *this->_shape->get_skeleton();
//    auto vertex_idx = _tips[idx];
//    Point3 point = skeleton[vertex_idx].point;
//    auto adjacency_iterator = boost::adjacent_vertices(vertex_idx, skeleton).first;
//    auto adjacent = *adjacency_iterator;
//    Point3 prev_point = skeleton[adjacent].point;
//    Vector3 up = point - prev_point;
//    point = this->_shape->ray_intersect(point, up);
//    auto plane = Plane3(point, up);

    shared_ptr<TipDescriptor> tip_descriptor = this->_shape->_tips[idx];
    auto point = tip_descriptor->point;
    auto direction = tip_descriptor->direction;
    auto face = this->_shape->ray_intersect(point, direction);
    if (not face.is_valid()) {
        return nullptr;
    }
    return new StochasTip(this->_app, this->_shape, face, direction);
};