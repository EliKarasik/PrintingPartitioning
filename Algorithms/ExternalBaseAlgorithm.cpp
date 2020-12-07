#include <QtNetwork/QtNetwork>
#include <CGAL/Polygon_mesh_processing/bbox.h>
#include <CGAL/Bbox_3.h>
#include "ExternalBaseAlgorithm.h"

ExternalBaseAlgorithm::ExternalBaseAlgorithm(SliceApp *app, Polyhedron *shape, Face reference_face)
        : BinarySearchHough(app, shape, reference_face), _base(shape->infinity_face(this->_reference_face)),
          _fully_printable(false) {
}

ExternalBaseAlgorithm::ExternalBaseAlgorithm(SliceApp *app, Polyhedron *shape, Plane3 base, Face reference_face)
        : BinarySearchHough(app, shape, reference_face), _base(base), _fully_printable(false) {
}

void ExternalBaseAlgorithm::preprocess() {
    this->clamp_point();
    this->_best_cut = this->_base;
    this->_cut_normal = normalize(this->_base.orthogonal_vector());
}

bool ExternalBaseAlgorithm::optimize_iterate() {
    //cout << "Cut normal: " << this->_cut_normal << endl;
    this->_shape->growing_cut_base(this->_base, this->_cut_normal, this->_reference_face, this->_last_cut_state);
    if (this->_last_cut_state.fully_printable) {
        this->_fully_printable = true;
        this->_best_score = -2;
        return false;
    }
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
    //cout << "Best score:" << this->_best_score << endl;
    return true;
}

K::FT ExternalBaseAlgorithm::printability_score(shared_ptr<CutOption> cut) {
    if (cut == nullptr)
        return -1;
    auto cut_result = this->_shape->cut(cut);
    bool reference_printable = false;
    InfinityFace infinity_base = this->_shape->infinity_face(this->_reference_face);
    K::FT score = 0;
    for (Polyhedron *shape : cut_result) {
        bool base_printable = shape->printable(infinity_base);
        bool cut_printable = shape->printable(Polyhedron::infinity_face(*cut));
        if ((base_printable or cut_printable) and shape->contains_point(this->_epsilon_reference_point)) {
            score += shape->area();
            reference_printable = true;
        }
    }
    if (!reference_printable)
        score = -1;
    for (Polyhedron *shape : cut_result)
        delete shape;
    return score;
}

shared_ptr<CutOption>
ExternalBaseAlgorithm::choose_intersect(InfinityFace infinity_face) {
    return this->_shape->intersect_from_face(infinity_face, this->_reference_face);
}

void ExternalBaseAlgorithm::perturb_orientation() {
    auto bad = this->_last_cut_state.boundary.top();
    auto bad_face = bad.face;
    auto bad_center = CGAL::centroid(this->_shape->triangles[bad_face]);
    vector<Point3> boundary_points;
    while (not this->_last_cut_state.currently_intersected.empty()) {
        auto face = this->_last_cut_state.currently_intersected.top().face;
        this->_last_cut_state.currently_intersected.pop();
        boundary_points.push_back(CGAL::centroid(this->_shape->triangles[face]));
    }
    auto bounding_box = bbox_3(boundary_points.begin(), boundary_points.end());
    auto center = Point3((bounding_box.xmin() + bounding_box.xmax()) / 2,
                         (bounding_box.ymin() + bounding_box.ymax()) / 2,
                         (bounding_box.zmin() + bounding_box.zmax()) / 2);
    Vector3 direction = -Vector3(center, bad_center);
    direction = normalize(direction);
    //cout << "Good direction: " << direction << endl;
    this->_cut_normal = normalize((1 - this->_alpha) * this->_cut_normal + this->_alpha * direction);
    this->_alpha *= 0.8;
}


set<Polyhedron *> ExternalBaseAlgorithm::cut() {
    if (_fully_printable) {
        auto copied = this->_shape->copy();
        copied->suggested_base = _base;
        set<Polyhedron *> res;
        res.insert(copied);
        return res;
    }
    auto results = CutAlgorithm::cut();
    for (auto poly : results) {
        poly->suggested_base = _base;
    }
    return results;
}


std::string ExternalBaseAlgorithm::name() {
    return "Base";
}

ExternalBaseFactory::ExternalBaseFactory(SliceApp *app, Polyhedron *shape) : AlgorithmFactory(app, shape) {
    shape->recognize_bases();
    K::FT max_area = 0;
    for (auto area : shape->_base_areas) {
        max_area = max(max_area, area);
    }
    for (uint idx = 0; idx < shape->_bases.size(); idx++) {
        if (shape->_base_face[idx].is_valid())
            this->add_weight(pow(shape->_base_areas[idx] / max_area, 0.5));
    }
    this->finalize();
}


CutAlgorithm *ExternalBaseFactory::construct(int idx) {
    //cout << "Weight: " <<  this->_weights[this->_cur_idx].first << endl;
    //cout << "Area: " << this->_shape->_base_areas[idx] << endl;
    auto face = this->_shape->_base_face[idx];
    return new ExternalBaseAlgorithm(this->_app, this->_shape, this->_shape->_bases[idx], face);
}