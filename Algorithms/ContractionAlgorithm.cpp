#include "ContractionAlgorithm.h"

ContractionAlgorithm::ContractionAlgorithm(SliceApp *app, Polyhedron *poly,
                                           shared_ptr<JunctionDescriptor> junction)
        : ExternalBaseAlgorithm(app, poly, Mesh::Face_index(0)), _junction(junction) {
    //reference face is reset in preprocess
}


ContractionAlgorithm::ContractionAlgorithm(SliceApp *app, Polyhedron *poly, Face reference_face)
        : ExternalBaseAlgorithm(app, poly, reference_face), _junction(nullptr) {
    double best_distance = numeric_limits<double>::infinity();
    for (auto junction : this->_shape->_junctions) {
        auto dist = CGAL::squared_distance(junction->cc_center, this->_reference_point);
        if (dist < best_distance) {
            _junction = junction;
            best_distance = dist;
        }
    }
}


void ContractionAlgorithm::preprocess() {
    if (_junction == nullptr) {
        throw std::runtime_error("No junction for Seperation!");
    }
    Plane3 plane = _junction->plane;
    auto point_on_cut = _junction->small_cc.front();
    auto choice = this->_shape->intersect_single_line(plane, point_on_cut);
    if (choice == nullptr)
        throw_with_trace(std::runtime_error("Can't do initial Contraction cut"));
//    if (choice) {
//        choice->split = false;
//        this->_result = new CutDescription(choice);
//        emit this->request_redraw();
//    }
    //return;
    auto cut_result = this->_shape->cut(choice);
    if (cut_result.size() > 2) {
        cout << "Contraction splits to more than 2 parts.." << endl;
    }
    for (auto shape : cut_result) {
        auto plane_normal = plane.orthogonal_vector();
        plane_normal /= sqrt(CGAL::to_double(plane_normal.squared_length()));
        for (auto face : shape->get_mesh()->faces()) {
            auto face_normal = shape->normals[face];
            if (CGAL::squared_distance(_junction->small_cc.front(), shape->triangles[face]) < 1
                and plane_normal * face_normal > 0.99) {
                this->_reference_face = face;
                this->_shape = shape;
            }
        }
    }
    for (auto shape : cut_result) {
        if (shape != this->_shape)
            delete shape;
    }

    this->_reference_point = point_on_cut;
    this->_epsilon_reference_point = this->_reference_point;
    this->_base = plane;
    this->_cut_normal = normalize(this->_base.orthogonal_vector());
}


void ContractionAlgorithm::optimize() {
    ExternalBaseAlgorithm::optimize();
}


set<Polyhedron *> ContractionAlgorithm::cut() {
    auto plane = this->_result->cut->plane;
    auto point_on_cut = this->_result->cut->segments.front().source();
    shared_ptr<CutOption> cut = this->_original_shape->intersect_single_line(plane, point_on_cut);
    return this->_original_shape->cut(cut, true);
}


std::string ContractionAlgorithm::name() {
    return "T-Junction";
}


ContractionFactory::ContractionFactory(SliceApp *app, Polyhedron *shape) : AlgorithmFactory(app, shape) {
    for (auto junction : this->_shape->_junctions) {
        this->add_weight(1);
    }
}


CutAlgorithm *ContractionFactory::construct(int idx) {
    auto junction = this->_shape->_junctions[idx];
    return new ContractionAlgorithm(this->_app, this->_shape, junction);
}