#include <CGAL/Polygon_mesh_processing/measure.h>
#include "RandomCut.h"


RandomCut::RandomCut(SliceApp *app, Polyhedron *shape, Vector3 cut_norm) : CutAlgorithm(app, shape,
                                                                                        Mesh::Face_index(0)),
                                                                           _norm(cut_norm) {}


void RandomCut::preprocess() {
}


void RandomCut::optimize() {
    std::priority_queue<DistanceAndFace> queue;
    Plane3 far_plane = Plane3(CGAL::Origin() - 100 * _norm, _norm);
    for (auto face : this->_shape->get_mesh()->faces()) {
        auto centroid = CGAL::centroid(this->_shape->triangles[face]);
        queue.emplace(face, CGAL::squared_distance(centroid, far_plane));
    }
    auto size = queue.size();
    for (ulong i = 0; i < size / 2; i++)
        queue.pop();
    auto middle = queue.top().face;
    auto cut_plane = Plane3(CGAL::centroid(this->_shape->triangles[middle]), _norm);
    auto cut_option = this->_shape->intersect(cut_plane);
    this->_result = new CutDescription(cut_option);
    this->progress_updated(1.0);
}


std::string RandomCut::name() {
    return "RandomCut";
}


RandomCutFactory::RandomCutFactory(SliceApp *app, Polyhedron *shape) : AlgorithmFactory(app, shape),
                                                                       _distribution(0.0, 1.0) {
    _generator.seed((ulong) time(nullptr));
    for (auto i = 0; i < 1; i++)
        this->add_weight(1);
    this->finalize();
}


CutAlgorithm *RandomCutFactory::construct(int idx) {
    double theta = 2 * M_PI * _distribution(_generator);
    double phi = acos(1 - 2 * _distribution(_generator));
    Vector3 norm(sin(phi) * cos(theta), sin(phi) * sin(theta), cos(phi));
    return new RandomCut(this->_app, this->_shape, norm);
}