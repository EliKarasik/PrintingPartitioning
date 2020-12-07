#include <vector>

#include "HoughSpace.h"
#include "QHeatMap.h"

#include "3D/Polyhedron.h"
#include "HoughVisualization.h"

void HoughSpace::close_visualize() {
    if (_visualizer != nullptr)
        _visualizer->close();
    _visualizer = nullptr;
}

void HoughSpace::visualize() {
    _visualizer = new HoughVisualization(this);
    _visualizer->show();
    _visualizer->redraw();
}

HoughSpace::HoughSpace(Point middle) : _middle(middle), _visualizer(nullptr), _selected(Orientation(0,0),0) {
    _vals = new double[ANGLES * ANGLES * TWO_DIST];
    for (int i = 0; i < ANGLES * ANGLES * TWO_DIST; i++)
        _vals[i] = 0;
    for (uint alpha = 0; alpha < ANGLES-1; alpha++) {
        for (uint beta = 0; beta < ANGLES-1; beta++) {
            _orientations.emplace_back(Orientation(alpha, beta));
            if (alpha % 20 == 0 and beta % 20 == 0)
                _coarse_orientations.emplace_back(Orientation(alpha, beta));
        }
    }
}

list<Orientation>& HoughSpace::orientations() {
    return _orientations;
}

void HoughSpace::set_selected(HoughCoords pos) {
    _selected = pos;
    emit selection_changed();
}

HoughCoords HoughSpace::get_selected() {
    return _selected;
}

HoughCoords HoughSpace::approximate(Plane3 infinity_face) {
    double distance = sqrt(CGAL::to_double(CGAL::squared_distance(_middle, infinity_face)));
    Vector3 face_normal = infinity_face.orthogonal_vector();
    face_normal = face_normal / sqrt(CGAL::to_double(face_normal.squared_length()));
    double alpha = atan2(CGAL::to_double(face_normal.y()),CGAL::to_double(face_normal.x()));
    if (alpha < 0)
        alpha += M_PI;
    auto rounded_alpha = (int)round((ANGLES - 1) / M_PI * alpha);
    auto beta = (int)round((ANGLES - 1) / M_PI * acos(CGAL::to_double(face_normal.z())));
    return HoughCoords(Orientation(rounded_alpha,beta), distance);
}

InfinityFace HoughSpace::inverse_hough(HoughCoords coords) {
    double dist = coords.distance;
    //double dist = coords.distance - MAX_DIST / 2;
    double alpha = (M_PI / (ANGLES - 1) * coords.orientation.alpha);
    double beta = (M_PI / (ANGLES - 1) * coords.orientation.beta);
    Vector3 dir(cos(alpha)*sin(beta), sin(alpha)*sin(beta), cos(beta));
    return Plane3(_middle + dir * (-dist), dir);
}

void HoughSpace::set_value(HoughCoords coords, double value) {
    if (coords_index(coords) >= ANGLES * ANGLES * TWO_DIST)
        throw std::runtime_error("OOB");
    _vals[coords_index(coords)] = value;
}

double HoughSpace::get_value(HoughCoords coords) {
    return _vals[coords_index(coords)];
}

int HoughSpace::coords_index(HoughCoords coords) {
    int dist = ((int)coords.distance) + MAX_DIST;
    return dist * (ANGLES*ANGLES) + coords.orientation.beta * ANGLES + coords.orientation.alpha;
}

HoughSpace::~HoughSpace() {
    delete[] _vals;
}

list<Orientation> &HoughSpace::coarse_orientations() {
    return _coarse_orientations;
}

void HoughSpace::reset_middle(Point middle) {
    _middle = middle;
}

void HoughSpace::similar_orientations(Orientation initial, list<Orientation>& orientation_list, int scale) {
    for (int i = -2; i <= 2; i++) {
        for (int j = -2; j <= 2; j++) {
            auto alpha = (initial.alpha + i*scale) % ANGLES; if (alpha < 0) alpha += ANGLES;
            auto beta = (initial.beta + j*scale) % ANGLES; if (beta < 0) beta += ANGLES;
            orientation_list.emplace_back(Orientation(alpha, beta));
        }
    }
}


Orientation::Orientation(int alpha, int beta) : alpha(alpha), beta(beta) {}

bool Orientation::operator<(const Orientation& other) const {
    if (alpha < other.alpha)
        return true;
    if (alpha > other.alpha)
        return false;
    return beta < other.beta;
}

HoughCoords::HoughCoords(Orientation orientation, double distance) : orientation(orientation), distance(distance) {}

HoughCoords::HoughCoords() : orientation(Orientation(-1,-1)), distance(0) {}