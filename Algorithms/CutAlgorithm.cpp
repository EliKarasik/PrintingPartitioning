#include "CutAlgorithm.h"
#include "../SliceApp.h"
#include "CutAlgorithmWidget.h"

#include "../3D/Polyhedron.h"

Point copy_point(Point point) {
    return Point3(CGAL::to_double(point.x()), CGAL::to_double(point.y()), CGAL::to_double(point.z()));
}

CutAlgorithm::CutAlgorithm(SliceApp *app, Polyhedron *shape, Face reference_face) :
        _canceled(false), _do_step(true), _app(app), _shape(shape->copy()), _original_shape(_shape),
        _reference_face(reference_face),
        _reference_point(CGAL::centroid(shape->triangles[reference_face])),
        _epsilon_reference_point(_reference_point),
        _result(nullptr), _widget(nullptr) {

    _color = QColor::fromRgb(rand() % 128, rand() % 128, rand() % 128, 255);
}


void CutAlgorithm::clamp_point() {
    int clamped_point = _shape->point_clamp(_reference_point);
    if (clamped_point != -1) {
        _reference_point = _shape->vertex(clamped_point);
        _epsilon_reference_point = _shape->vertex_inside(clamped_point);
    }
}

QColor CutAlgorithm::get_color() {
    return _color;
}

CutAlgorithm::~CutAlgorithm() {
    delete _widget;
    delete _result;
    delete _shape;
}

CutAlgorithmWidget *CutAlgorithm::get_widget(bool create) {
    if (!_widget and create) {
        _widget = new CutAlgorithmWidget(this);
    }
    return _widget;
}

CutDescription *CutAlgorithm::get_result() {
    return _result;
}

void CutAlgorithm::visualize() {}

void CutAlgorithm::close_visualize() {}

void CutAlgorithm::join() {
    _future.waitForFinished();
}

void CutAlgorithm::quit() {
    _canceled = true;
    _future.waitForFinished();
}

void CutAlgorithm::run() {
    _future = QtConcurrent::run(this, &CutAlgorithm::calculate);
}

void CutAlgorithm::calculate() {
    try {
        preprocess();
        optimize();
    } catch (exception &e) {
        cerr << "Fail:" << e.what() << endl;
    };
}

void CutAlgorithm::set_idx(int idx) {
    _algorithm_idx = idx;
}

int CutAlgorithm::get_idx() {
    return _algorithm_idx;
}

K::FT CutAlgorithm::get_score() {
    return _score;
}

set<Polyhedron *> CutAlgorithm::cut() {
    return _original_shape->cut(_result->cut, true);
}


CutDescription::CutDescription(shared_ptr<CutOption> cut) : cut(cut) {}

CutScore::CutScore() : angle(M_PI), pairs_score(0), printable(-1) {}

double CutScore::to_double() const {
    return pairs_score * 5 + (M_PI - angle) + 1000 * printable; //todo: work on this
}

bool CutScore::operator<(const CutScore &rhs) const {
    return to_double() < rhs.to_double();
}

AlgorithmFactory::AlgorithmFactory(SliceApp *app, Polyhedron *shape) : _shape(shape), _app(app), _cur_idx(0),
                                                                       _distribution(0.0, 1.0) {
    _generator.seed((ulong) time(nullptr));
}

void AlgorithmFactory::add_weight(double weight) {
    auto sample = _distribution(_generator);
    weight = pow(sample, 1 / weight);
    _weights.emplace_back(std::make_pair(weight, _weights.size()));
}

void AlgorithmFactory::finalize() {
    std::sort(_weights.begin(), _weights.end(), [this](const auto &t1, const auto &t2) { return t2.first < t1.first; });
}

CutAlgorithm *AlgorithmFactory::pick() {
    if (_cur_idx >= _weights.size())
        return nullptr;
    auto res = construct(_weights[_cur_idx].second);
    _cur_idx++;
    return res;
}

AlgorithmFactory::~AlgorithmFactory() {}



