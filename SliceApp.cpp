#include <utility>
#include <QtWidgets/QVBoxLayout>
#include <boost/range/adaptor/map.hpp>
#include <QtGui/QVector2D>
#include "SliceApp.h"
#include "Algorithms/CutAlgorithmWidget.h"
#include "Algorithms/ExternalBaseAlgorithm.h"
#include "Algorithms/StochasTip.h"

#include "3D/Visualizer3D.h"
#include "3D/Polyhedron.h"
#include "Algorithms/ContractionAlgorithm.h"
#include "Algorithms/RandomCut.h"

SliceApp::SliceApp(QApplication* app, Polyhedron* shape, string filename, std::vector<double>& algorithm_distribution,
                      double backtrack_lin, double backtrack_exp, bool debug_prints, int timelimit, int cut_limit) :
        _app(app),  _current_shape(nullptr), _backtrack_button(nullptr), _next_algorithm(0),
        _selected_algorithm(nullptr),  _visualizer(nullptr), _best_size(100), _max_depth(15), _distribution(0.0, 1.0),
        _algorithm_distribution(std::move(algorithm_distribution)), _backtrack_lin(backtrack_lin),
        _backtrack_expbase(backtrack_exp), _debug_prints(debug_prints), _time_limit(timelimit),
        _cut_limit(cut_limit), _filename(filename), _screenid(rand()),  shapes_mutex(QMutex::Recursive){
    _generator.seed((ulong)time(nullptr));
    _algorithms_widget.setLayout(new QVBoxLayout(&_algorithms_widget));
    add_shape(shape);
    rotate_current_shape();
}

Visualizer3D* SliceApp::visualize() {
    _visualizer = new Visualizer3D(this, _debug_prints);
    QObject::connect(this, &SliceAppBase::request_redraw, _visualizer, &Visualizer3D::draw);
    emit request_redraw();

    //create the control widget
    auto layout = new QVBoxLayout(&_control_widget);
    layout->setSizeConstraint(QLayout::SetFixedSize);
    _control_widget.setLayout(layout);

    QPushButton* button_cut = new QPushButton("Cut!");
    QObject::connect(button_cut, &QPushButton::clicked, this, &SliceApp::manualCut);
    layout->addWidget(button_cut);

    QPushButton* button_revert = new QPushButton("Revert?");
    QObject::connect(button_revert, &QPushButton::clicked, this, &SliceApp::revertCut);
    layout->addWidget(button_revert);

    QPushButton* button_next = new QPushButton("Next shape");
    QObject::connect(button_next, &QPushButton::clicked, this, &SliceApp::next_shape);
    layout->addWidget(button_next);

    QPushButton* button_seperator = new QPushButton("T-Separator");
    QObject::connect(button_seperator, &QPushButton::clicked, this, &SliceApp::start_seperator);
    layout->addWidget(button_seperator);

    QPushButton* button_externalbase = new QPushButton("ExternalBase");
    QObject::connect(button_externalbase , &QPushButton::clicked, this, &SliceApp::start_external_base);
    layout->addWidget(button_externalbase);

    QPushButton* button_stochastip = new QPushButton("StochasTip");
    QObject::connect(button_stochastip , &QPushButton::clicked, this, &SliceApp::start_stochastip);
    layout->addWidget(button_stochastip);

    QPushButton* button_randomcut = new QPushButton("Random Cut");
    QObject::connect(button_randomcut, &QPushButton::clicked, this, &SliceApp::start_randomcut);
    layout->addWidget(button_randomcut);

    QPushButton* button_step = new QPushButton("Step");
    QObject::connect(button_step, &QPushButton::clicked, this, &SliceApp::do_step);
    layout->addWidget(button_step);

    QPushButton* button_merge = new QPushButton("Merge!");
    QObject::connect(button_merge, &QPushButton::clicked, this, &SliceApp::merge_final);
    layout->addWidget(button_merge);

    _backtrack_button = new QPushButton("Automatic search");
    _backtrack_button->setCheckable(true);
    QObject::connect(_backtrack_button, &QPushButton::toggled, this, &SliceApp::random_backtrack_runner);
    layout->addWidget(_backtrack_button);

    QPushButton* button_randalg = new QPushButton("Random Alg");
    QObject::connect(button_randalg, &QPushButton::clicked, this, &SliceApp::random_algorithm);
    layout->addWidget(button_randalg);

    QPushButton* button_export = new QPushButton("Export Result");
    QObject::connect(button_export, &QPushButton::clicked, this, &SliceApp::export_result);
    layout->addWidget(button_export);

    QPushButton* button_screenshot = new QPushButton("Screenshot");
    QObject::connect(button_screenshot, &QPushButton::clicked, this, &SliceApp::screenshot);
    layout->addWidget(button_screenshot);

    QPushButton* button_exit = new QPushButton("Exit");
    QObject::connect(button_exit, &QPushButton::clicked, this, &SliceApp::exit);
    layout->addWidget(button_exit);

    layout->addWidget(&_algorithms_widget);

    _control_widget.show();
    _control_widget.resize(200, 200);
    return _visualizer;
}

void SliceApp::add_shape(Polyhedron* shape) {
    if (shape->printable()) {
        _good_shapes.push_back(shape);
    } else {
        _bad_shapes.push_back(shape);
    }
    emit request_redraw();
}

void SliceApp::manualCut() {
    if (!_selected_algorithm) {
        if (_algorithms.empty())
            return;
        else
            select_algorithm(_algorithms.begin()->first);
    }
    if (_selected_algorithm->get_widget())
        _selected_algorithm->get_widget()->deselect();
    push_state(_selected_algorithm->cut(), _selected_algorithm->name());
}

void SliceApp::select_algorithm(int algorithm_idx) {
    _selected_algorithm = _algorithms[algorithm_idx];
    for (auto id_and_algorithm : _algorithms) {
        CutAlgorithm* alg = id_and_algorithm.second;
        if (alg != _selected_algorithm) {
            if (alg->get_widget()) {
                alg->get_widget()->deselect();
            }
        }
    }
}

void SliceApp::delete_algorithm(int algorithm_idx) {
    auto algorithm = _algorithms[algorithm_idx];
    if (algorithm == _selected_algorithm)
        _selected_algorithm = nullptr;
    _algorithms.erase(algorithm_idx);
    algorithm->quit();
    delete algorithm;
    redraw();
}


void SliceApp::start_seperator() {
    auto alg = new ContractionAlgorithm(this, _current_shape,
                                     _current_shape->face_clamp(_visualizer->get_selected_point()));
    add_algorithm(alg, true);
    _visualizer->set_selected_point(alg->_reference_point);
}

void SliceApp::start_external_base() {
    _current_shape->recognize_bases();
    auto face = _current_shape->face_clamp(_visualizer->get_selected_point());
    auto base_idx = _current_shape->_base_map[face];
    if (base_idx == - 1) {
        cout << "No base at this face" << endl;
        return;
    }
    auto base = _current_shape->_bases[base_idx];
    add_algorithm(new ExternalBaseAlgorithm(this, _current_shape, base, face), true);
}


void SliceApp::start_stochastip() {
    auto alg = new StochasTip(this, _current_shape,  _current_shape->face_clamp(_visualizer->get_selected_point()));
    add_algorithm(alg, true);
    _visualizer->set_selected_point(alg->_reference_point);
}

void SliceApp::start_randomcut() {
    init_factories(_algorithm_factories);
    auto alg = _algorithm_factories[3]->pick();
    add_algorithm(alg, true);
}

void SliceApp::add_algorithm(CutAlgorithm* alg, bool add_widget) {
    QObject::connect(alg, &CutAlgorithm::progress_updated, this, &SliceApp::redraw);
    QObject::connect(alg, &CutAlgorithm::request_redraw, this, &SliceApp::redraw);
    alg->set_idx(_next_algorithm);
    _algorithms[_next_algorithm] = alg;
    _next_algorithm++;
    if (add_widget) {
        auto widget = alg->get_widget(true);
        _algorithms_widget.layout()->addWidget((QWidget*)widget);
        QObject::connect(widget, &CutAlgorithmWidget::selected, this, &SliceApp::select_algorithm);
        QObject::connect(widget, &CutAlgorithmWidget::delete_requested, this, &SliceApp::delete_algorithm);
    }
    alg->run();
    if (_selected_algorithm == nullptr)
        _selected_algorithm = alg;
}

struct combined {
    set<int> shapes;
    uint base;
};

int SliceApp::optimistic_partcount(vector<Polyhedron*> shapes, bool do_merge) {
    ulong size = shapes.size();
    vector<combined> result;
    vector<set<Face>> deleted_faces;
    for (uint i = 0; i < size; i++) {
        combined c; c.base = i;
        c.shapes.insert(i);
        result.push_back(c);
        deleted_faces.push_back(set<Face>());
    }
    vector<vector<set<Face>>> overlaps;
    for (uint added = 0; added < shapes.size(); added++) {
        overlaps.emplace_back();
        for (uint base = 0; base < shapes.size(); base++) {
            overlaps[added].emplace_back();
            if (base == added)
                continue;
            shapes[added]->get_overlap(shapes[base], overlaps[added][base]);
        }
    }
    bool changed = true;
    while (changed) {
        changed = false;
        for (uint base = 0; base < result.size(); base++) {
            for (uint added = 0; added < result.size(); added++) {
                if (result[base].base != base or result[added].base != added or base == added) //any merged, or same
                    continue;
                auto added_printable = true;
                vector<set<Face>> overlap;
                int t = 0;
                int sum_overlap = 0;
                for (auto added_shape_idx : result[added].shapes) {
                    auto added_shape = shapes[added_shape_idx];
                    overlap.push_back(set<Face>());
                    for (auto base_shape_idx : result[base].shapes) {
                        for (auto face : overlaps[added_shape_idx][base_shape_idx])
                            overlap[t].insert(face);
                    }
                    sum_overlap += overlap[t].size();
                    for (auto deleted : deleted_faces[added_shape_idx])
                        overlap[t].insert(deleted);
                    if (not added_shape->printable(shapes[result[base].base]->suggested_base, overlap[t])) {
                        added_printable = false;
                        break;
                    }
                    t++;
                }
                if (sum_overlap > 5 and added_printable) {
                    //cout << added <<  " merged to " << base << endl;
                    //cout << "new base" << shapes[result[base].base]->suggested_base << endl;
                    changed = true;
                    t = 0;
                    //delete added side
                    for (auto added_shape_idx : result[added].shapes) {
                        for (auto overlap_face : overlap[t])
                            deleted_faces[added_shape_idx].insert(overlap_face);
                        t++;
                    }
                    //delete second side
                    for (auto base_shape_idx : result[base].shapes) {
                        for (auto added_shape_idx : result[added].shapes) {
                            for (auto f : overlaps[base_shape_idx][added_shape_idx])
                                deleted_faces[base_shape_idx].insert(f);
                        }
                    }
                    //mark merged
                    for (auto s : result[added].shapes) {
                        result[base].shapes.insert(s);
                    }
                    result[added].base = base;
                }
            }
        }
    }
    auto res_size = 0;
    for (uint i = 0; i < result.size(); i++) {
        if (result[i].base == i) {
            res_size++;
        } else if (do_merge) {
            auto base = result[i].base;
            while (result[base].base != base) {
                base = result[base].base;
            }
            shapes[i]->merge_base = shapes[base];
            shapes[base]->merge_children.push_back(shapes[i]);
        }
    }
    return res_size;
}

void SliceApp::merge_polys(vector<Polyhedron*>& polys) {
    ulong size = polys.size();
    vector<bool> merged; merged.assign(size, false);

    for (uint i = 0; i < polys.size(); i++) {
        Polyhedron * cur = polys[i];
        if (cur == nullptr)
            continue;
        for (uint j = i+1; j < polys.size(); j++) {
            Polyhedron* tested = polys[j];
            if (j == i or tested == nullptr)
                continue;
            Polyhedron* temp = cur->merge(tested, true);
            if (temp == nullptr)
                continue;
            cout << "merged " << j << "to " << i << " : " << (void*)temp << endl;
            delete cur;
            cur = temp;
            polys[j] = nullptr;
            j = 0;
        }
        polys[i] = cur;
    }

    vector<Polyhedron*> result;
    for (uint i = 0; i < polys.size(); i++) {
        if (polys[i] != nullptr)
            result.push_back(polys[i]);
    }
    polys = result;
}

void SliceApp::random_backtrack_runner(bool checked) {
    if (checked) {
        _run_search = true;
        auto future = QtConcurrent::run(this, &SliceApp::random_backtrack_wrapped);
    } else {
        _run_search = false;
    }
}

bool SliceApp::check_limits() {
    if (not _run_search)
        return false;
    auto time_test = (time(nullptr) - _start_time) / 60.0;
    if (_time_limit != -1 and time_test > _time_limit)
        return false;
    if (_cut_limit != -1 and (int)(_cuts_done) > _cut_limit)
        return false;
    return true;
}

void SliceApp::random_backtrack_wrapped() {
    try {
        cout << "Started Random backtrack.." << endl;
        _cuts_done = 0;
        _start_time = time(nullptr);
        _run_search = true;
        random_backtrack();
        if (not _best_history.empty()) {
            shapes_mutex.lock();
            _history = _best_history;
            _good_shapes = _best_result;
            _current_shape = nullptr;
            _best_history.clear();
            _best_result.clear();
            merge_final();
            shapes_mutex.unlock();
            if (_backtrack_button != nullptr)
                _backtrack_button->setChecked(false);
            cout << "Total Cuts done: " << _cuts_done << endl;
            cout << "Total time: " << time(nullptr) - _start_time << " seconds" << endl;
            cout << "Methods in best cut: ";
            bool first = true;
            for (auto h : _history) {
                if (not first)
                    cout << ", ";
                cout << h.method;
                first = false;
            }
            cout << endl;
            emit request_redraw();
        } else {
            cout << "NO RESULT on given parameters" << endl;
        }
    } catch (exception& e) {
        cout << "Fail:" << e.what() << endl;
    }
}

void SliceApp::save_best_result() {
    //cleanup
    while (not _best_history.empty()) {
        auto last = _best_history.back(); _best_history.pop_back();
        for (auto res : last.cut_results)
            delete res;
        if (_best_history.empty())
            delete last.original;
    }
    _best_result.clear();
    //save full state
    map<Polyhedron*,Polyhedron*> sidemap;
    for (auto history : _history) {
        HistoricState side_state;
        if (sidemap.find(history.original) == sidemap.end())
            sidemap[history.original] = history.original->copy();
        side_state.original = sidemap[history.original];
        side_state.method = history.method;
        for (auto cut_result : history.cut_results) {
            if (sidemap.find(cut_result) == sidemap.end())
                sidemap[cut_result] = cut_result->copy();
            side_state.cut_results.insert(sidemap[cut_result]);
        }
        _best_history.push_back(side_state);
    }
    for (auto s : _good_shapes) {
        _best_result.push_back(sidemap[s]);
    }
}

void SliceApp::init_factories(std::vector<AlgorithmFactory*>& factories) {
    if (not factories.empty())
        return;
    factories.push_back(new StochasTipFactory(this, _current_shape));
    factories.push_back(new ExternalBaseFactory(this, _current_shape));
    factories.push_back(new ContractionFactory(this, _current_shape));
    factories.push_back(new RandomCutFactory(this, _current_shape));
}

CutAlgorithm* SliceApp::choose_algorithm(std::vector<AlgorithmFactory*>& factories) {
    vector<double> dists = _algorithm_distribution;
    CutAlgorithm* cut_algorithm = nullptr;
    while (cut_algorithm == nullptr) {
        double sum = 0;
        for (auto d : dists)
            sum += d;
        if (sum == 0)
            break;
        double roll = _distribution(_generator) * sum;
        uint alg = 0;
        while (roll > dists[alg]) {
            roll -= dists[alg];
            alg++;
        }
        if (alg < factories.size())
            cut_algorithm = factories[alg]->pick();
        if (cut_algorithm != nullptr)
            break;
        dists[alg] = 0; //reset this dist, find other
    }
    if (_debug_prints and cut_algorithm != nullptr) {
        cout << "choose " << cut_algorithm->name() << endl;
    }
    return cut_algorithm;
}

void SliceApp::clear_factories(std::vector<AlgorithmFactory*>& factories) {
    for (auto factory : factories)
        delete factory;
    factories.clear();
}

void SliceApp::random_backtrack(uint depth) {
    if (_debug_prints)
        cout << "entering depth" << depth << " with " << num_shapes() << " shapes" << endl;
    std::vector<AlgorithmFactory* > factories;
    init_factories(factories);
    while (check_limits()) {
        CutAlgorithm* cut_algorithm = choose_algorithm(factories);
        if (cut_algorithm == nullptr)
            break;
        if (_visualizer)
            _visualizer->set_selected_point(cut_algorithm->_reference_point);
        emit request_redraw();
        cut_algorithm->run();
        cut_algorithm->join();
        if (not cut_algorithm->get_result()) {
            delete cut_algorithm;
            continue;
        }
        try {
            push_state(cut_algorithm->cut(), cut_algorithm->name());
            _cuts_done++;
            if (_current_shape == nullptr) { //done
                vector<Polyhedron*> temp;
                for (auto s : _good_shapes)
                    temp.push_back(s);
                auto size = (uint)optimistic_partcount(temp); //tries some merges
                //auto size = _good_shapes.size();
                if (size < _best_size or _good_shapes.size() < _best_size) {
                    _best_size = (uint)size;
                    _max_depth = depth + 1;
                    save_best_result();
                    cout << "new best " << _best_size
                         << " with " << (depth+1) << " cuts"
                         << " after " << this->_cuts_done << " attempted cuts"
                         << " and " << time(nullptr) - _start_time << " seconds" << endl;
                }
            }
            else if (depth < _max_depth) { //recursion possibly useful
                random_backtrack(depth + 1);
            }
            //not recursing - can revert
            revertCut();
            if (_debug_prints)
                cout << "back to depth" << depth << " with " << num_shapes() << " shapes" << endl;
        } catch (exception& e) {
            if (_debug_prints) {
                cout << "Failed Applying cut in backtrack: " << e.what() << endl
                     << "Choosing another algorithm..." << endl;
                //delete cut_algorithm;
                //continue;
            }
        }
        delete cut_algorithm;
        if (_best_result.size() == 0 and depth == 0)
            continue; //always retry
        //trying another one, or backtracking?
        auto alpha = _distribution(_generator);
        auto criterion = _backtrack_lin * pow(_backtrack_expbase, depth);
        if (alpha > criterion) {
            if (_debug_prints)
                cout << "backtracking, prob was " << criterion << endl;
            clear_factories(factories);
            return; //not trying another one
        }
    }
}

void SliceApp::random_algorithm() {
    if (_algorithm_factories.empty())
        init_factories(_algorithm_factories);
    auto alg = choose_algorithm(_algorithm_factories);
    if (alg != nullptr) {
        add_algorithm(alg, true);
        if (_visualizer)
            _visualizer->set_selected_point(alg->_reference_point);
    }
    else
        cout << "No more of that type" << endl;

}

void SliceApp::revertCut() {
    if (_history.size() == 0)
        return;
    shapes_mutex.lock();
    for (auto id_and_algorithm : _algorithms) {
        auto algorithm = id_and_algorithm.second;
        algorithm->quit();
        delete algorithm;
    }
    HistoricState previous = _history.back(); _history.pop_back();
    if (_current_shape != nullptr)
        _bad_shapes.push_back(_current_shape);
    _current_shape = nullptr;
    for (auto shape : previous.cut_results) {
        _good_shapes.remove(shape);
        _bad_shapes.remove(shape);
        delete shape;
    }
    _bad_shapes.push_back(previous.original);
    rotate_current_shape();
    shapes_mutex.unlock();
}

void SliceApp::push_state(set<Polyhedron*> children, string method) {
    shapes_mutex.lock();
    for (auto id_and_algorithm : _algorithms) {
        auto algorithm = id_and_algorithm.second;
        algorithm->quit();
        delete algorithm;
    }
    clear_factories(_algorithm_factories);
    HistoricState state;
    state.original = _current_shape; state.cut_results = children; state.method = method;
    _algorithms.clear();
    _current_shape = NULL;
    _history.push_back(state);
    _algorithms.clear();
    _selected_algorithm = nullptr;

    for (auto shape : children) {
        add_shape(shape);
    }
    rotate_current_shape();
    shapes_mutex.unlock();
}

void SliceApp::reload_algorithm_widgets() {
    for (auto id_and_algorithm : _algorithms) {
        CutAlgorithm* algorithm =  id_and_algorithm.second;
        auto widget = algorithm->get_widget();
        if (widget != nullptr and widget->parent() == nullptr)
            _algorithms_widget.layout()->addWidget((QWidget*)widget);
    }
    _algorithms_widget.update();
}

void SliceApp::rotate_current_shape() {
    if (_current_shape) {
        _bad_shapes.push_back(_current_shape);
        _current_shape = nullptr;
    }
    _algorithms.clear();
    _selected_algorithm = nullptr;
    if (_bad_shapes.size() != 0) {
        _current_shape = _bad_shapes.back();
        _bad_shapes.pop_back();
    }
    redraw();
}

void SliceApp::redraw() {
    emit request_redraw();
    reload_algorithm_widgets();
}

Polyhedron* SliceApp::get_current_shape() {
    return _current_shape;
}

typename SliceApp::ShapeList SliceApp::get_good_shapes() {
    return _good_shapes;
}

typename SliceApp::ShapeList SliceApp::get_bad_shapes() {
    return _bad_shapes;
}

typename SliceApp::AlgorithmList SliceApp::get_algorithms() {
    return _algorithms;
}

void SliceApp::do_step() {
    if (not _selected_algorithm)
        return;
    _selected_algorithm->_do_step = true;
}

void SliceApp::merge_final() {
    vector<Polyhedron*> res(_good_shapes.begin(), _good_shapes.end());
    cout << "Shapes before merge: " << _good_shapes.size() << endl;
    auto new_parts = optimistic_partcount(res, true);
    cout << "Optimistic parts:" << new_parts << endl;
//    merge_polys(res);
//    _good_shapes.clear();
//    for (auto shape : res) {
//        _good_shapes.push_back(shape);
//    }
    //cout << "Shapes after merge: " << _good_shapes.size() << endl;
    emit request_redraw();
}

void SliceApp::next_shape() {
    if (_current_shape) {
        _bad_shapes.push_front(_current_shape);
        _current_shape = nullptr;
    }
    rotate_current_shape();
}

void SliceApp::exit() {
    delete _current_shape;
    for (auto shape : _good_shapes)
        delete shape;
    for (auto shape : _bad_shapes)
        delete shape;
    for (auto history : _history) {
        delete history.original;
    }
    delete _visualizer;
    QApplication::quit();
}

uint SliceApp::num_shapes() {
    return (uint)(_good_shapes.size() + _bad_shapes.size() + 1);
}

void SliceApp::export_result() {
    auto last_dot = _filename.rfind('.');
    std::string without_suffix = _filename.substr(0, last_dot);
    uint i = 1;
    for (auto good : _good_shapes) {
        good->write_mesh(without_suffix + "_" + std::to_string(i) + ".stl");
        i++;
    }
    if (_current_shape != nullptr) {
        _current_shape->suggested_base = Plane3(Point3(0,0,0),Vector3(0,0,-1));
        _current_shape->write_mesh(without_suffix + "_" + std::to_string(i) + ".stl");
    }
    cout << "export successful!" << endl;
}

void SliceApp::screenshot() {
    auto last_dot = _filename.rfind('.');
    std::string without_suffix = _filename.substr(0, last_dot);
    auto fname = without_suffix + "_" + std::to_string(_screenid++) + ".png";
    _visualizer->screenshot(fname);
}