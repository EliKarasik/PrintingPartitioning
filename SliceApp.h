#ifndef SLICE_SLICEAPP_H
#define SLICE_SLICEAPP_H

#include <QtCore/QObject>
#include <QtWidgets/QGraphicsScene>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QPushButton>
#include <random>
#include <QtCore/QMutex>
#include <time.h>
#include "defs.h"
#include "3D/Polyhedron.h"
#include "Algorithms/CutAlgorithm.h"

class SliceAppBase : public QObject {
    Q_OBJECT
signals:
    void request_redraw();
};

class SliceApp : public SliceAppBase {
    typedef list<Polyhedron*> ShapeList;
    typedef map<int, CutAlgorithm*> AlgorithmList;

    struct HistoricState {
        Polyhedron* original;
        set<Polyhedron*> cut_results;
        string method;
    };

    QApplication* _app;

    deque<HistoricState> _history;
    Polyhedron* _current_shape;
    ShapeList _good_shapes, _bad_shapes;

    QWidget _control_widget;
    QWidget _algorithms_widget;

    QPushButton* _backtrack_button;

    AlgorithmList _algorithms;
    int _next_algorithm;
    CutAlgorithm* _selected_algorithm;

    Visualizer3D* _visualizer;

    atomic<bool> _run_search;
    uint _best_size;
    uint _max_depth;
    list<Polyhedron*> _best_result;
    uint _cuts_done;
    time_t _start_time;
    deque<HistoricState> _best_history;
    std::uniform_real_distribution<double> _distribution;
    std::minstd_rand _generator;

    std::vector<AlgorithmFactory*> _algorithm_factories;
    std::vector<double> _algorithm_distribution;
    double _backtrack_lin, _backtrack_expbase;
    bool _debug_prints;
    int _time_limit;
    int _cut_limit;
    std::string _filename;
    int _screenid;

    void add_shape(Polyhedron* shape);
    void init_factories(std::vector<AlgorithmFactory*>& factories);
    CutAlgorithm* choose_algorithm(std::vector<AlgorithmFactory*>& factories);
    void clear_factories(std::vector<AlgorithmFactory*>& factories);
    bool check_limits();

public:
    SliceApp(QApplication* app, Polyhedron* shape, std::string filename, std::vector<double>& algorithm_distribution,
             double backtrack_lin, double backtrack_exp, bool debug_prints, int timelimit, int cut_limit);
    void redraw();
    Polyhedron* get_current_shape();
    ShapeList get_good_shapes();
    ShapeList get_bad_shapes();
    uint num_shapes();
    AlgorithmList get_algorithms();

    void push_state(set<Polyhedron*> children, string method);

    void reload_algorithm_widgets();
    void add_algorithm(CutAlgorithm* alg, bool add_widget);
    void select_algorithm(int algorithm_idx);
    void delete_algorithm(int algorithm_idx);

    void rotate_current_shape();
    void manualCut();
    void revertCut();
    void next_shape();
    void start_tip_algorithm();
    void start_seperator();
    void start_external_base();
    void start_stochastip();
    void start_split();
    void start_randomcut();
    void merge_final();
    void do_step();
    void random_algorithm();
    void export_result();
    void screenshot();
    void exit();

    void random_backtrack(uint depth=0);
    void random_backtrack_wrapped();
    void random_backtrack_runner(bool checked);
    void merge_polys(vector<Polyhedron*>& polys);
    int optimistic_partcount(vector<Polyhedron*> shapes, bool do_merge=false);
    void save_best_result();

    Visualizer3D* visualize();

    QMutex shapes_mutex;
};


#endif //SLICE_SLICEAPP_H
