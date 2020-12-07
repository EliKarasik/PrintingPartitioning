#ifndef SLICE_CUTALGORITHM_H
#define SLICE_CUTALGORITHM_H

#include <QtConcurrent/QtConcurrent>
#include <random>
#include "../defs.h"
#include "../3D/Polyhedron.h"

class SliceApp;

class CutAlgorithmWidget;

struct CutScore {
    double angle;
    double pairs_score;
    int printable;

    CutScore();

    double to_double() const;

    bool operator<(const CutScore &rhs) const;
};

struct CutDescription {
    explicit CutDescription(shared_ptr<CutOption> cut);

    shared_ptr<CutOption> cut;
    CutScore score;
};

class CutAlgorithmBase : public QObject {
Q_OBJECT
signals:

    void progress_updated(double value);

    void request_redraw();
};

class CutAlgorithm : public CutAlgorithmBase {
public:
    CutAlgorithm(SliceApp *app, Polyhedron *shape, Face reference_face);

    CutAlgorithmWidget *get_widget(bool create = false);

    virtual CutDescription *get_result();

    virtual set<Polyhedron *> cut();

    K::FT get_score();

    virtual void preprocess() = 0;

    virtual void optimize() = 0;

    virtual void visualize();

    virtual void close_visualize();

    virtual void calculate();

    virtual void run();

    void quit();

    void join();

    QColor get_color();

    void set_idx(int idx);

    int get_idx();

    ~CutAlgorithm() override;

public:
    QFuture<void> _future;
    atomic<bool> _canceled;
    atomic<bool> _do_step;
    SliceApp *_app;
    Polyhedron *_shape, *_original_shape;
    Face _reference_face;
    Point _reference_point;
    Point _epsilon_reference_point;
    CutDescription *_result;
    K::FT _score;

    int _algorithm_idx;
    CutAlgorithmWidget *_widget;
    QColor _color;

    void clamp_point();

    virtual std::string name() = 0;
};

class AlgorithmFactory {
public:
    AlgorithmFactory(SliceApp *app, Polyhedron *shape);

    virtual CutAlgorithm *pick();

    virtual ~AlgorithmFactory();

protected:
    virtual CutAlgorithm *construct(int idx) = 0;

    void add_weight(double weight);

    void finalize();

    Polyhedron *_shape;
    SliceApp *_app;
    std::vector<pair<double, int>> _weights;
    uint _cur_idx;
    std::uniform_real_distribution<double> _distribution;
    std::minstd_rand _generator;
};

#endif //SLICE_CUTALGORITHM_H
