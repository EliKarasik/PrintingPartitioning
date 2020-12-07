#include <QtWidgets/QHBoxLayout>
#include "HoughVisualization.h"
#include "QHeatMap.h"

#include "3D/Polyhedron.h"

HoughVisualization::HoughVisualization(HoughSpace* source) : _source(source),
                                                                _heatmap(_source, QSize(TWO_DIST, ANGLES)) {
    auto grid = new QGridLayout();
    setLayout(grid);
    grid->addWidget(&_heatmap, 0, 0);
    grid->addWidget(&_slider, 0, 1);
    grid->addWidget(&_label, 1, 0);
    _slider.setMaximum(ANGLES);
    _slider.setTickInterval(1);
    _slider.setFocusPolicy(Qt::NoFocus);
    QObject::connect(&_slider, &QSlider::valueChanged, this, &HoughVisualization::beta_changed);
}

void HoughVisualization::redraw() {
    _heatmap.redraw();
    auto selected = _source->get_selected();
    _slider.setValue(selected.orientation.beta);
    stringstream s; s << "Alpha:" << selected.orientation.alpha << ", Beta:" << selected.orientation.beta
                                                             << ", Distance:" << selected.distance;
    _label.setText(QString::fromStdString(s.str()));
}

void HoughVisualization::keyPressEvent(QKeyEvent *e) {
    HoughCoords selected = _source->get_selected();
    bool changed = false;
    switch (e->key()) {
        case Qt::Key_W:
            selected.orientation.alpha = max(selected.orientation.alpha - 1, 0);
            changed = true;
            break;
        case Qt::Key_S:
            selected.orientation.alpha = min(selected.orientation.alpha + 1, ANGLES-1);
            changed = true;
            break;
        case Qt::Key_Q:
            selected.orientation.beta = max(selected.orientation.beta - 1, 0);
            changed = true;
            break;
        case Qt::Key_E:
            selected.orientation.beta = min(selected.orientation.beta + 1, ANGLES-1);
            changed = true;
            break;
        case Qt::Key_D:
            selected.distance = min((int)round(selected.distance) + 1, MAX_DIST);
            changed = true;
            break;
        case Qt::Key_A:
            selected.distance = max((int)round(selected.distance) - 1, -MAX_DIST);
            changed = true;
            break;
        default:
            break;
    }
    if (changed) {
        _source->set_selected(selected);
        cout << _source->get_value(selected) << endl;
        redraw();
    }
}

void HoughVisualization::beta_changed(int value) {
    HoughCoords selected = _source->get_selected();
    selected.orientation.beta = value;
    _source->set_selected(selected);
    redraw();
}
