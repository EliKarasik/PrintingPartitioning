#include "CutAlgorithmWidget.h"
#include "CutAlgorithm.h"


CutAlgorithmWidget::CutAlgorithmWidget(CutAlgorithm *parent) : _parent(parent), _is_selected(false) {
    this->setFrameShape(QFrame::Panel);
    _cancel.setText("X");
    _progress.setValue(0);
    _progress.setStyleSheet("::chunk{background-color:" + _parent->get_color().name() + ";}");
    _progress.setMaximum(100);
    _layout.addWidget(&_progress, 0, 0);
    _layout.addWidget(&_label, 0, 0);
    _layout.addWidget(&_cancel, 0, 1);
    setLayout(&_layout);

    QObject::connect(parent, &CutAlgorithm::progress_updated, this, &CutAlgorithmWidget::update_progress);
    QObject::connect(&_cancel, &QToolButton::pressed, this, &CutAlgorithmWidget::delete_pressed);
}


void CutAlgorithmWidget::delete_pressed() {
    emit delete_requested(_parent->get_idx());
}


void CutAlgorithmWidget::deselect() {
    _is_selected = false;
    setStyleSheet("");
    _parent->close_visualize();
}


void CutAlgorithmWidget::mousePressEvent(QMouseEvent *me) {
    if (me->button() == Qt::LeftButton) {
        if (_is_selected)
            return;
        _is_selected = true;
        _parent->visualize();
        setStyleSheet("QFrame{color:red;}");
        emit selected(_parent->get_idx());
    }
}


void CutAlgorithmWidget::update_progress(double progress) {
    _progress.setValue((int) (progress * 100));
}