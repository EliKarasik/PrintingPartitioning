#ifndef SLICE_CUTALGORITHMWIDGET_H
#define SLICE_CUTALGORITHMWIDGET_H

#include <QProgressBar>
#include <QGridLayout>
#include <QLabel>
#include <QToolButton>
#include <QPushButton>
#include <QMouseEvent>

class CutAlgorithm;

class CutAlgorithmWidgetBase : public QFrame {
Q_OBJECT
public slots:

    virtual void update_progress(double progress) = 0;

    virtual void delete_pressed() = 0;

signals:

    void selected(int algorithm_idx);

    void delete_requested(int algorithm_idx);

protected:
    QProgressBar _progress;
};

class CutAlgorithmWidget : public CutAlgorithmWidgetBase {
public:
    explicit CutAlgorithmWidget(CutAlgorithm *parent);

    void deselect();

    void update_progress(double progress) override;

    void delete_pressed() override;

protected:
    void mousePressEvent(QMouseEvent *me) override;

private:
    CutAlgorithm *_parent;

    QLabel _label;
    QToolButton _cancel;
    QGridLayout _layout;
    bool _is_selected;
};


#endif //SLICE_CUTALGORITHMWIDGET_H
