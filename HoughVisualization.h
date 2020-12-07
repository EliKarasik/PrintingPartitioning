#ifndef SLICE_HOUGHVISUALIZATION_H
#define SLICE_HOUGHVISUALIZATION_H

#include <QtWidgets/QWidget>
#include <QtWidgets/QSlider>
#include "HoughSpace.h"
#include "QHeatMap.h"


class HoughVisualization : public QWidget {

public:
    HoughVisualization(HoughSpace* source);
    void redraw();
    void beta_changed(int value);
protected:
    void keyPressEvent(QKeyEvent* e);
private:
    HoughSpace* _source;
    QHeatMap _heatmap;
    QSlider _slider;
    QLabel _label;
};


#endif //SLICE_HOUGHVISUALIZATION_H
