#ifndef SLICE_QHEATMAP_H
#define SLICE_QHEATMAP_H


#include <QtGui/QMouseEvent>
#include <QtWidgets/QLabel>

class HoughSpace;



class QHeatMapBase : public QLabel {
    Q_OBJECT

protected:
    void mousePressEvent(QMouseEvent* e)=0;
};

class QHeatMap : public QHeatMapBase {
public:
    QHeatMap(HoughSpace* source, QSize size);
    void redraw();

    void mousePressEvent(QMouseEvent* e) override;
private:
    double _scale;
    QSize _size;
    QImage* _image;
    HoughSpace* _source;
};


#endif //SLICE_QHEATMAP_H
