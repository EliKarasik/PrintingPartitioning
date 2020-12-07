#include "QHeatMap.h"
#include "HoughSpace.h"

QHeatMap::QHeatMap(HoughSpace* source, QSize size) : _source(source) {
    _scale = 5;
    _size = size;
    _image = new QImage(_size * _scale,  QImage::Format_RGB32);
    setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    setScaledContents(true);
    setMinimumSize((_image->size()));
    adjustSize();
}

void QHeatMap::redraw() {
    double max_val = 0.01;
    HoughCoords coords = _source->get_selected();
    for (int i = 0; i < _size.width(); i++) {
        for (int j = 0; j < _size.height(); j++) {
            coords.orientation.alpha = j;
            coords.distance = i - MAX_DIST;
            if (_source->get_value(coords) > max_val)
                max_val = _source->get_value(coords);
        }
    }

    _image->fill(QColor(127,127,127));
    QPainter painter(_image);
    for (int i = 0; i < _size.width(); i++) {
        for (int j = 0; j < _size.height(); j++) {
            coords.orientation.alpha = j;
            coords.distance = i - MAX_DIST;
            if (coords.distance == (int)_source->get_selected().distance and
                    j == _source->get_selected().orientation.alpha) {
                painter.setPen(QColor(0,255,0));
            } else {
                painter.setPen(Qt::NoPen);
            }
            double strength = max(_source->get_value(coords),0.0) / max_val;
            if (strength > 0.95) {
                painter.setBrush(QColor(0, 0, (int)strength*255));
            } else {
                painter.setBrush(QColor((int)strength*255, 0, 0));
            }
            if (_source->get_value(coords) != 0)
                painter.setBrush(QColor(0, 0, 255));
            if (_source->get_value(coords) == -1)
                painter.setBrush(QColor(255, 0, 0));
            if (coords.distance == 0)
                painter.setBrush(QColor(255, 255, 255));
            painter.drawRect(i*_scale,j*_scale,0.9*_scale,0.9*_scale);
        }
    }

    setPixmap(QPixmap::fromImage(*_image));
}

void QHeatMap::mousePressEvent(QMouseEvent *e) {
    QLabel::mousePressEvent(e);
    auto selected = _source->get_selected();
    int x = (e->x()) / ((double)(width()) / _size.width()) - MAX_DIST;
    int y = (e->y()) / ((double)(height()) / _size.height());
    auto coords = HoughCoords(Orientation(y,selected.orientation.beta), x);
    _source->set_selected(coords);
    cout << _source->get_value(coords) << endl;
    redraw();
}