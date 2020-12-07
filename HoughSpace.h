#ifndef SLICE_HOUGHSPACE_H
#define SLICE_HOUGHSPACE_H

#include <QtWidgets/QLabel>
#include "defs.h"
#include "3D/Polyhedron.h"

class HoughVisualization;

struct Orientation {
    Orientation(int alpha, int beta);
    int alpha;
    int beta;

    bool operator<(const Orientation& other) const;
};
struct HoughCoords {
    HoughCoords(Orientation orientation, double distance);
    HoughCoords(); //nullptr, -1
    Orientation orientation;
    double distance;


};

class HoughSpaceBase : public QObject {
Q_OBJECT
signals:
    void selection_changed();
};


class HoughSpace : public HoughSpaceBase {

protected:
    Point _middle;
    HoughVisualization* _visualizer;
    HoughCoords _selected;
    double* _vals;

    list<Orientation> _orientations;
    list<Orientation> _coarse_orientations;

    int coords_index(HoughCoords coords);

public:
    HoughSpace(Point middle);
    ~HoughSpace();

    void reset_middle(Point middle);
    list<Orientation>& orientations();
    list<Orientation>& coarse_orientations();
    void similar_orientations(Orientation initial, list<Orientation>& orientation_list, int scale);
    void set_selected(HoughCoords pos);
    HoughCoords get_selected();

    double get_value(HoughCoords coords);
    void set_value(HoughCoords coords, double value);
    InfinityFace inverse_hough(HoughCoords coords);
    HoughCoords approximate(InfinityFace infinity_face);
    void visualize();
    void close_visualize();
};


#endif //SLICE_HOUGHSPACE_H
