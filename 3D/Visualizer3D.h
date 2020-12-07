#ifndef SLICE_VISUALIZER_H
#define SLICE_VISUALIZER_H

#include <Qt3DExtras/qt3dwindow.h>
#include <Qt3DCore/QTransform>
#include <Qt3DRender/QPickEvent>
#include <QtWidgets/QWidget>
#include <boost/bimap.hpp>
#include <Qt3DRender/QRenderCapture>
#include "../SliceApp.h"
#include "Polyhedron.h"
#include "PolyhedronVisualizer.h"

typedef Qt3DCore::QEntity QEntity;
typedef Qt3DCore::QNode QNode;
typedef Qt3DCore::QTransform Q3DTransform;

class Visualizer3D : public Qt3DExtras::Qt3DWindow {
Q_OBJECT
public:
    Visualizer3D(SliceApp *app, bool debug);

    QEntity *add_lines(QEntity *parent, vector<Segment3> lines);

    QEntity *root_entity();

    Point3 get_selected_point();

    void set_selected_point(Point3 point);

    void screenshot(string filename);

    virtual ~Visualizer3D();

public slots:

    void draw();

    void point_picked_global(Qt3DRender::QPickEvent *event);

    void point_picked(PolyhedronVisualizer *poly, Point3 local, Point3 global);

    void update_light();

    void capture_complete();

private:

    QEntity *_root;
    Q3DTransform *_light_transform;
    SliceApp *_app;
    bool _debug;
    Qt3DCore::QTransform *_selected_point_transform;
    QEntity *_selected_point_entity;
    bool _selected_point_enabled;
    map<int, PolyhedronVisualizer *> _poly_to_visualizer;
    typedef boost::bimap<CutDescription *, QEntity *> CutsBimap;
    CutsBimap _cuts;
    set<QNode *> _in_use;
    Point3 _selected_point, _selected_point_global;
    PolyhedronVisualizer *_selected_visualizer;
    bool _hide_others;
    QEntity *_print_plane;
    Qt3DRender::QRenderCapture *_capture;
    Qt3DRender::QRenderCaptureReply *_capture_reply;
    string _capture_filename;

    void setup_camera();

    PolyhedronVisualizer *add_polyhedron(Polyhedron *poly);

    void add_selected_point();

    void keyPressEvent(QKeyEvent *e) override;
};

QVector3D from_vector(Vector3 p);

#endif //SLICE_VISUALIZER_H
