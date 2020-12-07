#ifndef SLICE_POLYHEDRONVISUALIZER_H
#define SLICE_POLYHEDRONVISUALIZER_H

#include <Qt3DRender/QObjectPicker>
#include <Qt3DExtras/QPhongAlphaMaterial>
#include <Qt3DExtras/QPerVertexColorMaterial>
#include <Qt3DRender/QGeometry>
#include <Qt3DRender/QBuffer>
#include <Qt3DCore/QTransform>
#include <Qt3DRender/QPickEvent>

#include "Polyhedron.h"

class Visualizer3D;

typedef Qt3DCore::QEntity QEntity;
typedef Mesh::Property_map <Face, QColor> FaceColoring;

class PolyhedronVisualizer : public QObject {
Q_OBJECT
public slots:

    void point_picked(Qt3DRender::QPickEvent *event);

public:
    PolyhedronVisualizer(Polyhedron *polyhedron, Visualizer3D *parent, QColor color, bool debug);

    void setActive(bool active);

    void setEnabled(bool enabled);

    void setAlphaColor(QColor color);

    void setSolidColor(QColor color);

    QColor getColor();

    Polyhedron *get_poly();

    void switchMaterial();

    void switchMaterial(bool alpha);

    void switchWireframe();

    void reload_vertex_buffer();

    void reload_junctions_buffer();

    void setFaceColor(Face face, QColor color);

    void rotate_to_base(double &next_x, double &min_y, double &max_y);

    void translate(QVector3D direction);

    QEntity *entity();

    ~PolyhedronVisualizer();

    Qt3DCore::QTransform *_transform;

private:
    void load_mesh();

    void load_wireframe();

    void load_picker();

    void load_skeleton();

    uint junction_points_drawed_count();

    void load_junctions();

    void load_tips();

    void add_point_and_color(float *buffer, Point3 point, QColor color);

    QEntity *load_lines_entity(Qt3DRender::QGeometry *geometry);

    QColor _color;
    Polyhedron *_poly;
    Visualizer3D *_parent;
    QEntity *_entity;
    Qt3DRender::QBuffer *_mesh_vertex_buffer, *_junctions_vertex_buffer, *_tips_vertex_buffer;
    Qt3DExtras::QPhongAlphaMaterial *_alpha_material;
    bool _alpha_used, _wireframe_enabled;
    Qt3DExtras::QPerVertexColorMaterial *_bad_face_material;
    QEntity *_wireframe_entity;
    QEntity *_skeleton_entity;
    QEntity *_junctions_entity;
    QEntity *_tips_entity;
    Qt3DRender::QAttribute *_junctions_position_attribute, *_junctions_color_attribute;
    Qt3DRender::QObjectPicker *_picker_entity;
    FaceColoring _coloring;
};


#endif //SLICE_POLYHEDRONVISUALIZER_H
