#include <Qt3DCore/QEntity>
#include <Qt3DCore/QComponent>
#include <Qt3DRender/QGeometry>
#include <Qt3DRender/QAttribute>
#include <Qt3DRender/QGeometryRenderer>
#include <Qt3DExtras/QPerVertexColorMaterial>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <Qt3DExtras/QSphereMesh>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DRender/QRenderStateSet>
#include <Qt3DRender/QPointSize>
#include <Qt3DRender/QShaderProgram>
#include <Qt3DRender/QRenderPass>
#include <Qt3DRender/QTechnique>
#include <Qt3DRender/QEffect>
#include <Qt3DRender/QParameter>
#include <Qt3DRender/QGraphicsApiFilter>
#include <QtCore/QUrl>
#include <Qt3DRender/QDepthTest>
#include <CGAL/Polygon_mesh_processing/bbox.h>
#include <QtGui/QQuaternion>
#include <Qt3DRender/QPickingSettings>
#include "PolyhedronVisualizer.h"
#include "Visualizer3D.h"


PolyhedronVisualizer::PolyhedronVisualizer(Polyhedron *polyhedron, Visualizer3D *parent, const QColor color, bool debug)
        : _color(color), _poly(polyhedron), _parent(parent), _alpha_used(true),
          _wireframe_enabled(false), _skeleton_entity(nullptr),
          _junctions_position_attribute(nullptr), _junctions_color_attribute(nullptr), _picker_entity(nullptr),
          _coloring(_poly->get_mesh()->add_property_map<Face, QColor>("f:Coloring").first) {
    for (auto face : _poly->get_mesh()->faces()) {
        setFaceColor(face, _color);
    }
    load_mesh();
    load_wireframe();
    if (debug) {
        load_skeleton();
        load_junctions();
        load_tips();
    }
    //load_picker();
}

QColor PolyhedronVisualizer::getColor() {
    return _color;
}

void PolyhedronVisualizer::reload_vertex_buffer() {
    auto mesh = _poly->get_mesh();
    QByteArray bufferArray;
    int chunk_size = 9;
    bufferArray.resize(chunk_size * 3 * mesh->num_faces() * sizeof(float));
    auto *vbufferContent = reinterpret_cast<float *>(bufferArray.data());
    uint idx = 0;
    map<Mesh::Vertex_index, int> indices;
    for (auto face : mesh->faces()) {
        QColor face_color;
        face_color = _coloring[face];
        auto normal = CGAL::Polygon_mesh_processing::compute_face_normal(face, *mesh);
        for (auto vertex : mesh->vertices_around_face(mesh->halfedge(face))) {
            auto point = mesh->point(vertex);
            vbufferContent[idx * chunk_size] = CGAL::to_double(point.x());
            vbufferContent[idx * chunk_size + 1] = CGAL::to_double(point.y());
            vbufferContent[idx * chunk_size + 2] = CGAL::to_double(point.z());
            vbufferContent[idx * chunk_size + 3] = CGAL::to_double(normal.x());
            vbufferContent[idx * chunk_size + 4] = CGAL::to_double(normal.y());
            vbufferContent[idx * chunk_size + 5] = CGAL::to_double(normal.z());
            vbufferContent[idx * chunk_size + 6] = face_color.red() / 255.0f;
            vbufferContent[idx * chunk_size + 7] = face_color.green() / 255.0f;
            vbufferContent[idx * chunk_size + 8] = face_color.blue() / 255.0f;
            idx++;
        }
    }
    _mesh_vertex_buffer->setData(bufferArray);
}


void PolyhedronVisualizer::load_mesh() {
    auto mesh = _poly->get_mesh();
    auto geometryRenderer = new Qt3DRender::QGeometryRenderer();
    geometryRenderer->setPrimitiveType(Qt3DRender::QGeometryRenderer::Triangles);
    auto meshGeometry = new Qt3DRender::QGeometry(geometryRenderer);
    _mesh_vertex_buffer = new Qt3DRender::QBuffer(meshGeometry);
    auto *indexBuffer = new Qt3DRender::QBuffer(meshGeometry);
    geometryRenderer->setGeometry(meshGeometry);
    int chunk_size = 9;
    reload_vertex_buffer();
    auto positionAttribute = new Qt3DRender::QAttribute(_mesh_vertex_buffer,
                                                        Qt3DRender::QAttribute::defaultPositionAttributeName(),
                                                        Qt3DRender::QAttribute::Float, 3,
                                                        mesh->num_vertices(),
                                                        0,
                                                        chunk_size * sizeof(float));
    meshGeometry->addAttribute(positionAttribute);
    auto normalAttribute = new Qt3DRender::QAttribute(_mesh_vertex_buffer,
                                                      Qt3DRender::QAttribute::defaultNormalAttributeName(),
                                                      Qt3DRender::QAttribute::Float, 3,
                                                      mesh->num_vertices(),
                                                      3 * sizeof(float),
                                                      chunk_size * sizeof(float));
    meshGeometry->addAttribute(normalAttribute);
    auto colorAttribute = new Qt3DRender::QAttribute(_mesh_vertex_buffer,
                                                     Qt3DRender::QAttribute::defaultColorAttributeName(),
                                                     Qt3DRender::QAttribute::Float, 3,
                                                     mesh->num_vertices(),
                                                     6 * sizeof(float),
                                                     chunk_size * sizeof(float));
    meshGeometry->addAttribute(colorAttribute);

    QByteArray ibufferArray;
    int vertices_in_face = 3;
    ibufferArray.resize(mesh->num_faces() * vertices_in_face * sizeof(quint32));
    auto *ibufferContent = reinterpret_cast<quint32 *>(ibufferArray.data());
    for (uint face_idx = 0; face_idx < mesh->num_faces(); ++face_idx) {
        uint vertex_idx = 0;
        //for (auto vertex : mesh->vertices_around_face(mesh->halfedge(face))) {
        //    ibufferContent[face_idx * vertices_in_face + vertex_idx] = (quint32)indices[vertex];
        //    vertex_idx++;
        //}
        for (vertex_idx = 0; vertex_idx < 3; vertex_idx++)
            ibufferContent[face_idx * vertices_in_face + vertex_idx] = face_idx * vertices_in_face + vertex_idx;
    }
    indexBuffer->setData(ibufferArray);
    auto indexAttribute = new Qt3DRender::QAttribute(indexBuffer,
                                                     Qt3DRender::QAttribute::UnsignedInt,
                                                     1,
                                                     mesh->num_faces() * vertices_in_face);
    indexAttribute->setAttributeType(Qt3DRender::QAttribute::IndexAttribute);
    meshGeometry->addAttribute(indexAttribute);

    _entity = new Qt3DCore::QEntity(_parent->root_entity());
    _entity->addComponent(geometryRenderer);
    //QPhongMaterial *material = new Qt3DExtras::QPhongMaterial();
    //material->setDiffuse(QColor(QRgb(0x928327)));
    //material->setShininess(100);

    _bad_face_material = new Qt3DExtras::QPerVertexColorMaterial();
    //_bad_face_material->setEnabled(false);
    //_entity->addComponent(_bad_face_material);

    _alpha_material = new Qt3DExtras::QPhongAlphaMaterial();
    //_alpha_material->setAmbient(color);
    _alpha_material->setAlpha(0.75);
    _alpha_material->setDiffuse(_color);
    _alpha_material->setSpecular(QColor(0, 0, 0));
    _entity->addComponent(_alpha_material);

    _transform = new Qt3DCore::QTransform();
    _entity->addComponent(_transform);
}

void PolyhedronVisualizer::setEnabled(bool enabled) {
    _entity->setEnabled(enabled);
    _wireframe_entity->setEnabled(_wireframe_enabled);
    if (not enabled) {
        setActive(false);
    }
}

void PolyhedronVisualizer::setActive(bool active) {
    if (_skeleton_entity)
        _skeleton_entity->setEnabled(active);
//    if (active) {
//        _entity->addComponent(_picker_entity);
//    } else {
//        _entity->removeComponent(_picker_entity);
//    }
}

void PolyhedronVisualizer::setSolidColor(QColor color) {
    _color = color;
    for (auto face : _poly->get_mesh()->faces()) {
        setFaceColor(face, _color);
    }
    reload_vertex_buffer();
}

void PolyhedronVisualizer::setAlphaColor(QColor color) {
    _alpha_material->setAmbient(color);
    _alpha_material->setDiffuse(color);
}

void PolyhedronVisualizer::switchMaterial() {
    switchMaterial(not _alpha_used);
}

void PolyhedronVisualizer::switchMaterial(bool alpha) {
    if (not(alpha ^ _alpha_used))
        return;
    if (_alpha_used) {
        _entity->addComponent(_bad_face_material);
        _entity->removeComponent(_alpha_material);
    } else {
        _entity->removeComponent(_bad_face_material);
        _entity->addComponent(_alpha_material);
    }
    _alpha_used = not _alpha_used;
}

QEntity *PolyhedronVisualizer::load_lines_entity(Qt3DRender::QGeometry *geometry) {
    //rendrer
    auto *geometryRenderer = new Qt3DRender::QGeometryRenderer();
    geometryRenderer->setPrimitiveType(Qt3DRender::QGeometryRenderer::Lines);
    geometryRenderer->setGeometry(geometry);

    //transform
    auto *point_transform = new Qt3DCore::QTransform();

    //entity
    auto entity = new QEntity(_entity);
    entity->addComponent(geometryRenderer);
    entity->addComponent(point_transform);
    return entity;
}

void PolyhedronVisualizer::load_wireframe() {
    Mesh *mesh = _poly->get_mesh();
    auto meshGeometry = new Qt3DRender::QGeometry();
    auto vertexBuffer = new Qt3DRender::QBuffer(meshGeometry);
    QByteArray bufferArray;
    int chunk_size = 6;
    uint num_vertices = mesh->num_edges() * 2;
    bufferArray.resize(chunk_size * num_vertices * sizeof(float));
    auto *vbufferContent = reinterpret_cast<float *>(bufferArray.data());
    map<Mesh::Vertex_index, int> indices;
    QColor color;
    for (uint idx = 0; idx < _poly->primitive_size(); idx += 1) {
        Segment3 segment = _poly->bad_primitive(idx);
        bool is_bad = _poly->is_bad(idx);
        color = is_bad ? QColor(255, 0, 0) : QColor(0, 0, 255);
        add_point_and_color(vbufferContent, segment.source(), color);
        vbufferContent += chunk_size;
        add_point_and_color(vbufferContent, segment.target(), color);
        vbufferContent += chunk_size;
    }
    vertexBuffer->setData(bufferArray);
    auto positionAttribute = new Qt3DRender::QAttribute(vertexBuffer,
                                                        Qt3DRender::QAttribute::defaultPositionAttributeName(),
                                                        Qt3DRender::QAttribute::Float, 3,
                                                        num_vertices,
                                                        0,
                                                        chunk_size * sizeof(float));
    meshGeometry->addAttribute(positionAttribute);
    auto colorAttribute = new Qt3DRender::QAttribute(vertexBuffer, Qt3DRender::QAttribute::defaultColorAttributeName(),
                                                     Qt3DRender::QAttribute::Float, 3,
                                                     num_vertices,
                                                     3 * sizeof(float),
                                                     chunk_size * sizeof(float));
    meshGeometry->addAttribute(colorAttribute);

    _wireframe_entity = load_lines_entity(meshGeometry);
    _wireframe_entity->setEnabled(false);
    _wireframe_entity->addComponent(new Qt3DExtras::QPerVertexColorMaterial());
}

void PolyhedronVisualizer::load_picker() {
    //_picker_entity = new Qt3DRender::QObjectPicker(_parent->root_entity());
    _picker_entity = new Qt3DRender::QObjectPicker(_entity);
    QObject::connect(_picker_entity, &Qt3DRender::QObjectPicker::clicked, this, &PolyhedronVisualizer::point_picked);
    //QObject::connect(_picker_entity, &Qt3DRender::QObjectPicker::clicked, nullptr, &printlol);
}

void PolyhedronVisualizer::point_picked(Qt3DRender::QPickEvent *event) {
    QVector3D qlocal = event->localIntersection();
    QVector3D qglobal = event->worldIntersection();
    Point3 local(qlocal.x(), qlocal.y(), qlocal.z());
    Point3 global(qglobal.x(), qglobal.y(), qglobal.z());
    _parent->point_picked(this, local, global);
}

Polyhedron *PolyhedronVisualizer::get_poly() {
    return _poly;
}


uint PolyhedronVisualizer::junction_points_drawed_count() {
    uint count = 0;
    for (auto junction : _poly->_junctions) {
        count += junction->small_cc.size() + 1;
    }
    return count;
}

void PolyhedronVisualizer::add_point_and_color(float *buffer, Point3 point, QColor color) {
    buffer[0] = CGAL::to_double(point.x());
    buffer[1] = CGAL::to_double(point.y());
    buffer[2] = CGAL::to_double(point.z());
    buffer[3] = color.red() / 255.0f;
    buffer[4] = color.green() / 255.0f;
    buffer[5] = color.blue() / 255.0f;
}

void PolyhedronVisualizer::reload_junctions_buffer() {
    QByteArray bufferArray;
    int chunk_size = 6;
    auto num_vertices = junction_points_drawed_count();
    bufferArray.resize((int) (chunk_size * num_vertices * sizeof(float)));
    auto *vbufferContent = reinterpret_cast<float *>(bufferArray.data());
    for (const auto &junction : _poly->_junctions) {
        for (auto point : junction->small_cc) {
            add_point_and_color(vbufferContent, point, QColor(0, 255, 0));
            vbufferContent += chunk_size;
        }
        //auto dist = min(_poly->_distances[_poly->voxel_idx(point)], 25);
        //QColor color = QColor(255 - 10 * dist, 10 * dist,0);
        add_point_and_color(vbufferContent, from_int_point(junction->center), QColor(255, 0, 0));
        vbufferContent += chunk_size;
    }
    _junctions_vertex_buffer->setData(bufferArray);
    _junctions_color_attribute->setCount(num_vertices);
    _junctions_position_attribute->setCount(num_vertices);
}

Qt3DRender::QMaterial *material_from_shader(Qt3DRender::QShaderProgram *shader) {
    auto point_size = new Qt3DRender::QPointSize();
    point_size->setSizeMode(Qt3DRender::QPointSize::Programmable);
    //point_size->setSizeMode(Qt3DRender::QPointSize::Fixed);
    //point_size->setValue(100);
    auto depth_test = new Qt3DRender::QDepthTest();
    depth_test->setDepthFunction(Qt3DRender::QDepthTest::Less);

    auto renderPass = new Qt3DRender::QRenderPass();
    renderPass->addRenderState(point_size);
    renderPass->addRenderState(depth_test);
    renderPass->setShaderProgram(shader);
    auto technique = new Qt3DRender::QTechnique();
    technique->addRenderPass(renderPass);
    auto effect = new Qt3DRender::QEffect();
    effect->addTechnique(technique);
    auto material = new Qt3DRender::QMaterial();
    material->setEffect(effect);
    auto filter = new Qt3DRender::QGraphicsApiFilter(technique);
    filter->setApi(Qt3DRender::QGraphicsApiFilter::OpenGL);
    filter->setProfile(Qt3DRender::QGraphicsApiFilter::CoreProfile);
    filter->setMajorVersion(3);
    filter->setMinorVersion(1);
    return material;
}

Qt3DRender::QMaterial *pointcloud_material() {
    auto shaderProgram = new Qt3DRender::QShaderProgram();
    shaderProgram->setVertexShaderCode(Qt3DRender::QShaderProgram::loadSource(
            QUrl::fromLocalFile("../3D/shaders/pointcloud.vert")));
    shaderProgram->setFragmentShaderCode(Qt3DRender::QShaderProgram::loadSource(
            QUrl::fromLocalFile("../3D/shaders/pointcloud.frag")));

    auto material = material_from_shader(shaderProgram);
    return material;
}


void PolyhedronVisualizer::load_junctions() {
    auto buffer_size = junction_points_drawed_count();
    auto meshGeometry = new Qt3DRender::QGeometry();
    _junctions_vertex_buffer = new Qt3DRender::QBuffer(meshGeometry);
    int chunk_size = 6;
    _junctions_position_attribute = new Qt3DRender::QAttribute(_junctions_vertex_buffer,
                                                               Qt3DRender::QAttribute::defaultPositionAttributeName(),
                                                               Qt3DRender::QAttribute::Float, 3, buffer_size, 0,
                                                               chunk_size * sizeof(float));
    meshGeometry->addAttribute(_junctions_position_attribute);
    _junctions_color_attribute = new Qt3DRender::QAttribute(_junctions_vertex_buffer,
                                                            Qt3DRender::QAttribute::defaultColorAttributeName(),
                                                            Qt3DRender::QAttribute::Float, 3,
                                                            buffer_size, 3 * sizeof(float),
                                                            chunk_size * sizeof(float));
    meshGeometry->addAttribute(_junctions_color_attribute);
    reload_junctions_buffer();

    auto *geometryRenderer = new Qt3DRender::QGeometryRenderer();
    geometryRenderer->setPrimitiveType(Qt3DRender::QGeometryRenderer::Points);
    geometryRenderer->setGeometry(meshGeometry);

    //transform
    auto *point_transform = new Qt3DCore::QTransform();

    //entity
    _junctions_entity = new QEntity(_entity);
    _junctions_entity->addComponent(geometryRenderer);
    _junctions_entity->addComponent(point_transform);

    auto state_set = new Qt3DRender::QRenderStateSet(_junctions_entity);
    auto point_size = new Qt3DRender::QPointSize(state_set);
    point_size->setSizeMode(Qt3DRender::QPointSize::Fixed);
    point_size->setValue(5);
    state_set->addRenderState(point_size);
    //entity->addComponent(state_set);


    auto material = new Qt3DExtras::QPerVertexColorMaterial();
    //auto material = pointcloud_material();
    //material->setAmbient(QColor(255,0,0));
    _junctions_entity->addComponent(material);
    //_skeleton_entity->addComponent(material);
}

void PolyhedronVisualizer::load_skeleton() {
    if (_poly->get_skeleton() == nullptr)
        return;
    Skeleton skeleton = *_poly->get_skeleton();
    auto meshGeometry = new Qt3DRender::QGeometry();
    auto vertexBuffer = new Qt3DRender::QBuffer(meshGeometry);
    QByteArray bufferArray;
    int chunk_size = 3;
    auto num_edges = (uint) boost::num_edges(skeleton);
    uint num_vertices = num_edges * 2;
    bufferArray.resize(chunk_size * num_vertices * sizeof(float));
    auto *vbufferContent = reinterpret_cast<float *>(bufferArray.data());
    boost::graph_traits<Skeleton>::edge_iterator edges_iter, edges_end;
    uint idx = 0;
    for (boost::tie(edges_iter, edges_end) = edges(*_poly->get_skeleton());
         edges_iter != edges_end; ++edges_iter, ++idx) {
        auto point = skeleton[boost::source(*edges_iter, skeleton)].point;
        vbufferContent[idx * 2 * chunk_size] = CGAL::to_double(point.x());
        vbufferContent[idx * 2 * chunk_size + 1] = CGAL::to_double(point.y());
        vbufferContent[idx * 2 * chunk_size + 2] = CGAL::to_double(point.z());
        point = skeleton[boost::target(*edges_iter, skeleton)].point;
        vbufferContent[(idx * 2 + 1) * chunk_size] = CGAL::to_double(point.x());
        vbufferContent[(idx * 2 + 1) * chunk_size + 1] = CGAL::to_double(point.y());
        vbufferContent[(idx * 2 + 1) * chunk_size + 2] = CGAL::to_double(point.z());
    }
    vertexBuffer->setData(bufferArray);
    auto positionAttribute = new Qt3DRender::QAttribute(vertexBuffer,
                                                        Qt3DRender::QAttribute::defaultPositionAttributeName(),
                                                        Qt3DRender::QAttribute::Float, 3,
                                                        num_vertices,
                                                        0,
                                                        chunk_size * sizeof(float));
    meshGeometry->addAttribute(positionAttribute);

    _skeleton_entity = load_lines_entity(meshGeometry);
    auto material = new Qt3DExtras::QPhongMaterial();
    material->setAmbient(QColor(0, 0, 255));
    _skeleton_entity->addComponent(material);

}

void PolyhedronVisualizer::switchWireframe() {
    _wireframe_enabled = not _wireframe_enabled;
    _wireframe_entity->setEnabled(_wireframe_enabled);
}

QEntity *PolyhedronVisualizer::entity() {
    return _entity;
}

PolyhedronVisualizer::~PolyhedronVisualizer() {
    delete _mesh_vertex_buffer;
    delete _junctions_vertex_buffer;
    delete _tips_vertex_buffer;
    delete _alpha_material;
    delete _bad_face_material;
    delete _wireframe_entity;
    delete _skeleton_entity;
    delete _tips_entity;
    delete _junctions_entity;
    delete _picker_entity;
}

void PolyhedronVisualizer::load_tips() {
    auto buffer_size = (uint) (2 * this->_poly->_tips.size());
    auto meshGeometry = new Qt3DRender::QGeometry();
    _tips_vertex_buffer = new Qt3DRender::QBuffer(meshGeometry);
    int chunk_size = 6;
    auto points_position_attribute = new Qt3DRender::QAttribute(_tips_vertex_buffer,
                                                                Qt3DRender::QAttribute::defaultPositionAttributeName(),
                                                                Qt3DRender::QAttribute::Float, 3, buffer_size, 0,
                                                                chunk_size * sizeof(float));
    meshGeometry->addAttribute(points_position_attribute);
    auto points_color_attribute = new Qt3DRender::QAttribute(_tips_vertex_buffer,
                                                             Qt3DRender::QAttribute::defaultColorAttributeName(),
                                                             Qt3DRender::QAttribute::Float, 3,
                                                             buffer_size, 3 * sizeof(float),
                                                             chunk_size * sizeof(float));
    meshGeometry->addAttribute(points_color_attribute);

    QByteArray bufferArray;
    bufferArray.resize((int) (chunk_size * buffer_size * sizeof(float)));
    auto *vbufferContent = reinterpret_cast<float *>(bufferArray.data());
    for (auto &tip : _poly->_tips) {
        auto point = tip->point;
        QColor color = QColor(0, 0, 255);
        add_point_and_color(vbufferContent, point, color);
        vbufferContent += chunk_size;
        add_point_and_color(vbufferContent, point + tip->direction, QColor(0, 0, 0, 0));
        vbufferContent += chunk_size;
    }
    _tips_vertex_buffer->setData(bufferArray);

    auto *pointsRenderer = new Qt3DRender::QGeometryRenderer();
    pointsRenderer->setPrimitiveType(Qt3DRender::QGeometryRenderer::Points);
    pointsRenderer->setGeometry(meshGeometry);

    auto *linesRenderer = new Qt3DRender::QGeometryRenderer();
    linesRenderer->setPrimitiveType(Qt3DRender::QGeometryRenderer::Lines);
    linesRenderer->setGeometry(meshGeometry);

    //transform
    auto *point_transform = new Qt3DCore::QTransform();

    //entity
    _tips_entity = new QEntity(_entity);
    _tips_entity->addComponent(pointsRenderer);
    _tips_entity->addComponent(linesRenderer);
    _tips_entity->addComponent(point_transform);

    auto material = new Qt3DExtras::QPerVertexColorMaterial();
    _tips_entity->addComponent(material);
}

void PolyhedronVisualizer::setFaceColor(Face face, QColor color) {
    _coloring[face] = color;
}

void PolyhedronVisualizer::rotate_to_base(double &next_x, double &min_y, double &max_y) {
    BaseUpResult res = _poly->base_up();
    //cout << res.xsize << " " << res.ysize << endl;
    QMatrix4x4 translate;
    translate.translate(next_x, min_y);
    max_y = max(max_y, min_y + res.ysize);
    next_x += res.xsize;
    _transform->setMatrix(translate * res.transform);
}