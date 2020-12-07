#include <QtGui/QKeyEvent>
#include <Qt3DRender/QCamera>
#include <Qt3DRender/QRenderSettings>
#include <Qt3DExtras/QFirstPersonCameraController>
#include <Qt3DExtras/QForwardRenderer>
#include <Qt3DRender/QGeometry>
#include <Qt3DRender/QMesh>
#include <Qt3DRender/QEffect>
#include <Qt3DRender/QTechnique>
#include <Qt3DRender/QGraphicsApiFilter>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DExtras/QPhongAlphaMaterial>
#include <Qt3DRender/QParameter>
#include <Qt3DRender/QBuffer>
#include <Qt3DRender/QAttribute>
#include <Qt3DRender/QLayer>
#include <Qt3DExtras/QPerVertexColorMaterial>
#include <Qt3DExtras/QTorusMesh>
#include <Qt3DExtras/QSphereMesh>
#include <Qt3DExtras/QCylinderMesh>
#include <Qt3DRender/QObjectPicker>
#include <Qt3DRender/QPickEvent>
#include <Qt3DRender/QPointSize>
#include <Qt3DRender/QRenderStateSet>
#include <Qt3DRender/QRenderSurfaceSelector>
#include <Qt3DRender/QViewport>
#include <Qt3DRender/QLayerFilter>
#include <Qt3DRender/QCameraSelector>
#include <Qt3DRender/QClearBuffers>
#include <Qt3DRender/QPointLight>
#include <Qt3DRender/QTechnique>
#include <Qt3DExtras/QGoochMaterial>
#include <Qt3DExtras/QPlaneMesh>
#include <Qt3DExtras/QOrbitCameraController>
#include <boost/bimap.hpp>
#include <Qt3DRender/QRenderCapture>
#include "Visualizer3D.h"
#include "Polyhedron.h"
#include "../Algorithms/CutAlgorithm.h"
#include "PolyhedronVisualizer.h"

QEntity *add_plane(QEntity *parent, Plane3 plane) {
    auto plane_entity = new QEntity(parent);
    auto mesh = new Qt3DExtras::QPlaneMesh(parent);
    mesh->setWidth(400);
    mesh->setHeight(400);
    plane_entity->addComponent(mesh);
    auto transform = new Qt3DCore::QTransform();
    transform->setRotation(QQuaternion::rotationTo(QVector3D(0, 1, 0), from_vector(plane.orthogonal_vector())));
    transform->setTranslation(from_vector(plane.projection(Point3(0, 0, 0)) - Point3(0, 0, 0)));
    plane_entity->addComponent(transform);
    auto material = new Qt3DExtras::QPhongMaterial();
    material->setAmbient(QColor(128, 128, 128));
    material->setDiffuse(QColor(128, 128, 128));
    material->setShininess(0);
    plane_entity->addComponent(material);
    return plane_entity;
}

Visualizer3D::Visualizer3D(SliceApp *app, bool debug) : _app(app), _debug(debug),
                                                        _selected_point_entity(nullptr),
                                                        _selected_point_enabled(true),
                                                        _selected_point(Polyhedron::UninitializedPoint),
                                                        _selected_visualizer(nullptr),
                                                        _hide_others(false) {
    _root = new QEntity();
    setRootEntity(_root);

    _print_plane = add_plane(_root, Plane3(CGAL::Origin(), Vector3(0, 0, 1)));
    _print_plane->setEnabled(false);
    _in_use.insert(_print_plane);

    auto viewport = new Qt3DRender::QViewport();
    viewport->setNormalizedRect(QRect(0, 0, 1, 1));
    auto surface_selector = new Qt3DRender::QRenderSurfaceSelector(viewport);
    auto layer_filter = new Qt3DRender::QLayerFilter(surface_selector);
    layer_filter->setEnabled(true);
    //layer_filter->addLayer(new QLayer());

    auto camera_selector = new Qt3DRender::QCameraSelector(surface_selector);
    camera_selector->setCamera(camera());
    auto clear_buffer = new Qt3DRender::QClearBuffers(camera_selector);
    clear_buffer->setBuffers(Qt3DRender::QClearBuffers::ColorDepthBuffer);
    clear_buffer->setClearColor(QColor(QRgb(0xffffff)));


    auto state_set = new Qt3DRender::QRenderStateSet(layer_filter);
    auto point_size = new Qt3DRender::QPointSize(state_set);
    point_size->setSizeMode(Qt3DRender::QPointSize::Fixed);
    point_size->setValue(500);
    state_set->addRenderState(point_size);
    viewport->childNodes().append(state_set);

    _capture = new Qt3DRender::QRenderCapture();
    viewport->setParent(_capture);
    setActiveFrameGraph(_capture);

    renderSettings()->pickingSettings()->setPickMethod(Qt3DRender::QPickingSettings::TrianglePicking);

    setup_camera();
    add_selected_point();
    resize(1200, 800);
    setPosition(QPoint(200, 200));
    show();
}

void Visualizer3D::update_light() {
    _light_transform->setTranslation(camera()->position());
}


void Visualizer3D::setup_camera() {
    //cam
    camera()->lens()->setPerspectiveProjection(45.0f, 16.0f / 9.0f, 0.1f, 1000.0f);
    camera()->setPosition(QVector3D(75, 75, -250.0f));
    camera()->setUpVector(QVector3D(0, 1, 0));
    camera()->setViewCenter(QVector3D(75, 75, 75));
    auto camController = new Qt3DExtras::QOrbitCameraController(_root);
    camController->setLookSpeed(500);
    camController->setLinearSpeed(200);
    //camController->setAcceleration(1000);
    //auto camController = new Qt3DExtras::QFirstPersonCameraController(_root);
    _in_use.insert(camController);
    camController->setCamera(camera());
    //light
    auto lightEntity = new Qt3DCore::QEntity(_root);
    _in_use.insert(lightEntity);
    auto *light = new Qt3DRender::QPointLight(lightEntity);
    light->setColor("white");
    light->setIntensity(1);
    lightEntity->addComponent(light);
    _light_transform = new Q3DTransform(lightEntity);
    _light_transform->setTranslation(camera()->position());
    lightEntity->addComponent(_light_transform);
    QObject::connect(camera(), &Qt3DRender::QCamera::positionChanged,
                     this, &Visualizer3D::update_light);

    auto picker_entity = new Qt3DRender::QObjectPicker(_root);
    QObject::connect(picker_entity, &Qt3DRender::QObjectPicker::clicked, this, &Visualizer3D::point_picked_global);
    _root->addComponent(picker_entity);
}


QVector3D from_vector(Vector3 p) {
    return QVector3D(CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z()));
}

void Visualizer3D::point_picked_global(Qt3DRender::QPickEvent *event) {
    QVector3D qlocal = event->localIntersection();
    QVector3D qglobal = event->worldIntersection();
    Point3 local(qlocal.x(), qlocal.y(), qlocal.z());
    Point3 global(qglobal.x(), qglobal.y(), qglobal.z());
    this->point_picked(nullptr, local, global);
}

void Visualizer3D::point_picked(PolyhedronVisualizer *poly, Point3 local, Point3 global) {
    _selected_visualizer = poly;
    cout.precision(std::numeric_limits<double>::max_digits10);
    cout << global.x() << " " << global.y() << " " << global.z() << endl;
    _selected_point = local;
    _selected_point_global = global;
    camera()->setViewCenter(from_vector(global - Point3(CGAL::Origin())));
    draw();
}

QEntity *Visualizer3D::add_lines(QEntity *parent, vector<Segment3> lines) {
    auto meshGeometry = new Qt3DRender::QGeometry();
    auto vertexBuffer = new Qt3DRender::QBuffer(meshGeometry);
    QByteArray bufferArray;
    int chunk_size = 3;
    size_t num_vertices = lines.size() * 2;
    bufferArray.resize(chunk_size * num_vertices * sizeof(float));
    float *vbufferContent = reinterpret_cast<float *>(bufferArray.data());
    for (uint idx = 0; idx < lines.size(); idx += 1) {
        //Point3 point = lines[idx];
        auto point = lines[idx].source();
        vbufferContent[idx * 2 * chunk_size] = CGAL::to_double(point.x());
        vbufferContent[idx * 2 * chunk_size + 1] = CGAL::to_double(point.y());
        vbufferContent[idx * 2 * chunk_size + 2] = CGAL::to_double(point.z());
        //point = lines[(idx+1)%lines.size()];
        point = lines[idx].target();
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
    auto *geometryRenderer = new Qt3DRender::QGeometryRenderer();
    geometryRenderer->setPrimitiveType(Qt3DRender::QGeometryRenderer::Lines);
    geometryRenderer->setGeometry(meshGeometry);

    //material
    auto *material = new Qt3DExtras::QPhongMaterial();
    material->setAmbient(QColor(0, 0, 0));
    //material->setDiffuse(QColor(QRgb(0x928327)));
    //QParameter* parameter = new QParameter("pointSize", 10, material);
    //material->addParameter(parameter);

    //transform
    auto point_transform = new Qt3DCore::QTransform();

    auto lines_entity = new Qt3DCore::QEntity(parent);
    lines_entity->addComponent(geometryRenderer);
    lines_entity->addComponent(material);
    lines_entity->addComponent(point_transform);

    return lines_entity;
}

void add_points(QEntity *parent, Mesh *poly) {
    auto meshGeometry = new Qt3DRender::QGeometry();
    auto vertexBuffer = new Qt3DRender::QBuffer(meshGeometry);
    QByteArray bufferArray;
    int chunk_size = 3;
    bufferArray.resize(chunk_size * poly->num_vertices() * sizeof(float));
    auto vbufferContent = reinterpret_cast<float *>(bufferArray.data());
    uint idx = 0;
    for (auto vertex : poly->vertices()) {
        Point3 point = poly->point(vertex);
        vbufferContent[idx] = CGAL::to_double(point.x());
        vbufferContent[idx + 1] = CGAL::to_double(point.y());
        vbufferContent[idx + 2] = CGAL::to_double(point.z());
        idx += chunk_size;
    }
    vertexBuffer->setData(bufferArray);
    auto *positionAttribute = new Qt3DRender::QAttribute(vertexBuffer,
                                                         Qt3DRender::QAttribute::defaultPositionAttributeName(),
                                                         Qt3DRender::QAttribute::Float, 3,
                                                         poly->num_vertices(),
                                                         0,
                                                         chunk_size * sizeof(float));
    meshGeometry->addAttribute(positionAttribute);


    //rendrer
    auto *geometryRenderer = new Qt3DRender::QGeometryRenderer();
    geometryRenderer->setPrimitiveType(Qt3DRender::QGeometryRenderer::Points);
    geometryRenderer->setGeometry(meshGeometry);

    //material
    auto *material = new Qt3DExtras::QPhongMaterial();
    //material->setAmbient(QColor(0xffffff));
    //material->setDiffuse(QColor(QRgb(0x928327)));
    //QParameter* parameter = new QParameter("pointSize", 10, material);
    //material->addParameter(parameter);

    //transform
    auto point_transform = new Qt3DCore::QTransform();

    auto mesh_points = new Qt3DCore::QEntity(parent);
    mesh_points->addComponent(geometryRenderer);
    mesh_points->addComponent(material);
    mesh_points->addComponent(point_transform);
}

void Visualizer3D::add_selected_point() {
    _selected_point_entity = new Qt3DCore::QEntity(_root);
    _in_use.insert(_selected_point_entity);
    auto point_mesh = new Qt3DExtras::QSphereMesh;
    point_mesh->setRadius(2);
    point_mesh->setRings(100);
    point_mesh->setSlices(20);
    _selected_point_entity->addComponent(point_mesh);
    auto *material = new Qt3DExtras::QPhongMaterial();
    material->setAmbient(QColor::fromRgb(128, 0, 128));
    material->setDiffuse(QColor::fromRgb(128, 0, 128));
    material->setSpecular(QColor(0, 0, 0));
    _selected_point_entity->addComponent(material);
    _selected_point_transform = new Qt3DCore::QTransform();
    //point_transform->setTranslation(from_vector(point - CGAL::Origin()));
    _selected_point_entity->addComponent(_selected_point_transform);
}

PolyhedronVisualizer *Visualizer3D::add_polyhedron(Polyhedron *poly) {
    PolyhedronVisualizer *visualizer = nullptr;
    if (_poly_to_visualizer.find(poly->polyidx()) != _poly_to_visualizer.end()) {
        visualizer = _poly_to_visualizer[poly->polyidx()];
    } else {
        QColor color = QColor(rand() % 128, rand() % 128, rand() % 128);
        visualizer = new PolyhedronVisualizer(poly, this, color, _debug);
        _poly_to_visualizer[poly->polyidx()] = visualizer;
    }
    _in_use.insert(visualizer->entity());
    visualizer->setEnabled(true);
    return visualizer;
}

void deleteRecursively(QEntity *entity) {
    QList<Qt3DCore::QComponent *> componentsToDelete;
    for (auto component : entity->components()) {
        entity->removeComponent(component);
        delete (component);
        component = NULL;
    }
    delete entity;
}


void Visualizer3D::draw() {
    _app->shapes_mutex.lock();
    //add relevant polys and enable them
    for (auto poly_and_visualizer : _poly_to_visualizer) {
        poly_and_visualizer.second->setEnabled(false);
        _in_use.erase(poly_and_visualizer.second->entity());
    }
    if (_app->get_current_shape()) {
        auto visualizer = add_polyhedron(_app->get_current_shape());
        visualizer->setActive(true);
        visualizer->setAlphaColor(QColor(128, 128, 0));
    }
    if (not _hide_others) {
        for (auto good : _app->get_good_shapes()) {
            auto visualizer = add_polyhedron(good);
            visualizer->setActive(false);
            visualizer->switchMaterial(false);
            if (good->merge_base != nullptr) {
                auto parent_color = add_polyhedron(good->merge_base)->getColor();
                visualizer->setSolidColor(parent_color);
            }
        }
        for (auto bad : _app->get_bad_shapes()) {
            auto visualizer = add_polyhedron(bad);
            visualizer->setActive(false);
            visualizer->setAlphaColor(QColor(128, 0, 0));
        }
    }

    //add selection point
    if (_selected_point != Polyhedron::UninitializedPoint and _selected_point_enabled) {
        _selected_point_transform->setTranslation(from_vector(_selected_point_global - CGAL::Origin()));
        _selected_point_entity->setEnabled(true);
    } else {
        _selected_point_entity->setEnabled(false);
    }
    //remove and readd add cuts
    for (auto cut_and_entity : _cuts) {
        cut_and_entity.right->setEnabled(false);
    }
    for (auto id_and_algorithm : _app->get_algorithms()) {
        CutAlgorithm *algorithm = id_and_algorithm.second;
        auto result = algorithm->get_result();
        if (!result)
            continue;

        if (_cuts.left.find(result) == _cuts.left.end()) {
            _cuts.insert(CutsBimap::value_type(result, add_lines(_root, result->cut->segments)));
        }
        auto entity = _cuts.left.find(result)->second;
        entity->setEnabled(true);
    }
    //kill disabled and not explicitly leftover
    for (auto child : _root->childNodes()) {
        if (_in_use.find(child) == _in_use.end() and not child->isEnabled()) {
            //remove from mapping
            map<int, PolyhedronVisualizer *>::iterator to_delete = _poly_to_visualizer.begin();
            while (to_delete != _poly_to_visualizer.end()) {
                if (to_delete->second->entity() == child)
                    break;
                to_delete++;
            }
            if (to_delete != _poly_to_visualizer.end())
                _poly_to_visualizer.erase(to_delete);
            //actually delete
            auto child_entity = (QEntity *) child;
            deleteRecursively(child_entity);
            if (_cuts.right.find(child_entity) != _cuts.right.end()) {
                _cuts.right.erase(child_entity);
            }
        }
    }
    _app->shapes_mutex.unlock();
}


Point3 Visualizer3D::get_selected_point() {
    return _selected_point;
}

void Visualizer3D::set_selected_point(Point3 point) {
    _selected_point = point;
    _selected_point_global = point;
}

QEntity *Visualizer3D::root_entity() {
    return _root;
}

static int cube_radius = 5;

void Visualizer3D::keyPressEvent(QKeyEvent *e) {
    auto int_selected = IntPoint(_selected_point.x(), _selected_point.y(), _selected_point.z());
    if (e->key() == Qt::Key_Y) {
        double next_x = -50, min_y = -100, max_y = -100;
        for (auto v : _poly_to_visualizer) {
            auto poly = v.second->get_poly();
            if (poly->merge_base == nullptr) {
                cout << "Base: " << poly->suggested_base << endl;
                v.second->rotate_to_base(next_x, min_y, max_y);
                if (next_x > 50) {
                    min_y = max_y;
                    next_x = -50;
                }
            }
        }
        for (auto v : _poly_to_visualizer) {
            auto poly = v.second->get_poly();
            if (poly->merge_base != nullptr) {
                QMatrix4x4 mat(_poly_to_visualizer[poly->merge_base->polyidx()]->_transform->matrix());
                v.second->_transform->setMatrix(mat);
            }
        }
    } else if (e->key() == Qt::Key_H) {
        _hide_others = not _hide_others;
    } else if (e->key() == Qt::Key_J) {
        _selected_point_enabled = not _selected_point_enabled;
    } else if (e->key() == Qt::Key_K) {
        _print_plane->setEnabled(not _print_plane->isEnabled());
    }
    //auto shape = this->_app->get_current_shape();
    //if (shape == nullptr) {
    if (_selected_visualizer == nullptr) {
        draw();
        return;
    }
    //auto vis = _poly_to_visualizer[shape->polyidx()];
    auto vis = _selected_visualizer;
    auto shape = vis->get_poly();
    if (e->key() == Qt::Key_M) {
        vis->switchMaterial();
    } else if (e->key() == Qt::Key_N) {
        vis->switchWireframe();
    } else if (e->key() == Qt::Key_B) {
        auto selected_face = shape->face_clamp(_selected_point);
        if (selected_face == Mesh::null_face())
            return;
        shape->suggested_base = shape->infinity_face(selected_face);
        for (auto face : shape->get_mesh()->faces()) {
            auto voilation = shape->violates_base(shape->suggested_base, face);
            if (voilation > 0) {
                if (voilation == 1)
                    vis->setFaceColor(face, QColor(255, 255, 0));
                else
                    vis->setFaceColor(face, QColor(255, 0, 0));
            } else
                vis->setFaceColor(face, vis->getColor());
        }
        vis->reload_vertex_buffer();
    } else if (e->key() == Qt::Key_V) {
        shape->recognize_bases();
        vector<QColor> coloring;
        for (uint i = 0; i < shape->_bases.size(); i++) {
            coloring.emplace_back(QColor(rand() % 128, rand() % 128, rand() % 128));
        }
        for (auto face : shape->get_mesh()->faces()) {
            auto base_idx = shape->_base_map[face];
            if (base_idx == -1 or not shape->_base_face[base_idx].is_valid())
                vis->setFaceColor(face, vis->getColor());
            else
                vis->setFaceColor(face, coloring[base_idx]);
        }
        vis->reload_vertex_buffer();
    } else if (e->key() == Qt::Key_O) {
        cube_radius = max(cube_radius - 1, 1);
        shape->voxel_cube(int_selected, cube_radius);
        vis->reload_junctions_buffer();
        cout << "radius: " << cube_radius << endl;
    } else if (e->key() == Qt::Key_P) {
        cube_radius++;
        shape->voxel_cube(int_selected, cube_radius);
        vis->reload_junctions_buffer();
        cout << "radius: " << cube_radius << endl;
    }
    draw();
}

void Visualizer3D::screenshot(string filename) {
    _capture_reply = _capture->requestCapture();
    _capture_filename = filename;
    QObject::connect(_capture_reply, &Qt3DRender::QRenderCaptureReply::completed,
                     this, &Visualizer3D::capture_complete);
}

void Visualizer3D::capture_complete() {
    _capture_reply->saveImage(_capture_filename.c_str());
    delete _capture_reply;
    _capture_reply = nullptr;
}

Visualizer3D::~Visualizer3D() {
    for (auto poly_and_visualizer : _poly_to_visualizer) {
        delete poly_and_visualizer.second;
    }
    delete _root;
}